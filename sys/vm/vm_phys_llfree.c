#include <sys/cdefs.h>
#include <sys/domainset.h>
#include <sys/types.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/smp.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_phys.h>

#include <machine/atomic.h>
#include <machine/cpufunc.h>
#include <machine/param.h>
#include <machine/vmparam.h>

#define invalid_pgidx (vm_pgidx_t)-1

typedef uint64_t vm_pgset_t;
typedef uint64_t vm_pgidx_t;
typedef unsigned long vm_align_t;

struct {
	struct vm_phys_seg array[VM_PHYSSEG_MAX];
	unsigned len;
} phys_segments __read_mostly;

enum {
	VM_PGSET_WORD	= sizeof(vm_pgset_t) * 8,
	VM_PGSET_ORDER	= VM_LEVEL_0_ORDER,
	VM_PGSET_COUNT	= 1 << VM_PGSET_ORDER,
	VM_PGSET_SIZE	= VM_PGSET_COUNT / VM_PGSET_WORD,
};

enum {
	VM_PGTREE_BITS	= 15,
	VM_PGTREE_COUNT	= 1 << VM_PGTREE_BITS,
	VM_PGTREE_SETS	= VM_PGTREE_COUNT / VM_PGSET_COUNT,

	VM_PGTREE_ALMOST_EMPTY = 4096,
	VM_PGTREE_ALMOST_FULL = VM_PGTREE_COUNT - VM_PGTREE_ALMOST_EMPTY,
};

/* 1 LLFree "instance" per domain. */
struct vm_phys_domain {
	vm_paddr_t start, end;
	uint8_t start_seg, end_seg;
	domainid_t id;
};

union vm_pgtree {
	struct {
		uint16_t	free     : VM_PGTREE_BITS;
		bool		reserved : 1;
	} __packed;
	uint16_t		bits;
};

enum {
	VM_PGTREE_NEIGHBORS	= CACHE_LINE_SIZE / sizeof(union vm_pgtree),
};

_Static_assert(sizeof(union vm_pgtree) == sizeof(uint16_t),
    "vm_pgtree must be 2 bytes");

union vm_pcpu_tree {
	struct {
		uint64_t	last_pgidx : 48;
		uint16_t	free       : VM_PGTREE_BITS;
		bool		reserved   : 1; 
	} __packed;
	uint64_t		bits;
};

_Static_assert(sizeof(union vm_pcpu_tree) == sizeof(uint64_t),
    "vm_pcpu_tree must be 8 bytes");

// XXX(PAVE): why is the allocated required? may be related to free to distinguish order 9 allocations from order 0-8.
union vm_pgcount {
	struct {
		uint16_t	free      : (VM_PGSET_ORDER + 1);
		//bool		allocated : 1;
	} __packed;
	uint16_t		bits;
};

_Static_assert(sizeof(union vm_pgcount) == sizeof(uint16_t),
    "vm_pgcount must be 2 bytes");

struct vm_pgset {
	vm_pgset_t	free[VM_PGSET_SIZE];
};

union vm_pgcount_2x {
	union vm_pgcount	counts[2];
	uint32_t		bits;
};

_Static_assert(sizeof(union vm_pgcount_2x) == sizeof(uint32_t),
    "vm_pgcount_2x must be 4 bytes");

union vm_pgcount_4x {
	union vm_pgcount	counts[4];
	uint64_t		bits;
};

_Static_assert(sizeof(union vm_pgcount_4x) == sizeof(uint64_t),
    "vm_pgcount_4x must be 8 bytes");

static struct {
	unsigned len;
	struct vm_pgset *sets;
} pgsets;

static struct vm_phys_domain domains[MAXMEMDOM];
static union vm_pgcount *pgcounts;
static union vm_pgtree *pgtrees;

DPCPU_DEFINE_STATIC(union vm_pcpu_tree, reserved_tree[MAXMEMDOM]);

/*
 * Allocates 2^O free physical pages.
 */
static vm_pgidx_t
vm_phys_alloc_domain(struct vm_phys_domain *domain, uint8_t order);

static vm_pgidx_t
vm_phys_alloc_domain_contig(struct vm_phys_domain *domain, uint16_t pages,
    vm_paddr_t low, vm_paddr_t high, vm_align_t alignment, vm_paddr_t boundary);

static union vm_pgtree *vm_phys_domain_search_tree(
    struct vm_phys_domain *domain, union vm_pcpu_tree *local,
    union vm_pcpu_tree old, uint16_t pages);

static inline unsigned vm_pcpu_tree_get_tree(union vm_pcpu_tree local);
static inline union vm_pcpu_tree vm_pcpu_tree_load(
    const union vm_pcpu_tree *local);
static inline int vm_pcpu_tree_fcmpset(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, union vm_pcpu_tree new);
static bool vm_pcpu_tree_alloc(union vm_pcpu_tree *local,
    union vm_pcpu_tree old, uint16_t pages);
static bool vm_pcpu_tree_alloc_sync(union vm_pcpu_tree *local,
    union vm_pcpu_tree old_local, uint16_t wanted_pgs);
static vm_pgidx_t vm_pcpu_tree_alloc_search(union vm_pcpu_tree *local,
    union vm_pcpu_tree old, uint8_t order, uint16_t pages);
static vm_pgidx_t vm_pcpu_tree_alloc_search_order_0to9(uint8_t order,
    uint16_t pages, unsigned treeset_idx);
static vm_pgidx_t vm_pcpu_tree_alloc_search_order_10to12(uint8_t order,
    unsigned treeset_idx);
static void vm_pcpu_tree_revert(union vm_pcpu_tree *local,
    union vm_pcpu_tree old, uint16_t pages);
static bool vm_pcpu_tree_unreserve(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old);

static inline union vm_pgtree vm_pgtree_load(const union vm_pgtree *tree);
static inline int vm_pgtree_fcmpset(union vm_pgtree *tree, union vm_pgtree *old,
    union vm_pgtree new);
static inline bool vm_pgtree_sync(union vm_pgtree *tree, uint16_t *pages);
static inline union vm_pgtree vm_pgtree_fetchadd(union vm_pgtree *tree,
    uint16_t pages);
static inline union vm_pgtree *vm_pgtree_get_neighbors(union vm_pgtree *tree);
static union vm_pgtree * vm_pgtree_reserve_search_neighbors(
    union vm_pgtree trees[VM_PGTREE_NEIGHBORS], union vm_pgtree *skip,
    uint16_t min, uint16_t max, uint16_t pages);
static union vm_pgtree * vm_pgtree_reserve_search_all(
    unsigned len, union vm_pgtree all[len],
    union vm_pgtree skip[VM_PGTREE_NEIGHBORS], uint16_t min, uint16_t max,
    uint16_t pages);
static bool vm_pgtree_reserve(union vm_pgtree *tree, uint16_t min, uint16_t max,
    uint16_t pages);
static void vm_pgtree_unreserve(union vm_pgtree *tree, uint16_t pages);

static inline union vm_pgcount vm_pgcount_load(
    volatile union vm_pgcount *count);
static inline int vm_pgcount_fcmpset(volatile union vm_pgcount *count,
    union vm_pgcount *old, union vm_pgcount new);
static int vm_pgcount_alloc_search_order_0to9(
    union vm_pgcount counts[VM_PGTREE_SETS], uint16_t pages);
static int vm_pgcount_alloc_search_order_10(
    union vm_pgcount counts[VM_PGTREE_SETS]);
static int vm_pgcount_alloc_search_order_11(
    union vm_pgcount counts[VM_PGTREE_SETS]);
static int vm_pgcount_alloc_search_order_12(
    union vm_pgcount counts[VM_PGTREE_SETS]);
static inline void vm_pgcount_free_order_0to8(union vm_pgcount *count,
    uint8_t order);
static inline void vm_pgcount_free_order_9(union vm_pgcount *count);
static void vm_pgcount_revert(union vm_pgcount *count, uint16_t pages,
    uint8_t order);

static inline bool vm_pgcount_2x_is_free(union vm_pgcount_2x counts2x);
static union vm_pgcount_2x vm_pgcount_2x_load(union vm_pgcount counts[2]);
static int vm_pgcount_2x_fcmpset(union vm_pgcount counts[2],
    union vm_pgcount_2x *old, union vm_pgcount_2x new);
static inline bool vm_pgcount_4x_is_free(union vm_pgcount_4x counts4x);
static union vm_pgcount_4x vm_pgcount_4x_load(union vm_pgcount counts[4]);
static int vm_pgcount_4x_fcmpset(union vm_pgcount counts[4],
    union vm_pgcount_4x *old, union vm_pgcount_4x new);
static inline bool vm_pgcount_4x_alloc(union vm_pgcount counts[4],
    union vm_pgcount_4x *old);
static void vm_pgcount_4x_revert(union vm_pgcount counts[4]);

static inline unsigned vm_pgset_alloc_order_0(struct vm_pgset *set);
static inline int vm_pgset_alloc_order_1to5(struct vm_pgset *set,
    uint8_t order);
static inline int vm_pgset_alloc_order_6to8(struct vm_pgset *set,
    uint8_t order);
static inline void vm_pgset_free_order_0(struct vm_pgset *set, unsigned pg);
static inline void vm_pgset_free(vm_pgidx_t pg);

static inline bool is_aligned(void *ptr, uint32_t alignment);

static vm_pgidx_t
vm_phys_domain_alloc(struct vm_phys_domain *domain, uint8_t order)
{
	union vm_pcpu_tree *local, old;
	uint16_t pages;
	vm_pgidx_t pg;
	union vm_pgtree *tree;

	pages = 1 << order;
	tree = NULL;
	critical_enter();

	local = DPCPU_PTR(reserved_tree[domain->id]);
	while (true) {
		old = vm_pcpu_tree_load(local);
		if (vm_pcpu_tree_alloc(local, old, pages)) {
			pg = vm_pcpu_tree_alloc_search(local, old, order,
			    pages);
			if (pg != invalid_pgidx)
				goto exit;
		}

		// this could be niceer
		if (tree)
			vm_pgtree_unreserve(tree, pages);
	
		tree = vm_phys_domain_search_tree(domain, local, old, pages);
		if (tree == NULL) {
			critical_exit();
			return (invalid_pgidx);
		}
	}

exit:
	critical_exit();
	return (pg);
}

static vm_pgidx_t
vm_phys_alloc_domain_contig(struct vm_phys_domain *domain, uint16_t pages,
    vm_paddr_t low, vm_paddr_t high, vm_align_t alignment, vm_paddr_t boundary)
{
	/*
	 * - restrict to max O=15?
	 * - check vm_phys_segs, find a suitable segment
	 * - restrict range of pgsets accordingly
	 * - search for a suitable range, i.e. a run of free bits
	 * - then reserve the associated pgtree and allocate from it
	 */
	unsigned i;
	struct vm_phys_seg *seg;
	vm_paddr_t range[2];

	KASSERT(0 < pages && pages <= (1 << 15),
	    ("%s: invalid page count %u", __func__, pages));

	if (low >= high || high <= domain->start || domain->end <= low)
		return (invalid_pgidx);

	for (i = domain->end_seg - 1; i >= domain->start_seg; i--) {
		seg = &phys_segments.array[i];
		if (seg->start >= high)
			break;
		if (low >= seg->end)
			continue;

		range[0] = (low <= seg->start) ? seg->start : low;
		range[1] = (high <= seg->end) ? high : seg->end;

		if (range[1] - range[0] < ptoa(pages))
			continue;
		//order 0-12: regular
		//order 12-_: allocate consecutive blocks of full O15 trees
	}

	return (invalid_pgidx);
}

static union vm_pgtree *
vm_phys_domain_search_tree(struct vm_phys_domain *domain,
    union vm_pcpu_tree *local, union vm_pcpu_tree old, uint16_t pages)
{
	union vm_pgtree *tree, *neighbors;
	union vm_pcpu_tree *remote;
	uint16_t min, max;
	int cpu;

	CRITICAL_ASSERT(curthread);

	/*
	 * First, unreserve the local tree.  If that succeeds, unreserve the
	 * global tree as well and synchronize its free count.
	 */
	tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
	if (vm_pcpu_tree_unreserve(local, &old)) {
		tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
		vm_pgtree_unreserve(tree, old.free);
	}

	// 1) unreserve tree
	// 2) search neighbors
	// 3) search sequentially (may skip neighbor range)
	// 4) search again, this time not skipping almost full/empty trees
	// 5) unreserve all local trees, try to reserve one of those
	// 6) give up

	/*
	 * Search the trees in the immediate neighborhood of the previous tree.
	 * Neighborhood is defined as "on the same cache line".
	 */
	neighbors = vm_pgtree_get_neighbors(tree);
	min = VM_PGTREE_ALMOST_EMPTY;
	max = VM_PGTREE_ALMOST_FULL;

	tree = vm_pgtree_reserve_search_neighbors(
	    neighbors, tree, min, max, pages);
	if (tree)
		return (tree);

	tree = vm_pgtree_reserve_search_neighbors(
	    neighbors, tree, min, 0, pages);
	if (tree)
		return (tree);

	tree = vm_pgtree_reserve_search_neighbors(
	    neighbors, tree, 0, 0, pages);
	if (tree)
		return (tree);

	/*
	 * Search for any free tree in the domain.
	 */
	// ...

	/*
	 * Unreserve all CPU reserved trees.
	 */
	CPU_FOREACH(cpu) {
		if (cpu == curcpu)
			continue;
		remote = DPCPU_ID_PTR(cpu, reserved_tree[domain->id]);
		old = vm_pcpu_tree_load(remote);
		if (vm_pcpu_tree_unreserve(remote, &old)) {
			tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
			vm_pgtree_unreserve(tree, old.free);
		}
	}

	return (NULL);
}

static inline unsigned
vm_pcpu_tree_get_tree(union vm_pcpu_tree local)
{
	return (local.last_pgidx / VM_PGTREE_COUNT);
}

static inline union vm_pcpu_tree
vm_pcpu_tree_load(const union vm_pcpu_tree *local)
{
	union vm_pcpu_tree pcpu_tree;

	CRITICAL_ASSERT(curthread);
	pcpu_tree.bits = atomic_load_64(&local->bits);

	return (pcpu_tree);
}

static inline int
vm_pcpu_tree_fcmpset(union vm_pcpu_tree *local, union vm_pcpu_tree *old,
    union vm_pcpu_tree new)
{
	return (atomic_fcmpset_64(&local->bits, &old->bits, new.bits));
}

// Must allocate less than 2^15 pages (for now)
// contig or noncontig? can't know until looking at the bitmasks ... 
/*
 * must support:
 *   - allocation of N noncontig pages (but may be less, could be capped at 2^15, even better: old impl is also capped at 2^12 pages!)
 *   - allocation of N contig pages (with low/high/alignment/boundary ... -> this is hardest!)
 *   - allocation of O order contig pages (1/2/4/8/16/32/64/.../4096) (Omax = 12)
 * O:
 *   -  O0(1):    cas(tree), cas(count), find  1 free bit
 *   -  O1(2):    cas(tree), cas(count), find  2 (aligned) free bits [may revert]
 *   -  O2(4):    cas(tree), cas(count), find  4 (aligned) free bits [may revert]
 *   -  O3(8):    cas(tree), cas(count), find  8 (aligned) free bits [may revert]
 *   -  O4(16):   cas(tree), cas(count), find 16 (aligned) free bits [may revert]
 *   -  O5(32):   cas(tree), cas(count), find 32 (aligned) free bits [may revert]
 *   -  O6(64):   cas(tree), cas(count), find  1 word w/ all zeroes  [may revert]
 *   -  O7(128):  cas(tree), cas(count), find  2 word w/ all zeroes  [may revert 2x]
 *   -  O8(256):  cas(tree), cas(count), find  4 word w/ all zeroes  [may revert 4x]
 *   -  O9(512):  cas(tree), cas(count/a)
 *   - O10(1024): cas(tree), cas(2 contig counts/a)
 *   - O11(2048): cas(tree), cas(4 contig counts/a)
 *   - O12(4096): cas(tree), ...
 *
 * contig alloc could be restricted to O12 (I've found no case where more than 256 pages requested)
 *   - search pgsets for runs of pages that are within the specified constraints
 *   - then try to reserve the appropriate tree and decrement the free count, then try to do the run of O9 allocations within that tree
 *   - could do more than 1 tree even that way, just make sure to unreserve all previous trees ... could limit it to 4 consecutive trees (single cas, without reserve)
 */
static bool
vm_pcpu_tree_alloc(union vm_pcpu_tree *local, union vm_pcpu_tree old,
    uint16_t pages)
{
	union vm_pcpu_tree new;

	CRITICAL_ASSERT(curthread);
	do {
		/*
		 * The local reservation may have been broken up by another
		 * thread.
		 */
		if (!old.reserved)
			return (false);

		/*
		 * If there are fewer free pages than the requested amount,
		 * try to sync with the global free count for the page tree.
		 */
		if (old.free < pages)
			return (vm_pcpu_tree_alloc_sync(local, old,
			    pages - old.free));

		new = old;
		new.free -= pages;
	} while (vm_pcpu_tree_fcmpset(local, &old, new));

	return (true);
}

/*
 * Searches the CPU reserved page tree for a run of free pages for the requested
 * order.
 *
 * Returns the index of the first page on success.
 */
static vm_pgidx_t
vm_pcpu_tree_alloc_search(union vm_pcpu_tree *local, union vm_pcpu_tree old,
    uint8_t order, uint16_t pages)
{
	unsigned tree_idx, treeset_idx;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);
	KASSERT((1 << order) == pages,
	    ("%s: order %u and pages mismatch: %u", __func__, order, pages));

	tree_idx = vm_pcpu_tree_get_tree(old);
	treeset_idx = tree_idx * VM_PGTREE_COUNT;
	pg = (order <= 9)
	    ? vm_pcpu_tree_alloc_search_order_0to9(order, pages, treeset_idx)
	    : vm_pcpu_tree_alloc_search_order_10to12(order, treeset_idx);

	if (pg == invalid_pgidx) {
		vm_pcpu_tree_revert(local, old, pages);
		return (invalid_pgidx);
	}

	return (pg);
}


static vm_pgidx_t
vm_pcpu_tree_alloc_search_order_0to9(uint8_t order, uint16_t pages,
    unsigned treeset_idx)
{
	int offset;
	unsigned pgset_idx;
	struct vm_pgset *set;

	KASSERT(order <= 9, ("%s: invalid order %u", __func__, order));
	KASSERT((1 << order) == pages,
	    ("%s: order %u and pages mismatch: %u", __func__, order, pages));

	/*
	 * Find a page count to allocate the required run of pages from.
	 */
	offset = vm_pgcount_alloc_search_order_0to9(&pgcounts[treeset_idx],
	    pages);

	/*
	 * An allocation request may succeed in reserving the required number of
	 * pages in a tree, but fail to find a consecutive run of free pages in
	 * the associated page sets, in which case the allocation needs to be
	 * reverted.
	 */
	if (offset == -1)
		return (invalid_pgidx);

	/*
	 * Order 9 allocations don't require modification to the individual page
	 * set bits.
	 */
	pgset_idx = treeset_idx + offset;
	if (order == 9)
		return (pgset_idx * VM_PGSET_COUNT);

	/* 
	 * For orders 0 to 8, try to allocate the desired run of pages from one
	 * of the page sets.  Single page allocations (order 0) are guaranteed
	 * to succeed, but larger ones (order 1 to 8) may fail and require to be
	 * reverted.
	 */
	set = &pgsets.sets[pgset_idx];
	if (order == 0)
		return ((pgset_idx + vm_pgset_alloc_order_0(set))
		    * VM_PGSET_COUNT);

	offset = (order <= 5)
	    ? vm_pgset_alloc_order_1to5(set, order)
	    : vm_pgset_alloc_order_6to8(set, order);
	if (offset == -1) {
		vm_pgcount_revert(&pgcounts[pgset_idx], pages, order);
		return (invalid_pgidx);
	}

	return ((pgset_idx + offset) * VM_PGSET_COUNT);
}

static vm_pgidx_t
vm_pcpu_tree_alloc_search_order_10to12(uint8_t order, unsigned treeset_idx)
{
	int offset;

	KASSERT(9 < order && order < VM_NFREEORDER_MAX,
	    ("%s: invalid order %u", __func__, order));

	switch (order) {
	case 10:
		offset = vm_pgcount_alloc_search_order_10(
		    &pgcounts[treeset_idx]);
	case 11:
		offset = vm_pgcount_alloc_search_order_11(
		    &pgcounts[treeset_idx]);
	case 12:
		offset = vm_pgcount_alloc_search_order_12(
		    &pgcounts[treeset_idx]);
	default:
		__unreachable();
	}

	if (offset == -1)
		return (invalid_pgidx);

	return (treeset_idx + offset * VM_PGSET_COUNT);
}

static bool
vm_pcpu_tree_alloc_sync(union vm_pcpu_tree *local, union vm_pcpu_tree old,
    uint16_t wanted_pgs)
{
	union vm_pcpu_tree new;
	union vm_pgtree *tree;
	uint16_t pages, new_pages;
	bool res;

	CRITICAL_ASSERT(curthread);

	/*
	 * Reset the global tree's freecount to zero.
	 */
	tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
	if (!vm_pgtree_sync(tree, &pages))
		return (false);

	/*
	 * The synchronization with the global page tree is triggered by an
	 * allocation request.  To reduce the number of atomic operations, if
	 * possible, do the synchronization and free page reservation in a
	 * single step.
	 */
	if (old.free + pages >= wanted_pgs) {
		new_pages = old.free + pages - wanted_pgs;
		res = true;
	} else {
		new_pages = old.free + pages;
		res = false;
	}

	/*
	 * Add the acquired number of global tree pages to the local tree.
	 */
	do {
		/*
		 * Abort the attempt and release the acquired pages back to the
		 * global tree, if the local tree has been un-reserved.  It is
		 * not possible for another thread to change the free count
		 * without also unreserving it.
		 */
		if (!old.reserved)
			goto release;
		new = old;
		new.free = new_pages;
	} while (vm_pcpu_tree_fcmpset(local, &old, new));

	return (res);

release:
	atomic_fetchadd_16(&tree->bits, pages);
	return (0);
}

static void
vm_pcpu_tree_revert(union vm_pcpu_tree *local, union vm_pcpu_tree old,
    uint16_t pages)
{
	union vm_pcpu_tree new;
	union vm_pgtree *tree;

	CRITICAL_ASSERT(curthread);

	while (true) {
		/*
		 * If the local tree has been un-reserved by another thread,
		 * the (reverted) free pages need to be added to the global free
		 * tree's free count.
		 */
		if (!old.reserved)
			break;
		new = old;
		new.free += pages;

		if (!vm_pcpu_tree_fcmpset(local, &old, new))
			return;
	}

	tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
	(void)vm_pgtree_fetchadd(tree, pages);
}

static bool
vm_pcpu_tree_unreserve(union vm_pcpu_tree *local, union vm_pcpu_tree *old)
{
	union vm_pcpu_tree new;

	CRITICAL_ASSERT(curthread);
	//sKASSERT(old->reserved, ("%s: local tree not reserved", __func__));

	do {
		/*
		 * Another thread may concurrently have unreserved the tree,
		 * in which case it would also have zeroed the (local) free
		 * count and assumed responsibility for also unreserving the
		 * global tree.
		 */
		if (!old->reserved)
			return (false);
		new = *old;
		new.reserved = false;
		new.free = 0;
	} while (vm_pcpu_tree_fcmpset(local, old, new));
}

static inline union vm_pgtree
vm_pgtree_load(const union vm_pgtree *tree)
{
	union vm_pgtree val;

	val.bits = atomic_load_16(&tree->bits);

	return (val);
}

static inline int
vm_pgtree_fcmpset(union vm_pgtree *tree, union vm_pgtree *old,
    union vm_pgtree new)
{
	return (atomic_fcmpset_16(&tree->bits, &old->bits, new.bits));
}

static inline union vm_pgtree
vm_pgtree_fetchadd(union vm_pgtree *tree, uint16_t pages)
{
	union vm_pgtree val;

	val.bits = atomic_fetchadd_16(&tree->bits, pages);

	return (val);
}

static inline union vm_pgtree *
vm_pgtree_get_neighbors(union vm_pgtree *tree)
{
	return ((union vm_pgtree *)((uintptr_t)tree & (CACHE_LINE_SIZE - 1)));
}

static union vm_pgtree *
vm_pgtree_reserve_search_neighbors(
    union vm_pgtree neighbors[VM_PGTREE_NEIGHBORS], union vm_pgtree *skip,
    uint16_t min, uint16_t max, uint16_t pages)
{
	union vm_pgtree *tree;

	for (unsigned i = 0; i < VM_PGTREE_NEIGHBORS; i++) {
		tree = &neighbors[i];
		if (tree == skip)
			continue;
		if (!vm_pgtree_reserve(tree, min, max, pages))
			continue;
		return (tree);
	}

	return (NULL);
}

static union vm_pgtree *
vm_pgtree_reserve_search_all(unsigned len, union vm_pgtree all[len],
    union vm_pgtree skip[VM_PGTREE_NEIGHBORS], uint16_t min, uint16_t max,
    uint16_t pages)
{
	return (NULL);
}

static bool
vm_pgtree_reserve(union vm_pgtree *tree, uint16_t min, uint16_t max,
    uint16_t pages)
{
	union vm_pgtree old, new;

	old = vm_pgtree_load(tree);
	do {
		if (old.reserved)
			return (false);
		if (old.free < pages)
			return (false);
		if (max && old.free > max)
			return (false);
		if (old.free < min)
			return (false);

		new = old;
		new.free = old.free - pages;
		new.reserved = true;
	} while (vm_pgtree_fcmpset(tree, &old, new));

	return (true);
}

/*
 * Who can unreserve a tree?
 *   - the owning CPU, if it can't find a free tree
 *   - any other CPU *without* a currently reserved tree
 *
 * 1) unreserve the local tree, zero its free count (!)
 * 2) the thread which "did the deed" *has* to unreserve the global one as well (?)
 * 3) better: any thread can do it, but can't because of ABA issues -> not lockfree,
 *    but this is, in the end, not critical, it just means the tree may not be
 *    available for a while and lead to "minor starvation" and OOM too early in extremely rare cases
 */
static void
vm_pgtree_unreserve(union vm_pgtree *tree, uint16_t pages)
{
	union vm_pgtree old, new;

	old = vm_pgtree_load(tree);
	do {
		/*
		 * Another thread may unreserve the tree, but this thread *must*
		 * release the previously reserved free page count.
		 * D'oh: ABA problem persists!
		 * It may be possible to "accidentally" unreserve a tree that is
		 * actually reserved by some other thread. this can, in fact
		 * lead to a cascade, where the threads do not agree on who
		 * reserves which reserves which tree.
		 * Lars et al. avoid ABA by making this stuff not lock-free,
		 * damn cheaters!
		 * Not really possible to avoid with this small amount of bits,
		 * would need to store the cpu id as well, increasing each pgcount
		 * to 32-bits
		 */
		new = old;
		new.free += pages;
		new.reserved = false;
	} while (vm_pgtree_fcmpset(tree, &old, new));
}

static inline bool
vm_pgtree_sync(union vm_pgtree *tree, uint16_t *pages)
{
	union vm_pgtree old, new;
	
	old = vm_pgtree_load(tree);
	do {
		if (!old.reserved || old.free == 0)
			return (false);
		new = old;
		new.free = 0;
	} while (vm_pgtree_fcmpset(tree, &old, new));

	*pages = old.free;
	return (true);
}

static inline union vm_pgcount
vm_pgcount_load(volatile union vm_pgcount *count)
{
	union vm_pgcount val;

	val.bits = atomic_load_16(&count->bits);
	return (val);
}

static inline int
vm_pgcount_fcmpset(volatile union vm_pgcount *count, union vm_pgcount *old,
    union vm_pgcount new)
{
	return (atomic_fcmpset_16(&count->bits, &old->bits, new.bits));
}

static int
vm_pgcount_alloc_search_order_0to9(union vm_pgcount counts[VM_PGTREE_SETS],
    uint16_t pages)
{
	union vm_pgcount old, new;
	
	for (unsigned i = 0; i < VM_PGTREE_SETS; i++) {
		old = vm_pgcount_load(&pgcounts[i]);
		while (true) {
			if (old.free < pages)
				break;
			new = old;
			new.free -= pages;

			if (!vm_pgcount_fcmpset(&pgcounts[i], &old, new))
				return (i);
		}
	}

	return (-1);
}

static int
vm_pgcount_alloc_search_order_10(union vm_pgcount counts[VM_PGTREE_SETS])
{
	union vm_pgcount_2x old, new;

	KASSERT(is_aligned(counts, 4),
	    ("%s: invalid counts %p alignment (expected 4)", __func__, counts));

	for (unsigned i = 0; i < VM_PGTREE_SETS; i += 2) {
		old = vm_pgcount_2x_load(&counts[i]);
		while (true) {
			if (!vm_pgcount_2x_is_free(old))
				break;
			new = old;
			new.counts[0].free = new.counts[1].free = 0;

			if (!vm_pgcount_2x_fcmpset(&pgcounts[i], &old, new))
				return (i);
		}
	}

	return (-1);
}

static int
vm_pgcount_alloc_search_order_11(union vm_pgcount counts[VM_PGTREE_SETS])
{
	union vm_pgcount_4x old, new;

	KASSERT(is_aligned(counts, 8),
	    ("%s: invalid counts %p alignment (expected 8)", __func__, counts));

	for (unsigned i = 0; i < VM_PGTREE_SETS; i += 4) {
		old = vm_pgcount_4x_load(&counts[i]);
		while (true) {
			if (!vm_pgcount_4x_is_free(old))
				break;
			new = old;
			new.counts[0].free = new.counts[1].free = 0;
			new.counts[2].free = new.counts[3].free = 0;

			if (!vm_pgcount_4x_fcmpset(&counts[i], &old, new))
				return (i);
		}
	}

	return (-1);
}

static int
vm_pgcount_alloc_search_order_12(union vm_pgcount counts[VM_PGTREE_SETS])
{
	union vm_pgcount_4x old1, old2;

	KASSERT(is_aligned(counts, 8),
	    ("%s: invalid counts %p alignment (expected 8)", __func__, counts));

	for (unsigned i = 0; i < VM_PGTREE_SETS; i += 4) {
		old1 = vm_pgcount_4x_load(&counts[i]);
		old2 = vm_pgcount_4x_load(&counts[i + 4]);

		if (!vm_pgcount_4x_is_free(old1)
		    || vm_pgcount_4x_is_free(old2))
			continue;

		if (!vm_pgcount_4x_alloc(&counts[i], &old1))
			continue;
		if (!vm_pgcount_4x_alloc(&counts[i + 4], &old2)) {
			vm_pgcount_4x_revert(&counts[i]);
			continue;
		}

		return (i);
	}

	return (-1);
}

static inline void
vm_pgcount_free_order_0to8(union vm_pgcount *count, uint8_t order)
{
	uint16_t pages;
	union vm_pgcount old, new;
	
	pages = 1 << order;
	old.bits = atomic_load_16(&count->bits);
	do {
		new = old;
		new.free = old.free + pages;
	} while (atomic_fcmpset_16(&count->bits, &old.bits, new.bits));
}

static inline void
vm_pgcount_free_order_9(union vm_pgcount *count)
{
	atomic_store_16(&count->bits, VM_PGSET_COUNT);
}

static void
vm_pgcount_revert(union vm_pgcount *count, uint16_t pages, uint8_t order)
{
	if (order == 9)
		atomic_store_16(&count->bits, pages);
	else
		atomic_add_16(&count->bits, pages);
}

static inline bool
vm_pgcount_2x_is_free(union vm_pgcount_2x counts2x)
{
	return (2 * VM_PGSET_COUNT == counts2x.counts[0].free
	    + counts2x.counts[1].free);
}

static union vm_pgcount_2x
vm_pgcount_2x_load(union vm_pgcount counts[2])
{
	union vm_pgcount_2x val;

	val.bits = atomic_load_32((uint32_t *)counts);

	return (val);
}

static int
vm_pgcount_2x_fcmpset(union vm_pgcount counts[2], union vm_pgcount_2x *old,
    union vm_pgcount_2x new)
{
	return (atomic_fcmpset_32((uint32_t *)counts, &old->bits, new.bits));
}

static bool
vm_pgcount_4x_is_free(union vm_pgcount_4x counts4x)
{
	return (4 * VM_PGSET_COUNT == counts4x.counts[0].free
	    + counts4x.counts[1].free
	    + counts4x.counts[2].free
	    + counts4x.counts[3].free);
}

static union vm_pgcount_4x
vm_pgcount_4x_load(union vm_pgcount counts[4])
{
	union vm_pgcount_4x val;

	val.bits = atomic_load_64((uint64_t *)counts);

	return (val);
}

static int
vm_pgcount_4x_fcmpset(union vm_pgcount counts[4], union vm_pgcount_4x *old,
    union vm_pgcount_4x new)
{
	return (atomic_fcmpset_64((uint64_t *)counts, &old->bits, new.bits));
}

static inline bool
vm_pgcount_4x_alloc(union vm_pgcount counts[4], union vm_pgcount_4x *old)
{
	union vm_pgcount_4x new;

	KASSERT(is_aligned(counts, 8),
	    ("%s: invalid counts %p alignment (expected 8)", __func__, counts));

	while (true) {
		if (!vm_pgcount_4x_is_free(*old))
			return (false);

		new = *old;
		new.counts[0].free = new.counts[1].free = 0;
		new.counts[2].free = new.counts[3].free = 0;

		if (!vm_pgcount_4x_fcmpset(counts, old, new))
			return (true);
	}
}

static void
vm_pgcount_4x_revert(union vm_pgcount counts[4])
{
	KASSERT(is_aligned(counts, 8),
	    ("%s: invalid counts %p alignment (expected 8)", __func__, counts));

	for (unsigned i = 0; i < 4; i++)
		atomic_store_16(&counts[i].bits, VM_PGSET_COUNT);
}

static inline unsigned
vm_pgset_alloc_order_0(struct vm_pgset *set)
{
	vm_pgset_t bits;
	uint16_t bit;

restart:
	for (unsigned i = 0; i < VM_PGSET_SIZE; i++) {
retry:
		bits = atomic_load_64(&set->free[i]);
		if ((bit = bsfl(~bits)) == 64)
			continue;
		if (atomic_testandset_64(&set->free[i], bit))
			goto retry;
		atomic_thread_fence_acq_rel();
		return (i * VM_PGSET_WORD + bit);
	}

	goto restart;
}

static inline int
vm_pgset_alloc_order_1to5(struct vm_pgset *set, uint8_t order)
{
	unsigned pages;
	vm_pgset_t mask, bits;

	pages = 1 << order;
	mask = pages - 1;

	for (unsigned i = 0; i < VM_PGSET_SIZE; i++) {
		bits = atomic_load_64(&set->free[i]);
		for (int shift = 0; shift < 64 / pages; shift++,
		    mask <<= pages) {
			while (true) {
				if ((bits & mask) != 0)
					break;

				if (!atomic_fcmpset_64(&set->free[i], &bits,
				    bits | mask))
					return (shift * pages);
			}
		}
	}

	return (-1);
}

static inline int
vm_pgset_alloc_order_6to8(struct vm_pgset *set, uint8_t order)
{
	unsigned words, word;
	vm_pgset_t bits;

	words = (1 << order) / 64;

	for (unsigned i = 0; i < VM_PGSET_SIZE; i += words) {
		for (word = 0; word < words; word++) {
			bits = atomic_load_64(&set->free[i + word]);
			do {
				if (bits != 0)
					goto revert;
			} while (atomic_fcmpset_64(&set->free[i + word], &bits,
			    ~(vm_pgset_t)0));
		}

		return (i);

revert:
		for (unsigned w = 0; w < word; w++)
			atomic_store_64(&set->free[i + w], 0);
		continue;
	}

	return (-1);
}

static inline void
vm_pgset_free_order_0(struct vm_pgset *set, unsigned pg)
{
	unsigned word, bit;

	word = pg / VM_PGSET_WORD;
	bit = pg & (VM_PGSET_WORD - 1);
	atomic_testandclear_64(&set->free[word], bit);
}

static inline void
vm_pgset_free(vm_pgidx_t pg)
{
	unsigned set_idx, word, bit;
	struct vm_pgset *set;

	set_idx = pg / VM_PGSET_COUNT;
	KASSERT(set_idx < pgsets.len,
	    ("%s: invalid page index %u", __func__, set_idx));
	set = &pgsets.sets[set_idx];
	word = pg & ((VM_PGSET_COUNT - 1)) / VM_PGSET_WORD;
	bit = 1 << (pg & (VM_PGSET_WORD - 1));
	atomic_clear_64(&set->free[word], bit); // XXX(PAVE): pg / ???
	atomic_thread_fence_acq_rel();
}

static inline bool
is_aligned(void *ptr, uint32_t alignment)
{
	KASSERT(alignment == 0,
	    ("%s: invalid alignment %u", __func__, alignment));
	KASSERT(powerof2(alignment),
	    ("%s: invalid alignment %u", __func__, alignment));
	return (((uintptr_t)ptr & (alignment - 1)) == 0);
}