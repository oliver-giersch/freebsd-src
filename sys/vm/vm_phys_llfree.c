#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/domainset.h>

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
};

/* 1 LLFree "instance" per domain. */
struct vm_phys_domain {
	domainid_t id;
	vm_paddr_t start;
	vm_paddr_t end;
};

union vm_pgtree {
	struct {
		uint16_t	free     : 15;
		bool		reserved : 1;
	} __packed;
	uint16_t		bits;
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

static struct {
	unsigned len;
	struct vm_pgset *sets;
} pgsets;

static struct vm_phys_domain domains[MAXMEMDOM];
static union vm_pgcount *pgcounts;
static union vm_pgtree *pgtrees;

DPCPU_DEFINE_STATIC(union vm_pcpu_tree, reserved_tree[MAXMEMDOM]);

static vm_pgidx_t
vm_phys_alloc_domain(struct vm_phys_domain *domain, uint8_t order);

static inline unsigned vm_pcpu_tree_get_tree(union vm_pcpu_tree local);
static inline union vm_pcpu_tree vm_pcpu_tree_load(
    const union vm_pcpu_tree *local);
static inline int vm_pcpu_tree_fcmpset(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, union vm_pcpu_tree new);
static bool vm_pcpu_tree_alloc(union vm_pcpu_tree *local,
    union vm_pcpu_tree old, uint16_t pages);
static bool vm_pcpu_tree_alloc_sync(union vm_pcpu_tree *local,
    union vm_pcpu_tree old_local, uint16_t wanted_pgs);

static inline union vm_pgtree vm_pgtree_load(const union vm_pgtree *tree);
static inline int vm_pgtree_fcmpset(union vm_pgtree *tree, union vm_pgtree *old,
    union vm_pgtree new);
static inline bool vm_pgtree_alloc(union vm_pgtree *tree, uint8_t order);

static inline bool vm_pgcount_alloc_order_9(union vm_pgcount *count);
static inline void vm_pgcount_free_order_0to8(union vm_pgcount *count,
    uint8_t order);
static inline void vm_pgcount_free_order_9(union vm_pgcount *count);

static inline unsigned vm_pgset_alloc_order_0(struct vm_pgset *set);
static inline void vm_pgset_alloc_free_0(struct vm_pgset *set, unsigned pg);
static inline unsigned vm_pgset_alloc_order_1to5(struct vm_pgset *set,
    uint8_t order);
static inline unsigned vm_pgset_alloc_order_6to8(struct vm_pgset *set,
    uint8_t order);
static inline void vm_pgset_free(vm_pgidx_t pg);

static vm_pgidx_t
vm_phys_domain_alloc(struct vm_phys_domain *domain, uint8_t order)
{
	union vm_pcpu_tree *local, old;
	uint16_t pages;
	unsigned tree_idx;

	pages = 1 << order;
	critical_enter();
	local = DPCPU_PTR(reserved_tree[domain->id]);
	old = vm_pcpu_tree_load(local);
	if (vm_pcpu_tree_alloc(local, old, pages)) {
		tree_idx = vm_pcpu_tree_get_tree(old);
		// vm_pgtree_alloc_search_order_0to8(...)
		// then, use the offset to get the pgset, and do the allocation there (some fallible, some infallibel!)
		critical_exit();
		// local tree count has been successfully reduced, now we need to sequentially check all 64 pgcounts
		// alloc from tree?
		// local can't change!

	}

exit:
	critical_exit();
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
	/*
	 * - deduce count (by CAS) from local state
	 * - if no longer reserved, fail
	 * - if not enough pages, fail
	 * - deduce count (by CAS) from referenced tree (which must *not* change)
	 */
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

// XXX: pass in uint16_t request to handle the page allocation in one go!
static bool
vm_pcpu_tree_alloc_sync(union vm_pcpu_tree *local, union vm_pcpu_tree old,
    uint16_t wanted_pgs)
{
	union vm_pcpu_tree new;
	union vm_pgtree *tree, old_tree, new_tree;
	uint16_t pages, add_pages;
	bool res;

	CRITICAL_ASSERT(curthread);
	tree = &pgtrees[vm_pcpu_tree_get_tree(old)];
	old_tree = vm_pgtree_load(tree);
	pages = 0;

	// Reset the global tree's freecount to zero.
	do {
		if (!old_tree.reserved || (pages = old_tree.free) == 0)
			return (false);
		new_tree = old_tree;
		new_tree.free = 0;
	} while (vm_pgtree_fcmpset(tree, &old_tree, new_tree));

	if (pages >= wanted_pgs) {
		add_pages = pages - wanted_pgs;
		res = true;
	} else {
		add_pages = pages;
		res = false;
	}

	// Add the acquired number of global tree pages to the local tree.
	do {
		if (!old.reserved)
			goto release;
		new = old;
		new.free += add_pages;
	} while (vm_pcpu_tree_fcmpset(local, &old, new));

	return (res);

release:
	atomic_fetchadd_16(&tree->bits, pages);
	return (0);
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

/*
 * tree's free count has already been decremented, now needs to find a consecutive slice of pages that work
 */
static unsigned
vm_pgtree_alloc_search_order_0to8(union vm_pgcount counts[VM_PGTREE_SETS],
    uint8_t order)
{
	for (unsigned count = 0; count < VM_PGTREE_SETS; count++) {
		if (!vm_pgcount_alloc_order_0to8)
			continue;
		return (count);
	}

	return (-1);
}

static inline bool
vm_pgtree_alloc(union vm_pgtree *tree, uint8_t order)
{
	union vm_pgtree old, new;
	unsigned pages;

	pages = 1 << order;
	old = vm_pgtree_load(tree);
	do {
		if (old.count < pages)
			return (false);
		new = old;
		new.count -= pages;
	} while (vm_pgtree_fcmpset(tree, &old, new));

	return (true);
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
		new.count = old.count + pages;
	} while (atomic_fcmpset_16(&count->bits, &old.bits, new.bits));
}

static inline bool
vm_pgcount_alloc_order_9(union vm_pgcount *count)
{
	union vm_pgcount old, new;

	old.bits = atomic_load_16(&count->bits);
	do {
		if (old.free != VM_PGSET_COUNT)
			return (true);
		new.bits = 0;
	} while (atomic_fcmpset_16(&count->bits, &old.bits, new.bits));

	return (false);
}

static inline void
vm_pgcount_free_order_9(union vm_pgcount *count)
{
	atomic_store_64(&count->bits, VM_PGSET_COUNT);
}

static inline unsigned
vm_pgset_alloc_order0(struct vm_pgset *set)
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

static inline unsigned
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

static inline unsigned
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