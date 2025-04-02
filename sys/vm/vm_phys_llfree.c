#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/smp.h>
#include <sys/domainset.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>

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
#define pgidx_mask (((vm_pgidx_t)1 << 48) - 1)

typedef uint64_t vm_pgset_t;
typedef uint64_t vm_pgidx_t;
typedef uint32_t vm_align_t;

#ifdef VM_FREELIST_DMA32
#define VM_DMA32_BOUNDARY ((vm_paddr_t)1 << 32)
#endif /* VM_FREELIST_DMA32 */

/*
 * Enforce the assumptions made by vm_phys_add_seg() and vm_phys_init() about
 * the ordering of the free list boundaries.
 */
#if defined(VM_LOWMEM_BOUNDARY) && defined(VM_DMA32_BOUNDARY)
CTASSERT(VM_LOWMEM_BOUNDARY < VM_DMA32_BOUNDARY);
#endif /* VM_LOWMEM_BOUNDARY && VM_DMA32_BOUNDARY */

/*
 * These "avail lists" are globals used to communicate boot-time physical
 * memory layout to other parts of the kernel.  Each physically contiguous
 * region of memory is defined by a start address at an even index and an
 * end address at the following odd index.  Each list is terminated by a
 * pair of zero entries.
 *
 * dump_avail tells the dump code what regions to include in a crash dump, and
 * phys_avail is all of the remaining physical memory that is available for
 * the vm system.
 *
 * Initially dump_avail and phys_avail are identical.  Boot time memory
 * allocations remove extents from phys_avail that may still be included
 * in dumps.
 */
vm_paddr_t phys_avail[PHYS_AVAIL_COUNT];
vm_paddr_t dump_avail[PHYS_AVAIL_COUNT];

struct {
	struct vm_phys_seg array[VM_PHYSSEG_MAX];
	unsigned len;
	bool initialized;
} phys_segments __read_mostly;

struct {
	struct vm_phys_seg array[8];
	unsigned len;
} phys_early_segments __read_mostly;

#ifdef NUMA
const struct mem_affinity __read_mostly *mem_affinity;
int __read_mostly *mem_locality;
#endif /* NUMA */

int __read_mostly vm_ndomains = 1;
domainset_t __read_mostly all_domains = DOMAINSET_T_INITIALIZER(0x1);

/*
 * Ficticious Physical Memory Segments.
 */

struct vm_phys_fictitious_seg;
static int vm_phys_fictitious_cmp(struct vm_phys_fictitious_seg *,
    struct vm_phys_fictitious_seg *);

RB_HEAD(fict_tree, vm_phys_fictitious_seg) vm_phys_fictitious_tree =
    RB_INITIALIZER(&vm_phys_fictitious_tree);

struct vm_phys_fictitious_seg {
	RB_ENTRY(vm_phys_fictitious_seg) node;
	/* Memory region data */
	vm_paddr_t	start, end;
	struct vm_page	*first_page;
};

RB_GENERATE_STATIC(fict_tree, vm_phys_fictitious_seg, node,
    vm_phys_fictitious_cmp);

static struct rwlock_padalign vm_phys_fictitious_reg_lock;
MALLOC_DEFINE(M_FICT_PAGES, "vm_fictitious", "Fictitious VM pages");

/* Utility functions. */

static inline bool is_aligned(void *ptr, uint32_t alignment);

/*
 * Physical segment initialization.
 */

static void vm_phys_create_seg(vm_paddr_t start, vm_paddr_t end);
static void vm_phys_create_seg_domain(vm_paddr_t start, vm_paddr_t end,
    int domain);

static int vm_phys_avail_count(void);
static void vm_phys_avail_check(int i);
static int vm_phys_avail_split(vm_paddr_t pa, int i);
#ifdef NUMA
static int vm_phys_avail_find(vm_paddr_t pa);
#endif /* NUMA */

static void vm_phys_domain_init(vm_offset_t *va);
static void vm_phys_domain_init_segments(void);

int
vm_phys_avail_largest(void)
{
	vm_paddr_t sz, largesz;
	int largest;

	largest = 0;
	largesz = 0;
	for (int i = 0; phys_avail[i + 1]; i += 2) {
		sz = vm_phys_avail_size(i);
		if (sz > largesz) {
			largesz = sz;
			largest = i;
		}
	}

	return (largest);
}

vm_paddr_t
vm_phys_avail_size(int i)
{
	return (phys_avail[i + 1] - phys_avail[i]);
}

void
vm_phys_early_startup(void)
{
	struct vm_phys_seg *seg;
	int idx;

	for (int i = 0; phys_avail[i + 1] != 0; i += 2) {
		phys_avail[i] = round_page(phys_avail[i]);
		phys_avail[i + 1] = trunc_page(phys_avail[i + 1]);
	}

	for (int i = 0; i < phys_early_segments.len; i++) {
		seg = &phys_early_segments.array[i];
		vm_phys_add_seg(seg->start, seg->end);
	}

	/*
	 * Disable early segment allocation.
	 */
	phys_early_segments.len = -1;

#ifdef NUMA
	if (mem_affinity == NULL)
		return;

	for (int i = 0; i < mem_affinity[i].end != 0; i++) {
		idx = vm_phys_avail_find(mem_affinity[i].start);
		if (idx != -1)
			vm_phys_avail_split(mem_affinity[i].start, idx);

		idx = vm_phys_avail_find(mem_affinity[i].end);
		if (idx != -1)
			vm_phys_avail_split(mem_affinity[i].end, idx);
	}
#endif /* NUMA */
	(void)idx;
}

void
vm_phys_early_add_seg(vm_paddr_t start, vm_paddr_t end)
{
	struct vm_phys_seg *seg;

	if (phys_early_segments.len == -1)
		panic("%s: called after initialization", __func__);
	if (phys_early_segments.len == nitems(phys_early_segments.array))
		panic("%s: ran out of early segments", __func__);

	seg = &phys_early_segments.array[phys_early_segments.len++];
	seg->start = start;
	seg->end = end;
}

vm_paddr_t
vm_phys_early_alloc(int domain, size_t alloc_size)
{
	int largest;
	vm_paddr_t pa, start, end, size, largestsz, align;
#ifdef NUMA
	int mem_idx;
#endif /* NUMA */

	KASSERT(domain == -1 || (domain >= 0 && domain < vm_ndomains),
	    ("%s: invalid domain index %d", __func__, domain));

	largestsz = 0;
	start = 0;
	end = -1;

#ifdef NUMA
	if (mem_affinity != NULL) {
		mem_idx = 0;
		for (int i = 0;; i++) {
			size = mem_affinity[i].end - mem_affinity[i].start;
			if (size == 0)
				break;
			if (domain != -1 && mem_affinity[i].domain != domain)
				continue;
			if (size > largestsz) {
				mem_idx = i;
				largestsz = size;
			}
		}

		start = mem_affinity[mem_idx].start;
		end = mem_affinity[mem_idx].end;
	}
#endif /* NUMA */

	/*
	 * Now find the largest physical segment in within the desired NUMA
	 * domain.
	 */
	largest = 0;
	largestsz = 0;
	for (int i = 0; phys_avail[i + 1]; i += 2) {
		/* Skip out-of-range segments */
		if (phys_avail[i + 1] - alloc_size < start
		    || phys_avail[i + 1] > end)
			continue;
	
		if ((size = vm_phys_avail_size(i)) > largestsz) {
			largest = i;
			largestsz = size;
		}
	}

	alloc_size = round_page(alloc_size);

	/*
	 * Grab single pages from the front to reduce fragmentation.
	 */
	if (alloc_size == PAGE_SIZE) {
		pa = phys_avail[largest];
		phys_avail[largest] += PAGE_SIZE;
		vm_phys_avail_check(largest);
		return (pa);
	}

	/*
	 * Naturally align large allocations.
	 */
	align = phys_avail[largest + 1] & (alloc_size - 1);
	if (alloc_size + align > largestsz)
		panic("%s: cannot find a large enough size\n", __func__);
	if (align != 0
	    && vm_phys_avail_split(phys_avail[largest + 1] - align, largest))
		/* Wasting memory. */
		phys_avail[largest + 1] -= align;

	phys_avail[largest + 1] -= align;
	vm_phys_avail_check(largest);
	pa = phys_avail[largest + 1];
	return (pa);
}

void
vm_phys_add_seg(vm_paddr_t start, vm_paddr_t end)
{
	vm_paddr_t pa;

	if (!is_aligned((void *)start, PAGE_SIZE))
		panic("%s: start (%jx) is not page aligned", __func__,
		   (uintmax_t)start);
	if (!is_aligned((void *)end, PAGE_SIZE))
		panic("%s: end (%jx) is not page aligned", __func__,
		    (uintmax_t)end);
	if (start > end)
		panic("%s: start (%jx) > end (%jx)", __func__, (uintmax_t)start,
		    (uintmax_t)end);

	if (start == end)
		return;

	/*
	 * Split the physical memory segment if it spans two or more free list
	 * boundaries.
	 */
	pa = start;

#ifdef VM_FREELIST_LOWMEM
	if (pa < VM_LOWMEM_BOUNDARY && end > VM_LOWMEM_BOUNDARY) {
		vm_phys_create_seg(pa, VM_LOWMEM_BOUNDARY);
		pa = VM_LOWMEM_BOUNDARY;
	}
#endif /* VM_FREELIST_LOWMEM */

#ifdef VM_FREELIST_DMA32
	if (pa < VM_DMA32_BOUNDARY && end > VM_DMA32_BOUNDARY) {
		vm_phys_create_seg(pa, VM_DMA32_BOUNDARY);
		pa = VM_DMA32_BOUNDARY;
	}
#endif /* VM_FREELIST_LOWMEM */

	vm_phys_create_seg(pa, end);
}

struct vm_page *
vm_phys_seg_paddr_to_vm_page(const struct vm_phys_seg *seg, vm_paddr_t pa)
{
	KASSERT(pa >= seg->start && pa < seg->end,
	    ("%s: pa %#jx is out of range", __func__, (uintmax_t)pa));

	return (&seg->first_page[atop(pa - seg->start)]);
}

void
vm_phys_init(vm_offset_t *va)
{
	/* Allocate memory for the physical page allocator. */
	vm_phys_domain_init(va);

	/*
	 * Initialize the physical memory segments.
	 *
	 * Add physical memory segments corresponding to the available
	 * physical pages.
	 */
	for (int i = 0; phys_avail[i + 1] != 0; i += 2)
		vm_phys_add_seg(phys_avail[i], phys_avail[i + 1]);
	phys_segments.initialized = true;

	/*
	 * Initialize the physical page allocator bitmasks and free page
	 * counters for each domain and its physical memory segments.
	 */
	vm_phys_domain_init_segments();
}

static void
vm_phys_create_seg(vm_paddr_t start, vm_paddr_t end)
{
#ifdef NUMA
	const struct mem_affinity *ma;

	if (mem_affinity == NULL) {
		vm_phys_create_seg_domain(start, end, 0);
		return;
	}

	for (int i = 0;; i++) {
		ma = &mem_affinity[i];
		if (ma->end == 0)
			panic("%s: reached end of affinity info", __func__);
		if (ma->end <= start)
			continue;
		if (ma->start > start)
			panic("%s: no affinity info for start %jx", __func__,
			    (uintmax_t)start);
		if (ma->end >= end) {
			vm_phys_create_seg_domain(start, end, ma->domain);
			break;
		}

		vm_phys_create_seg_domain(start, ma->end, ma->domain);
		start = ma->end;
	}
#else
	vm_phys_create_seg_domain(start, end, 0);
#endif /* NUMA */
}

static void
vm_phys_create_seg_domain(vm_paddr_t start, vm_paddr_t end, int domain)
{
	struct vm_phys_seg *seg;

	if (phys_segments.initialized)
		panic("%s: must not create further segments after the physical "
		    "memory allocator has been initialized", __func__);
	if (domain < 0 || domain >= vm_ndomains)
		panic("%s: invalid domain %d (max = %d)", __func__, domain,
		    vm_ndomains);
	if (phys_segments.len >= VM_PHYSSEG_MAX)
		panic("%s: not enough storage for physical segments, "
		    "increase VM_PHYSSEG_MAX", __func__);

	
	seg = &phys_segments.array[phys_segments.len++];
	while (seg > phys_segments.array && seg[-1].start >= end) {
		*seg = *(seg - 1);
		seg--;
	}

	seg->start = start;
	seg->end = end;
	seg->domain = domain;

	if (seg != phys_segments.array && seg[-1].end > start)
		panic("%s: overlapping physical segments: Current [%#jx,%#jx) "
		    "at index %zu, previous [%#jx,%#jx)", __func__,
		    (uintmax_t)start, (uintmax_t)end, seg - phys_segments.array,
		    (uintmax_t)seg[-1].start, (uintmax_t)seg[-1].end);
}

/*
 * Return the index of the first unused slot which may be the terminating
 * entry.
 */
static int
vm_phys_avail_count(void)
{ 
	for (int i = 0; i < PHYS_AVAIL_COUNT; i += 2)
		if (phys_avail[i] == 0 && phys_avail[i + 1] == 0)
			return (i);
 
	panic("%s: improperly terminated phys_avail[]", __func__);
}

/*
 * Assert that a phys_avail entry is valid.
 */
static void
vm_phys_avail_check(int i)
{
	if (i % 2 != 0)
		panic("%s: chunk start index %d is not even.", __func__, i);
	if (phys_avail[i] & PAGE_MASK)
		panic("%s: unaligned phys_avail[%d]: %#jx", __func__, i,
		    (intmax_t)phys_avail[i]);
	if (phys_avail[i + 1] & PAGE_MASK)
		panic("%s: unaligned phys_avail[%d + 1]: %#jx", __func__, i,
		    (intmax_t)phys_avail[i + 1]);
	if (phys_avail[i + 1] < phys_avail[i])
		panic("%s: phys_avail[%d]: start %#jx > end %#jx", __func__, i,
		    (intmax_t)phys_avail[i], (intmax_t)phys_avail[i + 1]);
}

/*
 * Split a chunk in phys_avail[] at the address 'pa'.
 *
 * 'pa' must be within a chunk (slots i and i + 1) or one of its boundaries.
 * Returns zero on actual split, in which case the two new chunks occupy slots
 * i to i + 3, else EJUSTRETURN if 'pa' was one of the boundaries (and no split
 * actually occurred) else ENOSPC if there are not enough slots in phys_avail[]
 * to represent the additional chunk caused by the split.
 */
static int
vm_phys_avail_split(vm_paddr_t pa, int i)
{
	int count;

	vm_phys_avail_check(i);
	if (pa < phys_avail[i] || pa > phys_avail[i + 1])
		panic("%s: address %#jx not in range at slot %d [%#jx;%#jx].",
		__func__, (uintmax_t)pa, i,
		(uintmax_t)phys_avail[i], (uintmax_t)phys_avail[i + 1]);

	if (pa == phys_avail[i] || pa == phys_avail[i + 1])
		return (EJUSTRETURN);
	count = vm_phys_avail_count();
	if (count >= PHYS_AVAIL_ENTRIES)
		return (ENOSPC);

	memmove(&phys_avail[i + 2], &phys_avail[i],
	    (count - i) * sizeof(phys_avail[0]));

	phys_avail[i + 1] = pa;
	phys_avail[i + 2] = pa;
	vm_phys_avail_check(i);
	vm_phys_avail_check(i + 2);

	return (0);
}

#ifdef NUMA
/*
 * Return the index of an overlapping phys_avail entry or -1.
 */
static int
vm_phys_avail_find(vm_paddr_t pa)
{
	for (int i = 0; phys_avail[i + 1]; i += 2)
		if (phys_avail[i] <= pa && phys_avail[i + 1] > pa)
			return (i);

	return (-1);
}
#endif /* NUMA */

/*
 * Physical page allocation.
 */

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

	VM_PGTREE_ALMOST_EMPTY	= 4096,
	VM_PGTREE_ALMOST_FULL	= VM_PGTREE_COUNT - VM_PGTREE_ALMOST_EMPTY,
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
		vm_pgidx_t	last_pgidx : 48;
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

/* 1 LLFree "instance" per domain. */
struct vm_phys_domain {
	vm_paddr_t start, end;
	union vm_pgtree *trees;
	union vm_pgcount *counts;
	struct vm_pgset *sets;
	uint64_t pages;
	domainid_t id;
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

DPCPU_DEFINE_STATIC(union vm_pcpu_tree, reserved_trees[MAXMEMDOM]);

struct vm_phys_alloc_args {
	uint16_t pages;
	vm_paddr_t low;
	vm_paddr_t high;
	vm_align_t alignment;
	vm_paddr_t boundary;
};

static inline bool
vm_pgidx_is_valid(vm_pgidx_t pg)
{
	return ((pg & ~pgidx_mask) == 0);
}

static inline uint8_t
vm_page_order(u_long pages)
{
	uint8_t order;
	order = flsl(pages) - 1;
	return powerof2(pages) ? order : order + 1;
}

/*
 * Allocates 2^O free physical pages.
 */

static void vm_phys_domain_init_seg(struct vm_phys_domain *domain,
    const struct vm_phys_seg *seg);
static vm_pgidx_t vm_phys_domain_alloc(struct vm_phys_domain *domain,
    uint8_t order);
static vm_pgidx_t vm_phys_domain_alloc_search(struct vm_phys_domain *domain,
    uint8_t order, uint16_t pages, union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, uint16_t *reserved_pages);
static vm_pgidx_t vm_phys_domain_alloc_contig(struct vm_phys_domain *domain,
    const struct vm_phys_alloc_args *args);
static vm_pgidx_t vm_phys_domain_alloc_search_all(
    const struct vm_phys_domain *domain, uint32_t neighbors_idx,
    uint32_t skip_idx, uint8_t order, uint16_t pages, uint16_t *reserved_pages);

static inline unsigned vm_pcpu_tree_get_tree(union vm_pcpu_tree local);
static inline union vm_pcpu_tree vm_pcpu_tree_load(
    const union vm_pcpu_tree *local);
static inline int vm_pcpu_tree_fcmpset(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, union vm_pcpu_tree new);
static inline union vm_pcpu_tree vm_pcpu_tree_swap(union vm_pcpu_tree *local,
    union vm_pcpu_tree new);
static vm_pgidx_t vm_pcpu_tree_alloc(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, uint8_t order, uint16_t pages);
static bool vm_pcpu_tree_acquire(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, uint16_t pages);
static bool vm_pcpu_tree_alloc_sync(union vm_pcpu_tree *local,
    union vm_pcpu_tree old_local, uint16_t wanted_pgs);
static void vm_pcpu_tree_release(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, uint16_t pages);
static void vm_pcpu_tree_update_pgidx(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old, vm_pgidx_t pg);
static void vm_pcpu_tree_reserve(union vm_pcpu_tree *local, vm_pgidx_t pg,
    uint16_t pages);
static bool vm_pcpu_tree_unreserve(union vm_pcpu_tree *local,
    union vm_pcpu_tree *old);
static vm_pgidx_t vm_pcpu_tree_steal(int cpu_id, int domain, uint8_t order,
    uint16_t pages, uint16_t *reserved_pages);

static inline union vm_pgtree vm_pgtree_load(const union vm_pgtree *tree);
static inline int vm_pgtree_fcmpset(union vm_pgtree *tree, union vm_pgtree *old,
    union vm_pgtree new);
static inline bool vm_pgtree_sync(union vm_pgtree *tree, uint16_t *pages);
static inline void vm_pgtree_add(union vm_pgtree *tree, uint16_t pages);
static inline union vm_pgtree vm_pgtree_fetchadd(union vm_pgtree *tree,
    uint16_t pages);
static inline uint32_t vm_pgtree_get_neighbors(uint32_t tree_idx);
static vm_pgidx_t vm_pgtree_alloc_search_neighbors(uint32_t neighbors_idx,
    uint32_t skip_idx, uint8_t order, uint16_t pages, uint16_t *reserved_pages);
static vm_pgidx_t vm_pgtree_alloc_search_all(unsigned all_idx, unsigned len,
     unsigned skip_idx, uint8_t order, uint16_t pages,
     uint16_t *reserved_pages);
static vm_pgidx_t vm_pgtree_alloc_search_neighbors_constrained(
    uint32_t neighbors_idx, uint32_t skip_idx, uint8_t order, uint16_t pages,
    uint16_t *reserved_pages, uint16_t min, uint16_t max);
static vm_pgidx_t vm_pgtree_alloc_search_all_constrained(unsigned all_idx,
    unsigned len, unsigned skip_idx, uint8_t order, uint16_t pages,
    uint16_t *reserved_pages, uint16_t min, uint16_t max);
static vm_pgidx_t vm_pgtree_alloc_search(unsigned tree_idx,
    uint8_t order, uint16_t pages, uint16_t *reserved_pages, uint16_t min,
    uint16_t max);
static inline vm_pgidx_t vm_pgtree_alloc_search_order(unsigned tree_idx,
    uint8_t order, uint16_t pages);
static vm_pgidx_t vm_pgtree_alloc_search_order_0to9(unsigned tree_idx,
    uint8_t order, uint16_t pages);
static vm_pgidx_t vm_pgtree_alloc_search_order_10to12(unsigned tree_idx,
    uint8_t order);
static uint16_t vm_pgtree_reserve(union vm_pgtree *tree, uint16_t pages,
    uint16_t min, uint16_t max);
static void vm_pgtree_unreserve(union vm_pgtree *tree, uint16_t pages);
static bool vm_pgtree_steal(union vm_pgtree *tree, uint16_t pages,
    uint16_t stolen_pages, uint16_t *reserved_pages);
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
static void vm_pgcount_free(union vm_pgcount counts[], uint8_t order);
static inline void vm_pgcount_free_order_0to8(union vm_pgcount *count,
    uint8_t order);
static inline void vm_pgcount_free_order_9(union vm_pgcount *count);
static void vm_pgcount_free_order_10to12(union vm_pgcount counts[],
    uint8_t order);
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

static void vm_pgset_init(struct vm_pgset *set);
static void vm_pgset_init_partial(struct vm_pgset *set, uint16_t pages,
    bool skip_front);
static inline unsigned vm_pgset_alloc_order_0(struct vm_pgset *set);
static inline int vm_pgset_alloc_order_1to5(struct vm_pgset *set,
    uint8_t order);
static inline int vm_pgset_alloc_order_6to8(struct vm_pgset *set,
    uint8_t order);
static void vm_pgset_free_order_0to8(struct vm_pgset *set, vm_pgidx_t pg,
    uint8_t order);
static inline void vm_pgset_free_order_0(struct vm_pgset *set, vm_pgidx_t pg);
static void vm_pgset_free_order_1to8(struct vm_pgset *set, vm_pgidx_t pg,
    uint8_t order);
static inline void vm_pgset_free(vm_pgidx_t pg);

// XXX: remove this
struct vm_page * _vm_phys_alloc_contig(int domain, u_long pages, vm_paddr_t low,
    vm_paddr_t high, vm_align_t alignment, vm_paddr_t boundary);
struct vm_page *
_vm_phys_alloc_contig(int domain, u_long pages, vm_paddr_t low, vm_paddr_t high,
    vm_align_t alignment, vm_paddr_t boundary)
{
	struct vm_phys_domain *dom;
	struct vm_phys_alloc_args args;
	vm_pgidx_t pg;

	KASSERT(0 < pages && pages <= (1 << 15),
	    ("%s: invalid page count %lu", __func__, pages));

	dom = &domains[domain];
	if (low >= high || high <= dom->start || dom->end <= low)
		return (NULL);

	args.pages = (uint16_t)pages;
	args.low = low;
	args.high = high;
	args.alignment = alignment;
	args.boundary = boundary;

	if ((pg = vm_phys_domain_alloc_contig(dom, &args)) == invalid_pgidx)
		return (NULL);

	panic("...");
}

void
vm_phys_free_pages(struct vm_page *m, int pool, int order)
{
	// FIXME: instead of segind/poolind/order, store pgidx?
	// find domain, within domain, find pgidx/setidx/treeidx
	struct vm_phys_domain *domain;
	vm_paddr_t offset;
	vm_pgidx_t pg;
	unsigned tree_idx, set_idx;

	KASSERT(order < VM_NFREEORDER,
	    ("%s: order %d is out of range", __func__, order));
	// XXX: alignment of page must match order!

	domain = &domains[phys_segments.array[m->segind].domain];
	offset = VM_PAGE_TO_PHYS(m) - domain->start;
	pg = offset / PAGE_SIZE;
	tree_idx = offset / VM_PGTREE_COUNT;
	set_idx = offset / VM_PGSET_COUNT;

	/*
	 * Clear the bits for the freed run of pages in the lowest-level page
	 * sets and/or adjust the free count(s) in the corresponding
	 * set-counters.
	 */
	if (order < 9)
		vm_pgset_free_order_1to8(&domain->sets[set_idx], pg, order);
	vm_pgcount_free(&domain->counts[set_idx], order);

	/*
	 * Add the freed pages back to the appropriate page trees.  Check, if
	 * the corresponding tree is reserved by the current CPU, in which case
	 * the added free pages go to the local representation, otherwise they
	 * go directly to the global tree count.
	 */
	critical_enter();
	union vm_pcpu_tree *local = DPCPU_PTR(reserved_trees[domain->id]);
	union vm_pcpu_tree old = vm_pcpu_tree_load(local);

	if (vm_pcpu_tree_get_tree(old) == tree_idx) {
		// cas the tree with an added count
		critical_exit();
	} else {
		critical_exit();
		vm_pgtree_add(&domain->trees[tree_idx], 1 << order);
	}
}

void
vm_phys_free_contig(struct vm_page *m, int pool, u_long pages)
{
	// not required to be order 2!
}

static void
vm_phys_domain_init(vm_offset_t *va)
{
	const struct mem_affinity *ma;
	vm_paddr_t pa0, start, end;
	vm_pgidx_t pg_start, pg_end, pg_total;
	int macount;
	struct vm_phys_domain *domain;
	uint64_t pgcount, treecount, setcount;
	size_t alloc_size;
	vm_paddr_t pa;

	pa0 = first_page * PAGE_SIZE;
	start = (vm_paddr_t)-1;
	end = 0;
	macount = 0;
	pg_total = 0;

	for (int i = 0; i < vm_ndomains; i++) {
		for (int j = 0; mem_affinity[j].end != 0; j++) {
			ma = &mem_affinity[j];
			if (ma->domain != i)
				continue;
			if (ma->end <= pa0)
				continue;
			
			if (ma->start < start)
				start = ma->start;
			if (ma->end > end)
				end = ma->end;
		}

		if (!macount)
			continue;

		pg_start = atop(start);
		pg_end = atop(end);
		if (pg_start < first_page)
			panic("%s: invalid page index %lu in domain %d",
			    __func__, pg_start, i);

		pgcount = pg_end - pg_start;
		if (pgcount == 0)
			continue;

		treecount = pgcount / VM_PGTREE_COUNT;
		setcount = pgcount / VM_PGSET_COUNT;

		domain = &domains[i];
		domain->start = start;
		domain->end = end;
		domain->pages = pgcount;
		domain->id = (domainid_t)i;

		/*
		 * Allocate and map consecutive memory for the domain's pgtrees,
		 * pgcounts and pgsets.
		 */
		alloc_size = round_page(sizeof(union vm_pgtree) * treecount);
		pa = vm_phys_early_alloc(i, alloc_size);
		domain->trees = (union vm_pgtree *)pmap_map(va, pa,
		    pa + alloc_size, VM_PROT_READ | VM_PROT_WRITE);
		bzero(domain->trees, alloc_size);
		KASSERT(is_aligned(domain->trees, _Alignof(union vm_pgtree)),
		    ("%s: invalid vm_pgtree alignment in domain %d",
		    __func__, i));

		alloc_size = round_page(sizeof(union vm_pgcount) * setcount);
		pa = vm_phys_early_alloc(i, alloc_size);
		domain->counts = (union vm_pgcount *)pmap_map(va, pa,
		    pa + alloc_size, VM_PROT_READ | VM_PROT_WRITE);
		bzero(domain->counts, alloc_size);
		KASSERT(
		    is_aligned(domain->trees, _Alignof(union vm_pgcount_4x)),
		    ("%s: invalid vm_pgcount alignment in domain %d",
		    __func__, i));

		alloc_size = round_page(sizeof(struct vm_pgset) * setcount);
		pa = vm_phys_early_alloc(i, alloc_size);
		domain->sets = (struct vm_pgset *)pmap_map(va, pa,
		    pa + alloc_size, VM_PROT_READ | VM_PROT_WRITE);
		/*
		 * We must initially mark every page as allocated, before later
		 * selectively marking the free ones based on what is available.
		 */
		memset(domain->sets, -1, alloc_size);
		KASSERT(is_aligned(domain->sets, _Alignof(struct vm_pgset)),
		    ("%s: invalid vm_pgset alignment in domain %d",
		    __func__, i));

		pg_total += pgcount;
	}

	if (pg_total != vm_page_array_size)
		panic("%s: mismatch between vm_page_array size and "
		    "vm_phys_domain page counts.", __func__);
}

static void
vm_phys_domain_init_segments(void)
{
	for (int i = 0; i < vm_ndomains; i++) {
		for (int j = 0; j < phys_segments.len; j++)
			vm_phys_domain_init_seg(&domains[i],
			    &phys_segments.array[j]);
	}
}

static void
vm_phys_domain_init_seg(struct vm_phys_domain *domain,
    const struct vm_phys_seg *seg)
{
	vm_pgidx_t pg;
	uint64_t pgcount, remaining_pgs;
	unsigned tree_idx, set_idx;
	uint16_t pgs;

	if (domain->id != seg->domain)
		return;

	KASSERT(seg->start >= domain->start,
	    ("%s: ... %d", __func__, domain->id));
	KASSERT(seg->end <= domain->end,
	    ("%s: .... %d", __func__, domain->id));

	pg = remaining_pgs = (seg->start - domain->start) / PAGE_SIZE;
	pgcount = (seg->end - seg->start) / PAGE_SIZE;

	/*
	 * Initialize all segment page tree structures.
	 */
	tree_idx = pg / VM_PGTREE_COUNT;
	if ((pgs = (pg & (VM_PGTREE_COUNT - 1)))) {
		pgs = MIN(pgs, pgcount);
		domain->trees[tree_idx].free += pgs;
		remaining_pgs -= pgs;
		tree_idx++;
	}

	for (unsigned i = tree_idx; remaining_pgs > 0; i++) {
		pgs = MAX(remaining_pgs, VM_PGTREE_COUNT);
		domain->trees[i].free += pgs;
		remaining_pgs -= pgs;
	}

	/*
	 * Initialize all segment page count and set structures.
	 */
	set_idx = pg / VM_PGSET_COUNT;
	remaining_pgs = pgcount;
	if ((pgs = (pg & (VM_PGSET_COUNT - 1)))) {
		pgs = MIN(pgs, pgcount);
		vm_pgset_init_partial(&domain->sets[set_idx], pgs, true);
		domain->counts[set_idx].free += pgs;
		remaining_pgs -= pgs;
		set_idx++;
	}

	for (unsigned i = set_idx; remaining_pgs > 0; i++) {
		if (remaining_pgs >= VM_PGSET_COUNT) {
			vm_pgset_init(&domain->sets[i]);
			domain->counts[i].free += VM_PGSET_COUNT;
			remaining_pgs -= VM_PGSET_COUNT;
		} else {
			vm_pgset_init_partial(&domain->sets[i], remaining_pgs,
			    false);
			domain->counts[i].free += remaining_pgs;
			remaining_pgs = 0;
		}
	}
}

static vm_pgidx_t
vm_phys_domain_alloc(struct vm_phys_domain *domain, uint8_t order)
{
	union vm_pcpu_tree *local, old;
	uint16_t pages, reserved_pages;
	vm_pgidx_t pg;

	pages = 1 << order;

	critical_enter();
	local = DPCPU_PTR(reserved_trees[domain->id]);

	/*
	 * Try to allocate the desired pages from the current CPU's reserved
	 * tree.  If allocating sufficient pages fails, unreserve the current
	 * tree before trying to find and reserve another one.
	 */
	pg = vm_pcpu_tree_alloc(local, &old, order, pages);
	if (pg != invalid_pgidx) {
		vm_pcpu_tree_update_pgidx(local, &old, pg);
		goto exit;
	}

	/*
	 * Try to find another tree with sufficient free pages, reserve it and
	 * allocate from that.  To reserve a new tree once found, at first only
	 * the reserved bit in the global tree is set and the free count is
	 * zeroed.  Only if the allocation ultimately succeedes, the local tree
	 * is also marked as reserved and its free count set, otherwise this is
	 * reverted.
	 */
	pg = vm_phys_domain_alloc_search(domain, order, pages, local, &old,
	    &reserved_pages);
	if (pg != invalid_pgidx)
		vm_pcpu_tree_reserve(local, pg, reserved_pages);

exit:
	critical_exit();
	return (pg);
}

static vm_pgidx_t
vm_phys_domain_alloc_search(struct vm_phys_domain *domain, uint8_t order,
    uint16_t pages, union vm_pcpu_tree *local, union vm_pcpu_tree *old,
    uint16_t *reserved_pages)
{
	unsigned tree_idx, neighbors_idx, all_idx, len;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);

	/*
	 * If there was a PCPU reserved tree but with insufficient free pages
	 * for the allocation request, we can search the cache line neighborhood
	 * of that tree instead of doing a full search over all trees in the
	 * domain right away.
	 */
	if (old->reserved) {
		tree_idx = vm_pcpu_tree_get_tree(*old);
		neighbors_idx = vm_pgtree_get_neighbors(tree_idx);

		/*
		 * Search the trees in the immediate neighborhood of the
		 * previous tree.  Neighborhood is defined as "on the same
		 * cache line".
		 */
		pg = vm_pgtree_alloc_search_neighbors(neighbors_idx, tree_idx,
		    order, pages, reserved_pages);
		if (pg != invalid_pgidx)
			return (pg);
	} else
		neighbors_idx = (unsigned)-1;

	/*
	 * Search for any free tree in the domain.
	 */
	all_idx = domain->start_tree;
	len = domain->end_tree - domain->start_tree;
	pg = vm_pgtree_alloc_search_all(all_idx, len, neighbors_idx, order,
	    pages, reserved_pages);
	if (pg != invalid_pgidx)
		return (pg);

	/*
	 * Unreserve other CPU reserved trees and try to steal from them.
	 */
	pg = vm_pcpu_tree_steal(curcpu, domain->id, order, pages,
	    reserved_pages);
	return (pg);
}

static vm_pgidx_t
vm_phys_domain_alloc_contig(struct vm_phys_domain *domain,
    const struct vm_phys_alloc_args *args)
{
	/*
	 * - restrict to max O=15?
	 * - check vm_phys_segs, find a suitable segment
	 * - restrict range of pgsets accordingly
	 * - search for a suitable range, i.e. a run of free bits
	 * - then reserve the associated pgtree and allocate from it
	 *
	 * - calculate next higher order
	 * - allocate N full trees (non-reserved, full count, maybe unreserve)
	 * - determine a range of valid trees min/max tree_idx, adjust for
	 *   alignment/boundary, determine 
	 *   
	 */
	unsigned i;
	uint8_t order;
	struct vm_phys_seg *seg;
	vm_paddr_t range[2];

	order = vm_page_order(args->pages);

	for (i = domain->end_seg - 1; i >= domain->start_seg; i--) {
		seg = &phys_segments.array[i];
		if (seg->start >= args->high)
			break;
		if (args->low >= seg->end)
			continue;

		range[0] = (args->low <= seg->start) ? seg->start : args->low;
		range[1] = (args->high <= seg->end) ? args->high : seg->end;

		if (range[1] - range[0] < ptoa(args->pages))
			continue;
		//order 0-12: regular
		//order 12-_: allocate consecutive blocks of full O15 trees
	}

	return (invalid_pgidx);
}

static inline unsigned
vm_pcpu_tree_get_tree(union vm_pcpu_tree local)
{
	KASSERT(local.reserved,
	    ("%s: PCPU tree is not valid (reserved)", __func__));
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

static inline union vm_pcpu_tree
vm_pcpu_tree_swap(union vm_pcpu_tree *local, union vm_pcpu_tree new)
{
	union vm_pcpu_tree old;

	old.bits = atomic_swap_64(&local->bits, new.bits);

	return (old);
}

static vm_pgidx_t
vm_pcpu_tree_alloc(union vm_pcpu_tree *local, union vm_pcpu_tree *old,
    uint8_t order, uint16_t pages)
{
	union vm_pgtree *tree;
	unsigned tree_idx;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);

	/*
	 * Try to allocate pages from the currently CPU reserved tree.
	 */
	*old = vm_pcpu_tree_load(local);
	if (vm_pcpu_tree_acquire(local, old, pages)) {
		// FIXME: try to do a last_pgidx allocation
		tree_idx = vm_pcpu_tree_get_tree(*old);
		pg = vm_pgtree_alloc_search_order(tree_idx, order, pages);
		if (pg != invalid_pgidx)
			return (pg);
		vm_pcpu_tree_release(local, old, pages);
	}

	/*
	 * If allocation from the PCPU tree failed (e.g., because of
	 * insufficient pages) but it was reserved, unreserve that tree before
	 * beginning to look for another.
	 */
	if (old->reserved) {
		if (vm_pcpu_tree_unreserve(local, old)) {
			tree = &pgtrees[vm_pcpu_tree_get_tree(*old)];
			vm_pgtree_unreserve(tree, old->free);
		}
	}

	return (invalid_pgidx);
}

static bool
vm_pcpu_tree_acquire(union vm_pcpu_tree *local, union vm_pcpu_tree *old,
    uint16_t pages)
{
	union vm_pcpu_tree new;

	CRITICAL_ASSERT(curthread);

	do {
		/*
		 * The local reservation may have been broken up by another
		 * thread.
		 */
		if (!old->reserved)
			return (false);

		/*
		 * If there are fewer free pages than the requested amount,
		 * try to sync with the global free count for the page tree.
		 */
		if (old->free < pages)
			return (vm_pcpu_tree_alloc_sync(local, *old,
			    pages - old->free));

		new = *old;
		new.free -= pages;
	} while (vm_pcpu_tree_fcmpset(local, old, new));

	return (true);
}

/*
 * When a CPU reserved tree holds insufficient pages for serving an allocation,
 * check its global counterpart, which may contain pages that were freed after
 * the tree had been reserved.  In that case, move these pages into the local
 * reserved tree and try to serve the allocation from the combined free page
 * count.
 */
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
vm_pcpu_tree_release(union vm_pcpu_tree *local, union vm_pcpu_tree *old,
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
		if (!old->reserved)
			break;
		new = *old;
		new.free += pages;

		if (!vm_pcpu_tree_fcmpset(local, old, new))
			return;
	}

	/*
	 * If the local tree has been remotely unreserved, add the release the
	 * pages back to the global tree instead.
	 */
	tree = &pgtrees[vm_pcpu_tree_get_tree(*old)];
	vm_pgtree_add(tree, pages);
}

static void
vm_pcpu_tree_update_pgidx(union vm_pcpu_tree *local, union vm_pcpu_tree *old,
    vm_pgidx_t pg)
{
	union vm_pcpu_tree new;

	KASSERT(vm_pgidx_is_valid(pg), ("%s: invalid pg %lu", __func__, pg));
	KASSERT(old->reserved, ("%s: previous PCPU not reserved", __func__));

	do {
		if (!old->reserved)
			return;
		new = *old;
		new.last_pgidx = pg;
	} while (vm_pcpu_tree_fcmpset(local, old, new));
}

static void
vm_pcpu_tree_reserve(union vm_pcpu_tree *local, vm_pgidx_t pg, uint16_t pages)
{
	union vm_pcpu_tree old, new;

	KASSERT(vm_pgidx_is_valid(pg), ("%s: invalid page %lu", __func__, pg));

	new.bits = 0;
	new.last_pgidx = pg;
	new.free = pages;
	new.reserved = true;

	old = vm_pcpu_tree_swap(local, new);
	KASSERT(!old.reserved, ("%s: PCPU tree already reserved", __func__));
}

static bool
vm_pcpu_tree_unreserve(union vm_pcpu_tree *local, union vm_pcpu_tree *old)
{
	union vm_pcpu_tree new;

	CRITICAL_ASSERT(curthread);

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

	return (true);
}

static vm_pgidx_t
vm_pcpu_tree_steal(int cpu_id, int domain, uint8_t order, uint16_t pages,
    uint16_t *reserved_pages)
{
	int cpu;
	union vm_pcpu_tree *remote, old;
	union vm_pgtree *tree;
	unsigned tree_idx, stolen_pages;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);

	CPU_FOREACH(cpu) {
		if (cpu == cpu_id)
			continue;

		/*
		 * Force the remote PCPU local tree to be unreserved.  This may
		 * fail due to a race with any local operation of that CPU or
		 * because of a concurrent steal attempt.
		 */
		remote = DPCPU_ID_PTR(cpu, reserved_trees[domain]);
		old = vm_pcpu_tree_load(remote);
		if (!vm_pcpu_tree_unreserve(remote, &old))
			continue;

		/*
		 * After the PCPU tree has been reserved, its acquired pages
		 * need to be released back to the global tree.  In the same
		 * step we attempt to execute our own allocation request.
		 * If the total (synced) free count of the tree is insufficient
		 * to serve the allocation request, the tree is unreserved and
		 * all previously reserved pages are released to it.  Otherwise,
		 * the tree remains reserved (now for the current CPU) and its
		 * entire free count is acquired and stored in `reserved_pages`.
		 */
		tree_idx = vm_pcpu_tree_get_tree(old);
		tree = &pgtrees[tree_idx];
		stolen_pages = old.free;

		if (!vm_pgtree_steal(tree, pages, stolen_pages, reserved_pages))
			continue;

		/*
		 * Finally, the allocation request has to be served by the
		 * corresponding subcounts and page sets.  This step may yet
		 * fail, in which case the tree must be unreserved and all
		 * acquired pages be released back to it.
		 */
		pg = vm_pgtree_alloc_search_order(tree_idx, order, pages);
		if (pg == invalid_pgidx) {
			vm_pgtree_unreserve(tree, pages + *reserved_pages);
			continue;
		}

		return (pg);
	}

	return (invalid_pgidx);
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

static inline void
vm_pgtree_add(union vm_pgtree *tree, uint16_t pages)
{
	atomic_add_16(&tree->bits, pages);
}

static inline union vm_pgtree
vm_pgtree_fetchadd(union vm_pgtree *tree, uint16_t pages)
{
	union vm_pgtree val;

	val.bits = atomic_fetchadd_16(&tree->bits, pages);

	return (val);
}

static inline uint32_t
vm_pgtree_get_neighbors(uint32_t tree_idx)
{
	return (tree_idx & ((CACHE_LINE_SIZE / sizeof(union vm_pgtree)) - 1));
}

static vm_pgidx_t
vm_pgtree_alloc_search_neighbors(uint32_t neighbors_idx, uint32_t skip_idx,
    uint8_t order, uint16_t pages, uint16_t *reserved_pages)
{
	uint16_t min, max;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);

	min = VM_PGTREE_ALMOST_EMPTY;
	max = VM_PGTREE_ALMOST_FULL;

	pg = vm_pgtree_alloc_search_neighbors_constrained(neighbors_idx,
	    skip_idx, order, pages, reserved_pages,  min, max);
	if (pg != invalid_pgidx)
		return (pg);

	pg = vm_pgtree_alloc_search_neighbors_constrained(neighbors_idx,
	    skip_idx, order, pages, reserved_pages, min, 0);
	if (pg != invalid_pgidx)
		return (pg);

	pg = vm_pgtree_alloc_search_neighbors_constrained(neighbors_idx,
	    skip_idx, order, pages, reserved_pages, min, 0);
	if (pg != invalid_pgidx)
		return (pg);

	return (invalid_pgidx);
}

static vm_pgidx_t
vm_pgtree_alloc_search_all(unsigned all_idx, unsigned len, unsigned skip_idx,
    uint8_t order, uint16_t pages, uint16_t *reserved_pages)
{
	uint16_t min, max;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);

	min = VM_PGTREE_ALMOST_EMPTY;
	max = VM_PGTREE_ALMOST_FULL;

	pg = vm_pgtree_alloc_search_all_constrained(all_idx, len, skip_idx,
	    order, pages, reserved_pages, min, max);
	if (pg != invalid_pgidx)
		return (pg);

	pg = vm_pgtree_alloc_search_all_constrained(all_idx, len, skip_idx,
	    order, pages, reserved_pages, min, 0);
	if (pg != invalid_pgidx)
		return (pg);

	pg = vm_pgtree_alloc_search_all_constrained(all_idx, len, skip_idx,
	    order, pages, reserved_pages, 0, 0);
	if (pg != invalid_pgidx)
		return (pg);

	return (invalid_pgidx);
}

static vm_pgidx_t
vm_pgtree_alloc_search_neighbors_constrained(unsigned neighbors_idx,
    unsigned skip_idx, uint8_t order, uint16_t pages, uint16_t *reserved_pages,
    uint16_t min, uint16_t max)
{
	vm_pgidx_t pg;
	unsigned last;

	/*
	 * Check all trees in the direct neighborhood of the previously
	 * reserved tree.
	 */
	last = neighbors_idx + VM_PGTREE_NEIGHBORS;
	for (unsigned tree_idx = neighbors_idx; tree_idx < last; tree_idx++) {
		if (tree_idx == skip_idx)
			continue;

		pg = vm_pgtree_alloc_search(tree_idx, order, pages,
		    reserved_pages, min, max);
		if (pg != invalid_pgidx)
			return (pg);
	}

	return (invalid_pgidx);
}

static vm_pgidx_t
vm_pgtree_alloc_search_all_constrained(unsigned all_idx, unsigned len,
    unsigned skip_idx, uint8_t order, uint16_t pages, uint16_t *reserved_pages,
    uint16_t min, uint16_t max)
{
	vm_pgidx_t pg;
	unsigned end, skip_end;

	end = all_idx + len;
	skip_end = skip_idx + VM_PGTREE_NEIGHBORS;

	for (unsigned tree_idx = all_idx; tree_idx < end; tree_idx++) {
		if (tree_idx >= skip_idx && tree_idx < skip_end)
			continue;

		pg = vm_pgtree_alloc_search(tree_idx, order, pages,
		    reserved_pages, min, max);
		if (pg != invalid_pgidx)
			return (pg);
	}

	return (invalid_pgidx);
}

static vm_pgidx_t
vm_pgtree_alloc_search(unsigned tree_idx, uint8_t order, uint16_t pages,
    uint16_t *reserved_pages, uint16_t min, uint16_t max)
{
	union vm_pgtree *tree;
	vm_pgidx_t pg;

	CRITICAL_ASSERT(curthread);
	KASSERT((1 << order) == pages,
	    ("%s: order %u and pages mismatch: %u", __func__, order, pages));

	/*
	 * Reserve the tree and zero its free count, if it matches the
	 * required constraints.
	 */
	tree = &pgtrees[tree_idx];
	*reserved_pages = vm_pgtree_reserve(tree, pages, min, max);
	if (*reserved_pages == 0)
		return (invalid_pgidx);

	/*
	 * Try to find a consecutive run of pages in the reserved tree.
	 * Upon failure to find a suitable run, free the un-reserve the
	 * the tree again.
	 */
	pg = vm_pgtree_alloc_search_order(tree_idx, order, pages);
	if (pg == invalid_pgidx) {
		vm_pgtree_unreserve(tree, pages + *reserved_pages);
		return (invalid_pgidx);
	}

	return (pg);
}

static inline vm_pgidx_t
vm_pgtree_alloc_search_order(unsigned tree_idx, uint8_t order, uint16_t pages)
{
	vm_pgidx_t pg;

	pg = (order <= 9)
	    ? vm_pgtree_alloc_search_order_0to9(tree_idx, order, pages)
	    : vm_pgtree_alloc_search_order_10to12(tree_idx, order);

	return (pg);
}

static vm_pgidx_t
vm_pgtree_alloc_search_order_0to9(unsigned tree_idx, uint8_t order,
    uint16_t pages)
{
	unsigned treeset_idx, pgset_idx;
	int offset;
	struct vm_pgset *set;

	KASSERT(order <= 9, ("%s: invalid order %u", __func__, order));
	KASSERT((1 << order) == pages,
	    ("%s: order %u and pages mismatch: %u", __func__, order, pages));

	/*
	 * Find a page count to allocate the required run of pages from.
	 */
	treeset_idx = tree_idx * VM_PGTREE_COUNT;
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

static vm_pgidx_t
vm_pgtree_alloc_search_order_10to12(unsigned tree_idx, uint8_t order)
{
	unsigned treeset_idx;
	int offset;

	KASSERT(9 < order && order < VM_NFREEORDER_MAX,
	    ("%s: invalid order %u", __func__, order));

	treeset_idx = tree_idx * VM_PGTREE_COUNT;
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
vm_pgtree_acquire_order_13_or_more(union vm_pgtree *tree, uint8_t order,
    uint16_t pages)
{
	return (false);
}

static uint16_t
vm_pgtree_reserve(union vm_pgtree *tree, uint16_t pages, uint16_t min,
    uint16_t max)
{
	union vm_pgtree old, new;

	old = vm_pgtree_load(tree);
	do {
		if (old.reserved)
			return (0);
		if (old.free < pages)
			return (0);
		if (max && old.free > max)
			return (0);
		if (old.free < min)
			return (0);

		new = old;
		new.free = 0;
		new.reserved = true;
	} while (vm_pgtree_fcmpset(tree, &old, new));

	return (old.free);
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

/*
 * Steals the reservation from the tree and attempts to reserve a number of
 * pages for allocation.
 *
 * The caller must previously have unreserved the corresponding PCPU
 * local-reserved tree and acquired the free count stored within and pass
 * this to this function.
 *
 * If the allocation is successful, the tree remains in a reserved state and the
 * number of pages stolen from the (global) tree (between 0 and 2^15 - 1) minus
 * the pages requested for the allocation is returned.
 *
 * If no allocation was possible, the tree is unreserved, all (locally)
 * stolen pages are released to the tree and -1 is returned.
 */
static bool
vm_pgtree_steal(union vm_pgtree *tree, uint16_t pages, uint16_t stolen_pages,
    uint16_t *reserved_pages)
{
	union vm_pgtree old, new;

	old = vm_pgtree_load(tree);
	do {
		KASSERT(old.reserved,
		    ("%s: unexpected tree %p not reserved", __func__, tree));
		new = old;
		if (old.free + stolen_pages >= pages)
			old.free = 0;
		else {
			old.free += stolen_pages;
			old.reserved = false;
		}
	} while (vm_pgtree_fcmpset(tree, &old, new));

	if (old.reserved) {
		*reserved_pages = old.free + stolen_pages - pages;
		return (true);
	} else
		return (false);
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

static void
vm_pgcount_free(union vm_pgcount counts[], uint8_t order)
{
	if (order < 9)
		vm_pgcount_free_order_0to8(&counts[0], order);
	else if (order == 9)
		vm_pgcount_free_order_9(&counts[0]);
	else
		vm_pgcount_free_order_10to12(counts, order);
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
vm_pgcount_free_order_10to12(union vm_pgcount counts[], uint8_t order)
{
	for (unsigned i = 0; i < (1 << order) / VM_PGSET_COUNT; i++)
		// KASSERT: swap, previous == 0?
		atomic_store_16(&counts[i].bits, VM_PGSET_COUNT);
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

static void
vm_pgset_init(struct vm_pgset *set)
{
	for (unsigned i = 0; i < VM_PGSET_SIZE; i++)
		set->free[i] = (vm_pgset_t)0;
}

static void
vm_pgset_init_partial(struct vm_pgset *set, uint16_t pages, bool skip_front)
{
	uint16_t pg;
	uint8_t word, bit;

	KASSERT(pages < VM_PGSET_COUNT, ("%s: ...", __func__));

	pg = (skip_front) ? VM_PGSET_COUNT - pages : 0;
	while (pages > 0) {
		word = pg / VM_PGSET_SIZE;
		bit = pg & (VM_PGSET_SIZE - 1);
		set->free[word] &= ~(1 << bit); 
		pages--;
	}
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

static void
vm_pgset_free_order_0to8(struct vm_pgset sets[], vm_pgidx_t pg, uint8_t order)
{
	if (order == 0)
		vm_pgset_free_order_0(&sets[0], pg);
	else
		vm_pgset_free_order_1to8(&sets[0], pg, order);
}

static inline void
vm_pgset_free_order_0(struct vm_pgset *set, vm_pgidx_t pg)
{
	unsigned word, bit;

	word = pg / VM_PGSET_WORD;
	bit = pg & (VM_PGSET_WORD - 1);
	atomic_testandclear_64(&set->free[word], bit);
}

static void
vm_pgset_free_order_1to8(struct vm_pgset *set, vm_pgidx_t pg, uint8_t order)
{
	uint16_t pages;
	unsigned word, bit;
	vm_pgset_t mask;

	pages = 1 << order;
	word = pg / VM_PGSET_WORD;

	if (order < 6) {
		bit = pg & (VM_PGSET_WORD - 1);

		KASSERT(bit + pages <= VM_PGSET_WORD,
		    ("%s: bad pg %lu alignment for order %u",
		    __func__, pg, order));

		mask = ((1 << pages) - 1) << bit;
		atomic_clear_64(&set->free[word], mask);
	} else {
		KASSERT(word + (pages / VM_PGSET_SIZE),
		    ("%s: bad pg %lu alignment for order %u",
		    __func__, pg, order));

		for (unsigned w = word; w < word + (pages / VM_PGSET_SIZE); w++)
			atomic_store_64(&set->free[w], 0);
	}
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

static int sysctl_vm_phys_free(SYSCTL_HANDLER_ARGS);

static int
sysctl_vm_phys_free(SYSCTL_HANDLER_ARGS)
{
	struct sbuf sbuf;
	int error;
	struct vm_phys_domain *dom;
	uint64_t pages;

	if ((error = sysctl_wire_old_buffer(req, 0)) != 0)
		return (error);
	sbuf_new_for_sysctl(&sbuf, NULL, 128 * vm_ndomains, req);
	for (int domain = 0; domain < vm_ndomains; domain++) {
		dom = &domains[domain];
	}
}