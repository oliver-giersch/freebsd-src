#ifndef _VM_PAGESCAN_
#define _VM_PAGESCAN_

#include <vm/vm_pagequeue.h>

struct vm_pagescan {
	struct vm_batchqueue	bq;
	struct vm_pagequeue	*pq;
	struct vm_pglinks	*marker;
	unsigned		count;
	unsigned		max_scan;
};

static inline void
vm_pagescan_init(struct vm_pagescan *ps, struct vm_pagequeue *pq,
    struct vm_page *after, unsigned max_scan)
{
	return;
}

static inline void
vm_pagescan_deinit(struct vm_pagescan *ps)
{
	return;
}

static inline void
vm_pagescan_collect_batch(struct vm_pagescan *ps, bool dequeue)
{
	struct vm_pagequeue *pq;

	pq = ps->pq;

	// at boot time, we must reserve N additional entries in the vm_pglinks
	// array, which are allocated in a bump-allocator style fashion by each
	// pagedaemon/domain.
	// N is calculated as an (upper) bound once at boot time.
	// it is calculated as (mp_ncpus * vm_ndomains) + (2 * vm_ndomains)
	vm_pagequeue_assert_locked(pq);
	KASSERT(())
}

#endif /* _VM_PAGESCAN_ */