/*-
 * SPDX-License-Identifier: (BSD-3-Clause AND MIT-CMU)
 *
 * Copyright (c) 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * This code is derived from software contributed to Berkeley by
 * The Mach Operating System project at Carnegie-Mellon University.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * Copyright (c) 1987, 1990 Carnegie-Mellon University.
 * All rights reserved.
 *
 * Authors: Avadis Tevanian, Jr., Michael Wayne Young
 *
 * Permission to use, copy, modify and distribute this software and
 * its documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 *
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND
 * FOR ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 *
 * Carnegie Mellon requests users of this software to return to
 *
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 *
 * any improvements or extensions that they make and grant Carnegie the
 * rights to redistribute these changes.
 */

/*
 * Resident memory system definitions.
 */

#ifndef _VM_PAGE_
#define _VM_PAGE_

#include <sys/domainset.h>

#include <vm/pmap.h>
#include <vm/_vm_phys.h>

/*
 *	Management of resident (logical) pages.
 *
 *	A small structure is kept for each resident
 *	page, indexed by page number.  Each structure
 *	is an element of several collections:
 *
 *		A radix tree used to quickly
 *		perform object/offset lookups
 *
 *		A list of all pages for a given object,
 *		so they can be quickly deactivated at
 *		time of deallocation.
 *
 *		An ordered list of pages due for pageout.
 *
 *	In addition, the structure contains the object
 *	and offset to which this page belongs (for pageout),
 *	and sundry status bits.
 *
 *	In general, operations on this structure's mutable fields are
 *	synchronized using either one of or a combination of locks.  If a
 *	field is annotated with two of these locks then holding either is
 *	sufficient for read access but both are required for write access.
 *	The queue lock for a page depends on the value of its queue field and is
 *	described in detail below.
 *
 *	The following annotations are possible:
 *	(A) the field must be accessed using atomic(9) and may require
 *	    additional synchronization.
 *	(B) the page busy lock.
 *	(C) the field is immutable.
 *	(F) the per-domain lock for the free queues.
 *	(M) Machine dependent, defined by pmap layer.
 *	(O) the object that the page belongs to.
 *	(Q) the page's queue lock.
 *
 *	The busy lock is an embedded reader-writer lock that protects the
 *	page's contents and identity (i.e., its <object, pindex> tuple) as
 *	well as certain valid/dirty modifications.  To avoid bloating the
 *	the page structure, the busy lock lacks some of the features available
 *	the kernel's general-purpose synchronization primitives.  As a result,
 *	busy lock ordering rules are not verified, lock recursion is not
 *	detected, and an attempt to xbusy a busy page or sbusy an xbusy page
 *	results will trigger a panic rather than causing the thread to block.
 *	vm_page_sleep_if_busy() can be used to sleep until the page's busy
 *	state changes, after which the caller must re-lookup the page and
 *	re-evaluate its state.  vm_page_busy_acquire() will block until
 *	the lock is acquired.
 *
 *	The valid field is protected by the page busy lock (B) and object
 *	lock (O).  Transitions from invalid to valid are generally done
 *	via I/O or zero filling and do not require the object lock.
 *	These must be protected with the busy lock to prevent page-in or
 *	creation races.  Page invalidation generally happens as a result
 *	of truncate or msync.  When invalidated, pages must not be present
 *	in pmap and must hold the object lock to prevent concurrent
 *	speculative read-only mappings that do not require busy.  I/O
 *	routines may check for validity without a lock if they are prepared
 *	to handle invalidation races with higher level locks (vnode) or are
 *	unconcerned with races so long as they hold a reference to prevent
 *	recycling.  When a valid bit is set while holding a shared busy
 *	lock (A) atomic operations are used to protect against concurrent
 *	modification.
 *
 *	In contrast, the synchronization of accesses to the page's
 *	dirty field is a mix of machine dependent (M) and busy (B).  In
 *	the machine-independent layer, the page busy must be held to
 *	operate on the field.  However, the pmap layer is permitted to
 *	set all bits within the field without holding that lock.  If the
 *	underlying architecture does not support atomic read-modify-write
 *	operations on the field's type, then the machine-independent
 *	layer uses a 32-bit atomic on the aligned 32-bit word that
 *	contains the dirty field.  In the machine-independent layer,
 *	the implementation of read-modify-write operations on the
 *	field is encapsulated in vm_page_clear_dirty_mask().  An
 *	exclusive busy lock combined with pmap_remove_{write/all}() is the
 *	only way to ensure a page can not become dirty.  I/O generally
 *	removes the page from pmap to ensure exclusive access and atomic
 *	writes.
 *
 *	The ref_count field tracks references to the page.  References that
 *	prevent the page from being reclaimable are called wirings and are
 *	counted in the low bits of ref_count.  The containing object's
 *	reference, if one exists, is counted using the VPRC_OBJREF bit in the
 *	ref_count field.  Additionally, the VPRC_BLOCKED bit is used to
 *	atomically check for wirings and prevent new wirings via
 *	pmap_extract_and_hold().  When a page belongs to an object, it may be
 *	wired only when the object is locked, or the page is busy, or by
 *	pmap_extract_and_hold().  As a result, if the object is locked and the
 *	page is not busy (or is exclusively busied by the current thread), and
 *	the page is unmapped, its wire count will not increase.  The ref_count
 *	field is updated using atomic operations in most cases, except when it
 *	is known that no other references to the page exist, such as in the page
 *	allocator.  A page may be present in the page queues, or even actively
 *	scanned by the page daemon, without an explicitly counted referenced.
 *	The page daemon must therefore handle the possibility of a concurrent
 *	free of the page.
 *
 *	The queue state of a page consists of the queue and act_count fields of
 *	its atomically updated state, and the subset of atomic flags specified
 *	by PGA_QUEUE_STATE_MASK.  The queue field contains the page's page queue
 *	index, or PQ_NONE if it does not belong to a page queue.  To modify the
 *	queue field, the page queue lock corresponding to the old value must be
 *	held, unless that value is PQ_NONE, in which case the queue index must
 *	be updated using an atomic RMW operation.  There is one exception to
 *	this rule: the page daemon may transition the queue field from
 *	PQ_INACTIVE to PQ_NONE immediately prior to freeing the page during an
 *	inactive queue scan.  At that point the page is already dequeued and no
 *	other references to that vm_page structure can exist.  The PGA_ENQUEUED
 *	flag, when set, indicates that the page structure is physically inserted
 *	into the queue corresponding to the page's queue index, and may only be
 *	set or cleared with the corresponding page queue lock held.
 *
 *	To avoid contention on page queue locks, page queue operations (enqueue,
 *	dequeue, requeue) are batched using fixed-size per-CPU queues.  A
 *	deferred operation is requested by setting one of the flags in
 *	PGA_QUEUE_OP_MASK and inserting an entry into a batch queue.  When a
 *	queue is full, an attempt to insert a new entry will lock the page
 *	queues and trigger processing of the pending entries.  The
 *	type-stability of vm_page structures is crucial to this scheme since the
 *	processing of entries in a given batch queue may be deferred
 *	indefinitely.  In particular, a page may be freed with pending batch
 *	queue entries.  The page queue operation flags must be set using atomic
 *	RWM operations.
 */

#if PAGE_SIZE == 4096
typedef uint8_t vm_page_bits_t;
#elif PAGE_SIZE == 8192
typedef uint16_t vm_page_bits_t;
#elif PAGE_SIZE == 16384
typedef uint32_t vm_page_bits_t;
#elif PAGE_SIZE == 32768
typedef uint64_t vm_page_bits_t;
#endif

#define VM_PAGE_BITS_ALL ((vm_page_bits_t)~(vm_page_bits_t)0)

/*
 * Special bits used in the ref_count field.
 *
 * ref_count is normally used to count wirings that prevent the page from being
 * reclaimed, but also supports several special types of references that do not
 * prevent reclamation.  Accesses to the ref_count field must be atomic unless
 * the page is unallocated.
 *
 * VPRC_OBJREF is the reference held by the containing object.  It can set or
 * cleared only when the corresponding object's write lock is held.
 *
 * VPRC_BLOCKED is used to atomically block wirings via pmap lookups while
 * attempting to tear down all mappings of a given page.  The page busy lock and
 * object write lock must both be held in order to set or clear this bit.
 */
enum {
	/* The mappings are being removed. */
	VPRC_BLOCKED		= 0x40000000,
	/* The VM object reference, cleared with (O). */
	VPRC_OBJREF		= 0x80000000,
	VPRC_WIRE_COUNT_MAX	= ~(VPRC_BLOCKED | VPRC_OBJREF),
	
};

static inline unsigned
vprc_wire_count(unsigned ref_count)
{
	return (ref_count & ~(VPRC_BLOCKED | VPRC_OBJREF));
}

/* XXX(PAVE): depracate/remove */
#define VPRC_WIRE_COUNT(c) vprc_wire_count((c))

/*
 * Busy page implementation details.
 * The algorithm is taken mostly by rwlock(9) and sx(9) locks implementation,
 * even if the support for owner identity is removed because of size
 * constraints.  Checks on lock recursion are then not possible, while the
 * lock assertions effectiveness is someway reduced.
 */
enum {
	VPB_BIT_SHARED		= 0x01,
	VPB_BIT_EXCLUSIVE	= 0x02,
	VPB_BIT_WAITERS		= 0x04,
	VPB_BIT_FLAGMASK	=
	    (VPB_BIT_SHARED | VPB_BIT_EXCLUSIVE | VPB_BIT_WAITERS)
};

enum {
	VPB_SHARERS_SHIFT	= 3,
	VPB_ONE_SHARER		= (1 << VPB_SHARERS_SHIFT),
};

static inline unsigned
vpb_sharers(unsigned busy_lock)
{
	return ((busy_lock & ~VPB_BIT_FLAGMASK) >> VPB_SHARERS_SHIFT);
}

static inline unsigned
vpb_sharers_word(unsigned busy_lock)
{
	return (busy_lock << VPB_SHARERS_SHIFT | VPB_BIT_SHARED);
}

#define VPB_SHARERS(x) vpb_sharers((x))
#define VPB_SHARERS_WORD(x) vpb_sharers_word((x))
#define VPB_SINGLE_EXCLUSIVE VPB_BIT_EXCLUSIVE

#ifdef INVARIANTS
#define VPB_CURTHREAD_EXCLUSIVE\
    (VPB_BIT_EXCLUSIVE | ((u_int)(uintptr_t)curthread & ~VPB_BIT_FLAGMASK))
#else
#define VPB_CURTHREAD_EXCLUSIVE VPB_SINGLE_EXCLUSIVE
#endif /* INVARIANTS */

#define VPB_UNBUSIED VPB_SHARERS_WORD(0)

/* Freed lock blocks both shared and exclusive. */
#define VPB_FREED (0xffffffff - VPB_BIT_SHARED)

/*
 * The VM page's atomic flags are updated using atomic operations.  To set or
 * clear these flags, the functions vm_page_state_set() and
 * vm_page_state_clear() must be used.  Neither these flags nor these functions
 * are part of the KBI.
 *
 * PGA_REFERENCED may be cleared only if the page is locked.  It is set by
 * both the MI and MD VM layers.  However, kernel loadable modules should not
 * directly set this flag.  They should call vm_page_reference() instead.
 *
 * PGA_WRITEABLE is set exclusively on managed pages by pmap_enter().
 * When it does so, the object must be locked, or the page must be
 * exclusive busied.  The MI VM layer must never access this flag
 * directly.  Instead, it should call pmap_page_is_write_mapped().
 *
 * PGA_EXECUTABLE may be set by pmap routines, and indicates that a page has
 * at least one executable mapping.  It is not consumed by the MI VM layer.
 *
 * PGA_NOSYNC must be set and cleared with the page busy lock held.
 *
 * PGA_SWAP_FREE is used to defer freeing swap space to the pageout daemon
 * when the context that dirties the page does not have the object write lock
 * held.
 */
 enum {
	/* The page may be mapped writeable. */
	PGA_WRITEABLE	= 0x0001,
	/* The page has been referenced. */
	PGA_REFERENCED	= 0x0002,
	/* The page may be mapped executable. */
	PGA_EXECUTABLE	= 0x0004,
	/* Do not collect for syncer. */
	PGA_NOSYNC	= 0x0080,
	/* The page with swap space was dirtied. */
	PGA_SWAP_FREE	= 0x0100,
	/* The page has allocated swap space. */
	PGA_SWAP_SPACE	= 0x0200,
};

/*
 * Page flags.  Updates to these flags are not synchronized, and thus they must
 * be set during page allocation or free to avoid races.
 *
 * The PG_PCPU_CACHE flag is set at allocation time if the page was
 * allocated from a per-CPU cache.  It is cleared the next time that the
 * page is allocated from the physical memory allocator.
 */
enum {
	/* Page was allocated form per-CPU caches. */
	PG_PCPU_CACHE	= 0x01,
	/* Physical page doesn't exist. */
	PG_FICTITIOUS	= 0x02,
	/* Page is zeroed. */
	PG_ZERO		= 0x04,
	/* DEPRECATED: Page is a special marker page. */
	__PG_MARKER	= 0x08,
	/* Don't include page in a dump. */
	PG_NODUMP	= 0x10,
	/* The page should never be freed. */
	PG_NOFREE	= 0x20,
};

/*
 * Page flags stored in oflags:
 *
 * Access to these page flags is synchronized by the lock on the object
 * containing the page (O).
 *
 * Note: VPO_UNMANAGED (used by OBJT_DEVICE, OBJT_PHYS and OBJT_SG)
 * 	 indicates that the page is not under PV management but
 * 	 otherwise should be treated as a normal page.  Pages not
 * 	 under PV management cannot be paged out via the
 * 	 object/vm_page_t because there is no knowledge of their pte
 * 	 mappings, and such pages are also not on any PQ queue.
 */
enum {
	/* kmem mapping allows execution. */
	VPO_KMEM_EXEC	= 0x01,
	/* Waiting for swap to finish. */
	VPO_SWAPSLEEP	= 0x02,
	/* No PV management for page. */
	VPO_UNMANAGED	= 0x04,
	/* Swap I/O in progress on page. */
	VPO_SWAPINPROG	= 0x08,
};

struct vm_page {
	/* The sorted (by pindex) list of pages in same object (O). */
	TAILQ_ENTRY(vm_page)	listq;
	/* The VM object the page belongs to (O). */
	struct vm_object	*object;
	/* The logical offset into the VM object (O,P). */
	vm_pindex_t		pindex;
	/* The physical page address (C). */
	vm_paddr_t		phys_addr;
	/* The page's index into the VM page arrays (C). */
	unsigned		pgind;
	/* The page reference count (A). */
	unsigned		ref_count;
	/* The busy owners lock (A). */
	unsigned		busy_lock;
	/* The atomically accessed page state. (A) */
	uint8_t			state;
	/* page PG_* flags (P) */
	uint8_t			flags;
	/* page VPO_* flags (O) */
	uint8_t			oflags;
	/* The index of the buddy queue (F). */
	uint8_t			order;
	/* The physical freepool index (F). */
	uint8_t			pool;
	/* The pagesizes[] index (O). */
	uint8_t			psind;
	/* The physical segment index (C). */
	uint8_t			segind;
	/*
	 * NOTE that these must support one bit per DEV_BSIZE in a page
	 * so, on normal X86 kernels, they must be at least 8 bits wide
	 * 
	 * The valid DEV_BSIZE chunk map (O,B).
	 */
	vm_page_bits_t		valid;
	/* The dirty DEV_BSIZE chunk map (M,B). */
	vm_page_bits_t		dirty;
#ifdef NUMA
	/* The physical NUMA domain index. */
	uint8_t		domain;
	uint8_t		reserved[2];
#else
	uint8_t		reserved[3];
#endif /* NUMA */
} __aligned(CACHE_LINE_SIZE);

/*
 * PGA_ENQUEUED is set and cleared when a page is inserted into or removed
 * from a page queue, respectively.  It determines whether the plinks.q field
 * of the page is valid.  To set or clear this flag, page's "queue" field must
 * be a valid queue index, and the corresponding page queue lock must be held.
 *
 * PGA_DEQUEUE is set when the page is scheduled to be dequeued from a page
 * queue, and cleared when the dequeue request is processed.  A page may
 * have PGA_DEQUEUE set and PGA_ENQUEUED cleared, for instance if a dequeue
 * is requested after the page is scheduled to be enqueued but before it is
 * actually inserted into the page queue.
 *
 * PGA_REQUEUE is set when the page is scheduled to be enqueued or requeued
 * in its page queue.
 *
 * PGA_REQUEUE_HEAD is a special flag for enqueuing pages near the head of
 * the inactive queue, thus bypassing LRU.
 * The PGA_DEQUEUE, PGA_REQUEUE and PGA_REQUEUE_HEAD flags must be set using an
 * atomic RMW operation to ensure that the "queue" field is a valid queue index,
 * and the corresponding page queue lock must be held when clearing any of the
 * flags.
 */
 enum {
	/* The page is enqueued in a page queue. */
	PGQ_ENQUEUED		= 0x0008,
	/* The page is due to be dequeued. */
	PGQ_DEQUEUE		= 0x0010,
	/* The page is due to be requeued. */
	PGQ_REQUEUE		= 0x0020,
	/* The page requeue should bypass LRU. */
	PGQ_REQUEUE_HEAD	= 0x0040,
	PGQ_OP_MASK		=
	    (PGQ_ENQUEUED | PGQ_REQUEUE | PGQ_REQUEUE_HEAD),
	PGQ_STATE_MASK		= (PGQ_ENQUEUED | PGQ_OP_MASK),
};

enum vm_pqkind {
	PQ_NONE		= 255,
	PQ_INACTIVE	= 0,
	PQ_ACTIVE	= 1,
	PQ_LAUNDRY	= 2,
	PQ_UNSWAPPABLE	= 3,
	PQ_COUNT,
};

union vm_pgstate {
	struct {
		uint8_t		flags;
		uint8_t		queue;
		uint8_t		act_count;
	};
	uint32_t		bits;
};

enum {
	PGL_MARKER = 0x01,
};

struct vm_pglinks {
	union {
		struct {
			union vm_pgstate	state;
			unsigned		next;
			unsigned		prev;
			uint8_t			flags;
		} pq;
		struct {
			void *slab;
			void *zone;
		} uma;
#ifdef DEBUG_MEMGUARD
		struct {
			u_long p;
			u_long v;
		} memguard;
#endif /* DEBUG_MEMGUARD */
	};
};

struct vm_page2 {
	struct vm_pglinks	links;
	struct md_page		md;
	struct vm_object	*object;
	vm_pindex_t		pindex;
	union vm_pgstate	state;
	unsigned		ref_count;
	unsigned		busy_lock;
	uint8_t			flags;
	uint8_t			oflags;
	vm_page_bits_t		valid;
	vm_page_bits_t		dirty;
};

struct vm_pgflags {
	TAILQ_ENTRY(vm)
	vm_paddr_t phys_addr;

};

#ifdef _KERNEL

#include <sys/kassert.h>

/* The pointer to the first resident VM page in table. */
extern struct vm_page vm_page_array[];
extern struct vm_pglinks vm_page_links[];
extern struct md_page vm_page_md_array[];

/* A bump allocator for marker VM pagelinks for the pagedaemons. */
extern struct vm_page_marker_alloc {
	struct mtx		mtx;
	struct vm_pglinks	*markers;
	unsigned		start;
	unsigned		count;
	unsigned		max;
} vm_page_markers;
/* The number of all VM pages. */
extern unsigned long vm_page_array_size;
/* The first physical page number. */
extern unsigned first_page;
extern struct vm_page *bogus_page;
#endif /* _KERNEL */

#ifndef VM_PAGE_HAVE_PGLIST
TAILQ_HEAD(pglist, vm_page);
#define VM_PAGE_HAVE_PGLIST
#endif /* VM_PAGE_HAVE_PGLIST */
SLIST_HEAD(spglist, vm_page);

#if defined(__arm__)
#define PDRSHIFT	PDR_SHIFT
#elif !defined(PDRSHIFT)
#define PDRSHIFT	21
#endif

enum {
	ACT_DECLINE		= 1,
	ACT_ADVANCE		= 3,
	ACT_INIT		= 5,
	ACT_MAX			= 64,
};

#ifdef _KERNEL
#include <machine/atomic.h>
struct pctrie_iter;

/*
 * Each pageable resident page falls into one of five lists:
 *
 *	free
 *		Available for allocation now.
 *
 *	inactive
 *		Low activity, candidates for reclamation.
 *		This list is approximately LRU ordered.
 *
 *	laundry
 *		This is the list of pages that should be
 *		paged out next.
 *
 *	unswappable
 *		Dirty anonymous pages that cannot be paged
 *		out because no swap device is configured.
 *
 *	active
 *		Pages that are "active", i.e., they have been
 *		recently referenced.
 *
 */

/*
 * PHYS_TO_VM_PAGE() returns the vm_page_t object that represents a memory
 * page to which the given physical address belongs. The correct vm_page_t
 * object is returned for addresses that are not page-aligned.
 */
vm_page_t PHYS_TO_VM_PAGE(vm_paddr_t pa);

/*
 * Page allocation parameters for vm_page for the functions
 * vm_page_alloc(), vm_page_grab(), vm_page_alloc_contig() and
 * vm_page_alloc_freelist().  Some functions support only a subset
 * of the flags, and ignore others, see the flags legend.
 *
 * The meaning of VM_ALLOC_ZERO differs slightly between the vm_page_alloc*()
 * and the vm_page_grab*() functions.  See these functions for details.
 *
 * Bits 0 - 1 define class.
 * Bits 2 - 15 dedicated for flags.
 * Legend:
 * (a) - vm_page_alloc() supports the flag.
 * (c) - vm_page_alloc_contig() supports the flag.
 * (g) - vm_page_grab() supports the flag.
 * (n) - vm_page_alloc_noobj() and vm_page_alloc_freelist() support the flag.
 * (p) - vm_page_grab_pages() supports the flag.
 * Bits above 15 define the count of additional pages that the caller
 * intends to allocate.
 */
enum {
	VM_ALLOC_NORMAL		= 0,
	VM_ALLOC_INTERRUPT	= 1,
	VM_ALLOC_SYSTEM		= 2,
	VM_ALLOC_CLASS_MASK	= 3,
	/* (acn) Sleep and retry. */
	VM_ALLOC_WAITOK		= 0x0008,
	/* (acn) Sleep and return error. */
	VM_ALLOC_WAITFAIL	= 0x0010,
	/* (acgnp) Allocate a wired page. */
	VM_ALLOC_WIRED		= 0x0020,
	/* (acgnp) Allocate a zeroed page. */
	VM_ALLOC_ZERO		= 0x0040,
	/* (c) Do not reclaim after failure. */
	VM_ALLOC_NORECLAIM	= 0x0080,
	/* (an) Page will never be released. */
	VM_ALLOC_NOFREE		= 0x0100,
	/* (acgp) Do not excl busy the page. */
	VM_ALLOC_NOBUSY		= 0x0200,
	/* (gp) Don't create a page. */
	VM_ALLOC_NOCREAT	= 0x0400,
	VM_ALLOC_AVAIL1		= 0x0800,
	/* (gp) Ignore shared busy flag. */
	VM_ALLOC_IGN_SBUSY	= 0x1000,
	/* (ag) don't include in dump. */
	VM_ALLOC_NODUMP		= 0x2000,
	/* (acgp) Shared busy the page. */
	VM_ALLOC_SBUSY		= 0x4000,
	/* (acgnp) Do not sleep.  */
	VM_ALLOC_NOWAIT		= 0x8000,
	VM_ALLOC_COUNT_MAX	= 0xffff,
};

#define VM_ALLOC_COUNT_SHIFT	16
#define VM_ALLOC_COUNT_MASK	(VM_ALLOC_COUNT(VM_ALLOC_COUNT_MAX))
#define VM_ALLOC_COUNT(count)	({				\
	KASSERT((count) <= VM_ALLOC_COUNT_MAX,			\
	    ("%s: invalid VM_ALLOC_COUNT value", __func__));	\
	(count) << VM_ALLOC_COUNT_SHIFT;			\
})

#ifdef M_NOWAIT
static inline int
malloc2vm_flags(int malloc_flags)
{
	int pflags;

	KASSERT((malloc_flags & M_USE_RESERVE) == 0 ||
	    (malloc_flags & M_NOWAIT) != 0,
	    ("M_USE_RESERVE requires M_NOWAIT"));
	pflags = (malloc_flags & M_USE_RESERVE) != 0 ? VM_ALLOC_INTERRUPT :
	    VM_ALLOC_SYSTEM;
	if ((malloc_flags & M_ZERO) != 0)
		pflags |= VM_ALLOC_ZERO;
	if ((malloc_flags & M_NODUMP) != 0)
		pflags |= VM_ALLOC_NODUMP;
	if ((malloc_flags & M_NOWAIT))
		pflags |= VM_ALLOC_NOWAIT;
	if ((malloc_flags & M_WAITOK))
		pflags |= VM_ALLOC_WAITOK;
	if ((malloc_flags & M_NORECLAIM))
		pflags |= VM_ALLOC_NORECLAIM;
	if ((malloc_flags & M_NEVERFREED))
		pflags |= VM_ALLOC_NOFREE;
	return (pflags);
}
#endif /* M_NOWAIT */

/*
 * Predicates supported by vm_page_ps_test():
 *
 *	PS_ALL_DIRTY is true only if the entire (super)page is dirty.
 *	However, it can be spuriously false when the (super)page has become
 *	dirty in the pmap but that information has not been propagated to the
 *	machine-independent layer.
 */
#define PS_ALL_DIRTY	0x1
#define PS_ALL_VALID	0x2
#define PS_NONE_BUSY	0x4

vm_offset_t vm_page_startup(vm_offset_t vaddr);
void vm_page_init_page(vm_page_t m, vm_paddr_t pa, int segind, int pool);
void vm_page_initfake(vm_page_t m, vm_paddr_t paddr, vm_memattr_t memattr);
/* DEPRACATED */
void vm_page_init_marker(vm_page_t marker, int queue, uint16_t aflags);

void vm_page_bits_set(struct vm_page *m, vm_page_bits_t *bits, vm_page_bits_t set);
struct vm_page *vm_page_mpred(struct vm_object *object, vm_pindex_t pindex);
struct vm_page *vm_page_next(const struct vm_page *m);
struct vm_page *vm_page_prev(const struct vm_page *m);

bool vm_page_busy_acquire(struct vm_page *m, int allocflags);
void vm_page_busy_downgrade(struct vm_page *m);
int vm_page_busy_tryupgrade(struct vm_page *m);
bool vm_page_busy_sleep(struct vm_page *m, const char *msg, int allocflags);
void vm_page_busy_sleep_unlocked(struct vm_object *obj, struct vm_page *m,
    vm_pindex_t pindex, const char *wmesg, int allocflags);
int vm_page_trysbusy(vm_page_t m);
int vm_page_tryxbusy(vm_page_t m);
int vm_page_sbusied(vm_page_t m);
void vm_page_sunbusy(vm_page_t m);
void vm_page_xunbusy_hard(vm_page_t m);
void vm_page_xunbusy_hard_unchecked(vm_page_t m);

void vm_page_free(struct vm_page *m);
void vm_page_free_zero(struct vm_page *m);
int vm_page_free_pages_toq(struct spglist *free, bool update_wire_count);
void vm_page_free_invalid(vm_page_t);

void vm_page_activate(struct vm_page *m);
void vm_page_deactivate(vm_page_t);
void vm_page_deactivate_noreuse(vm_page_t);
void vm_page_launder(vm_page_t m);
void vm_page_unswappable(vm_page_t m);

void vm_page_dequeue(vm_page_t m);
void vm_page_dequeue_deferred(vm_page_t m);

void vm_page_advise(vm_page_t m, int advice);

struct vm_page *vm_page_alloc(struct vm_page *m, vm_pindex_t pindex, int req);
struct vm_page *vm_page_alloc_domain_after(struct vm_object *object,
    vm_pindex_t pindex, int domain, int req, struct vm_page *mpred);
struct vm_page *vm_page_alloc_contig(struct vm_object *object,
    vm_pindex_t pindex, int req, u_long npages, vm_paddr_t low, vm_paddr_t high,
    u_long alignment, vm_paddr_t boundary, vm_memattr_t memattr);
struct vm_page *vm_page_alloc_contig_domain(struct vm_object *object,
    vm_pindex_t pindex, int domain, int req, u_long npages, vm_paddr_t low,
    vm_paddr_t high, u_long alignment, vm_paddr_t boundary,
    vm_memattr_t memattr);
struct vm_page *vm_page_alloc_noobj(int req);
struct vm_page *vm_page_alloc_noobj_domain(int domain, int req);
struct vm_page *vm_page_alloc_noobj_contig(int req, u_long npages,
    vm_paddr_t low, vm_paddr_t high, u_long alignment, vm_paddr_t boundary,
    vm_memattr_t memattr);
struct vm_page *vm_page_alloc_noobj_contig_domain(int domain, int req,
    u_long npages, vm_paddr_t low, vm_paddr_t high, u_long alignment,
    vm_paddr_t boundary, vm_memattr_t memattr);

bool vm_page_blacklist_add(vm_paddr_t pa, bool verbose);

struct vm_page *vm_page_grab(struct vm_object *object, vm_pindex_t pindex, int req);
struct vm_page *vm_page_grab_unlocked(struct vm_object *object, vm_pindex_t pindex, int req);
int vm_page_grab_pages(struct vm_object *object, vm_pindex_t pindex, int allocflags,
    vm_page_t *ma, int count);
int vm_page_grab_pages_unlocked(struct vm_object *object, vm_pindex_t pindex,
    int allocflags, vm_page_t *ma, int count);
int vm_page_grab_valid(vm_page_t *mp, struct vm_object *object, vm_pindex_t pindex,
    int allocflags);
int vm_page_grab_valid_unlocked(vm_page_t *mp, struct vm_object *object,
    vm_pindex_t pindex, int allocflags);

vm_page_t vm_page_find_least(struct vm_object *object, vm_pindex_t pindex);
vm_page_t vm_page_iter_lookup_ge(struct pctrie_iter *, vm_pindex_t);
vm_page_t vm_page_getfake(vm_paddr_t paddr, vm_memattr_t memattr);

int vm_page_insert (struct vm_page *m, struct vm_object *object, vm_pindex_t pindex);
void vm_page_invalid(struct vm_page *m);
void vm_page_iter_free(struct pctrie_iter *pages, vm_page_t m);
void vm_page_iter_init(struct pctrie_iter *, struct vm_object *object);
void vm_page_iter_limit_init(struct pctrie_iter *, struct vm_object *object, vm_pindex_t pindex);
vm_page_t vm_page_iter_lookup(struct pctrie_iter *, vm_pindex_t pindex);
bool vm_page_iter_remove(struct pctrie_iter *pages, struct vm_page *m);
bool vm_page_iter_rename(struct pctrie_iter *old_pages, struct vm_page *m,
    struct vm_object *new_object, vm_pindex_t new_pindex);

vm_page_t vm_page_lookup(struct vm_object *object, vm_pindex_t pindex);
vm_page_t vm_page_lookup_unlocked(struct vm_object *object, vm_pindex_t pindex);

void vm_page_pqbatch_drain(void);
void vm_page_pqbatch_submit(vm_page_t m, uint8_t queue);
bool vm_page_pqstate_commit(vm_page_t m, union vm_pgstate *old,
    union vm_pgstate new);
bool vm_page_ps_test(vm_page_t m, int psind, int flags, vm_page_t skip_m);
void vm_page_putfake(vm_page_t m);
void vm_page_readahead_finish(vm_page_t m);
int vm_page_reclaim_contig(int req, u_long npages, vm_paddr_t low,
    vm_paddr_t high, u_long alignment, vm_paddr_t boundary);
int vm_page_reclaim_contig_domain(int domain, int req, u_long npages,
    vm_paddr_t low, vm_paddr_t high, u_long alignment, vm_paddr_t boundary);
int vm_page_reclaim_contig_domain_ext(int domain, int req, u_long npages,
    vm_paddr_t low, vm_paddr_t high, u_long alignment, vm_paddr_t boundary,
    int desired_runs);
void vm_page_reference(vm_page_t m);
#define	VPR_TRYFREE	0x01
#define	VPR_NOREUSE	0x02
void vm_page_release(vm_page_t m, int flags);
void vm_page_release_locked(vm_page_t m, int flags);
vm_page_t vm_page_relookup(struct vm_object *object, vm_pindex_t pindex);
bool vm_page_remove(struct vm_page *m);
bool vm_page_remove_xbusy(struct vm_page *m);
void vm_page_replace(struct vm_page *mnew, struct vm_object *object,
    vm_pindex_t pindex, struct vm_page *mold);

vm_page_bits_t vm_page_set_dirty(vm_page_t m);
void vm_page_set_valid_range(vm_page_t m, int base, int size);

bool vm_page_try_remove_all(vm_page_t m);
bool vm_page_try_remove_write(vm_page_t m);

void vm_page_wire(vm_page_t);
void vm_page_unwire(vm_page_t m, uint8_t queue);
bool vm_page_wire_mapped(vm_page_t m);
bool vm_page_unwire_noq(vm_page_t m);

void vm_page_unhold_pages(vm_page_t *ma, int count);
void vm_page_updatefake(vm_page_t m, vm_paddr_t paddr, vm_memattr_t memattr);


void vm_page_set_validclean (vm_page_t, int, int);
void vm_page_clear_dirty(vm_page_t, int, int);
void vm_page_dirty_KBI(vm_page_t m);
void vm_page_set_invalid(vm_page_t, int, int);
void vm_page_valid(vm_page_t m);
int vm_page_is_valid(vm_page_t, int, int);
void vm_page_test_dirty(vm_page_t);
vm_page_bits_t vm_page_bits(int base, int size);
void vm_page_zero_invalid(vm_page_t m, boolean_t setvalid);

static inline unsigned
vm_page_busy_fetch(const struct vm_page *m)
{
	return (atomic_load_int(&m->busy_lock));
}

#define	vm_page_assert_busied(m)					\
	KASSERT(vm_page_busied(m),					\
	    ("vm_page_assert_busied: page %p not busy @ %s:%d", \
	    (m), __FILE__, __LINE__))

#define	vm_page_assert_sbusied(m)					\
	KASSERT(vm_page_sbusied(m),					\
	    ("vm_page_assert_sbusied: page %p not shared busy @ %s:%d", \
	    (m), __FILE__, __LINE__))

#define	vm_page_assert_unbusied(m)					\
	KASSERT((vm_page_busy_fetch(m) & ~VPB_BIT_WAITERS) !=		\
	    VPB_CURTHREAD_EXCLUSIVE,					\
	    ("vm_page_assert_unbusied: page %p busy_lock %#x owned"	\
	     " by me (%p) @ %s:%d",					\
	    (m), (m)->busy_lock, curthread, __FILE__, __LINE__));	\

#define	vm_page_assert_xbusied_unchecked(m) do {			\
	KASSERT(vm_page_xbusied(m),					\
	    ("vm_page_assert_xbusied: page %p not exclusive busy @ %s:%d", \
	    (m), __FILE__, __LINE__));					\
} while (0)
#define	vm_page_assert_xbusied(m) do {					\
	vm_page_assert_xbusied_unchecked(m);				\
	KASSERT((vm_page_busy_fetch(m) & ~VPB_BIT_WAITERS) ==		\
	    VPB_CURTHREAD_EXCLUSIVE,					\
	    ("vm_page_assert_xbusied: page %p busy_lock %#x not owned"	\
	     " by me (%p) @ %s:%d",					\
	    (m), (m)->busy_lock, curthread, __FILE__, __LINE__));	\
} while (0)

#define	vm_page_busied(m)						\
	(vm_page_busy_fetch(m) != VPB_UNBUSIED)

#define	vm_page_xbusied(m)						\
	((vm_page_busy_fetch(m) & VPB_SINGLE_EXCLUSIVE) != 0)

#define	vm_page_busy_freed(m)						\
	(vm_page_busy_fetch(m) == VPB_FREED)

/* Note: page m's lock must not be owned by the caller. */
#define	vm_page_xunbusy(m) do {						\
	if (!atomic_cmpset_rel_int(&(m)->busy_lock,			\
	    VPB_CURTHREAD_EXCLUSIVE, VPB_UNBUSIED))			\
		vm_page_xunbusy_hard(m);				\
} while (0)
#define	vm_page_xunbusy_unchecked(m) do {				\
	if (!atomic_cmpset_rel_int(&(m)->busy_lock,			\
	    VPB_CURTHREAD_EXCLUSIVE, VPB_UNBUSIED))			\
		vm_page_xunbusy_hard_unchecked(m);			\
} while (0)

#ifdef INVARIANTS
void vm_page_object_busy_assert(vm_page_t m);
#define	VM_PAGE_OBJECT_BUSY_ASSERT(m)	vm_page_object_busy_assert(m)
void vm_page_assert_pga_writeable(vm_page_t m, uint16_t bits);
#define	VM_PAGE_ASSERT_PGA_WRITEABLE(m, bits)				\
	vm_page_assert_pga_writeable(m, bits)
/*
 * Claim ownership of a page's xbusy state.  In non-INVARIANTS kernels this
 * operation is a no-op since ownership is not tracked.  In particular
 * this macro does not provide any synchronization with the previous owner.
 */
#define	vm_page_xbusy_claim(m) do {					\
	u_int _busy_lock;						\
									\
	vm_page_assert_xbusied_unchecked((m));				\
	do {								\
		_busy_lock = vm_page_busy_fetch(m);			\
	} while (!atomic_cmpset_int(&(m)->busy_lock, _busy_lock,	\
	    (_busy_lock & VPB_BIT_FLAGMASK) | VPB_CURTHREAD_EXCLUSIVE)); \
} while (0)
#else
#define	VM_PAGE_OBJECT_BUSY_ASSERT(m)	(void)0
#define	VM_PAGE_ASSERT_PGA_WRITEABLE(m, bits)	(void)0
#define	vm_page_xbusy_claim(m)
#endif

#if BYTE_ORDER == BIG_ENDIAN
#define	VM_PAGE_AFLAG_SHIFT	16
#else
#define	VM_PAGE_AFLAG_SHIFT	0
#endif

/*
 * Load a snapshot of a page's 32-bit atomic state.
 */
static inline union vm_pgstate
vm_pgstate_load(const struct vm_page *m)
{
	union vm_pgstate state;

	state.bits = atomic_load_32(&vm_page_links[m->pgind].pq.state.bits);
	return (state);
}

/*
 * Atomically compare and set a page's atomic state.
 */
static inline bool
vm_pgstate_fcmpset(struct vm_pglinks *links, union vm_pgstate *old,
    union vm_pgstate new)
{
	KASSERT(new.queue == PQ_INACTIVE || (new.flags & PGQ_REQUEUE_HEAD) == 0,
	    ("%s: invalid head requeue request for page %p", __func__, links));
	KASSERT((new.flags & PGQ_ENQUEUED) == 0 || new.queue != PQ_NONE,
	    ("%s: setting PGA_ENQUEUED with PQ_NONE in page %p", __func__, links));
	KASSERT(new.bits != old->bits,
	    ("%s: bits are unchanged", __func__));

	return (atomic_fcmpset_32(&links->pq.state.bits, &old->bits, new.bits) != 0);
}

/*
 * Clear the given bits in the specified page.
 */
static inline void
vm_page_state_clear(struct vm_page *m, uint8_t state)
{
	atomic_clear_8(&m->state, state);
}

/*
 *	Set the given bits in the specified page.
 */
static inline void
vm_page_aflag_set(vm_page_t m, uint8_t bits)
{
	VM_PAGE_ASSERT_PGA_WRITEABLE(m, bits);

	atomic_set_8(&m->state, bits);
}

/*
 * Set all bits in the page's dirty field.
 *
 * The object containing the specified page must be locked if the call is made
 * from the machine-independent layer.
 *
 * See vm_page_clear_dirty_mask().
 */
static inline void
vm_page_dirty(struct vm_page *m)
{
	/* Use vm_page_dirty_KBI() under INVARIANTS to save memory. */
#if (defined(KLD_MODULE) && !defined(KLD_TIED)) || defined(INVARIANTS)
	vm_page_dirty_KBI(m);
#else
	m->dirty = VM_PAGE_BITS_ALL;
#endif
}

/*
 * Set page to not be dirty.  Note: does not clear pmap modify bits
 */
static inline void
vm_page_undirty(struct vm_page *m)
{
	VM_PAGE_OBJECT_BUSY_ASSERT(m);
	m->dirty = 0;
}

static inline enum vm_pqkind
vm_pgstate_queue(union vm_pgstate state)
{
	if ((state.flags & PGQ_DEQUEUE) != 0)
		return (PQ_NONE);
	return (state.queue);
}

/*
 * Return the index of the queue containing m.
 */
static inline enum vm_pqkind
vm_page_queue(const struct vm_page *m)
{
	return (vm_pgstate_queue(vm_pgstate_load(m)));
}

static inline bool
vm_page_active(const struct vm_page *m)
{
	return (vm_page_queue(m) == PQ_ACTIVE);
}

static inline bool
vm_page_inactive(const struct vm_page *m)
{

	return (vm_page_queue(m) == PQ_INACTIVE);
}

static inline bool
vm_page_in_laundry(const struct vm_page *m)
{
	enum vm_pqkind queue;

	queue = vm_page_queue(m);
	return (queue == PQ_LAUNDRY || queue == PQ_UNSWAPPABLE);
}

static inline void
vm_page_clearref(struct vm_page *m)
{
	unsigned ref_count;

	ref_count = m->ref_count;
	while (atomic_fcmpset_int(&m->ref_count, &ref_count,
	    ref_count & (VPRC_BLOCKED | VPRC_OBJREF)) == 0) {}
}

/*
 * Release a reference to a page and return the old reference count.
 */
static inline unsigned
vm_page_drop(struct vm_page *m, unsigned refs)
{
	unsigned old;

	/*
	 * Synchronize with vm_page_free_prep(): ensure that all updates to the
	 * page structure are visible before it is freed.
	 */
	atomic_thread_fence_rel();
	old = atomic_fetchadd_int(&m->ref_count, -refs);
	KASSERT(old != VPRC_BLOCKED,
	    ("%s: page %p has an invalid ref_count value", __func__, m));
	return (old);
}

/*
 * Perform a racy check to determine whether a reference prevents the page
 * from being reclaimable.  If the page's object is locked, and the page is
 * unmapped and exclusively busied by the current thread, no new wirings
 * may be created.
 */
static inline bool
vm_page_wired(const struct vm_page *m)
{
	return (vprc_wire_count(m->ref_count) > 0);
}

static inline bool
vm_page_all_valid(const struct vm_page *m)
{
	return (m->valid == VM_PAGE_BITS_ALL);
}

static inline bool
vm_page_any_valid(const struct vm_page *m)
{
	return (m->valid != 0);
}

static inline bool
vm_page_none_valid(const struct vm_page *m)
{
	return (m->valid == 0);
}

static inline int
vm_page_domain(const struct vm_page *m)
{
#ifdef NUMA
	KASSERT((m->flags & PG_FICTITIOUS) == 0,
		("%s: invalid page %p is ficticious", __func__, m));
	return (m->domain);
#else
	(void)m
	return (0);
#endif /* NUMA */
}

#endif /* _KERNEL */
#endif /* !_VM_PAGE_ */
