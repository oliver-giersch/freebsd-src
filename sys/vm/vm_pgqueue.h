#ifndef _VM_PGQUEUE_
#define _VM_PGQUEUE_

struct vm_pgtailq {
	unsigned head;
	unsigned tail;
};

static inline void
vm_pgtailq_init(struct vm_pgtailq *q)
{
	q->head = 0;
	q->tail = 0;
}

static inline void
vm_pgtailq_insert_head(struct vm_pgtailq *q)
{
	(void)q;
}

#endif /* _VM_PGQUEUE_ */