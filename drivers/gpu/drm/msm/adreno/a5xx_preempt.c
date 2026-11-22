// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2017 The Linux Foundation. All rights reserved.
 */

#include <linux/moduleparam.h>

#include "msm_gem.h"
#include "a5xx_gpu.h"

/*
 * Diagnostic/test knob: if the context switch has not completed but the
 * GPU is still retiring ALU work, wait a bounded amount of extra time
 * instead of resetting it immediately.
 */
static unsigned int preempt_progress_max_ms = 30000;
module_param(preempt_progress_max_ms, uint, 0644);
MODULE_PARM_DESC(preempt_progress_max_ms,
	"A5xx: maximum ALU-progress extension of the preemption watchdog in ms");

/*
 * Try to transition the preemption state from old to new. Return
 * true on success or false if the original state wasn't 'old'
 */
static inline bool try_preempt_state(struct a5xx_gpu *a5xx_gpu,
		enum preempt_state old, enum preempt_state new)
{
	enum preempt_state cur = atomic_cmpxchg(&a5xx_gpu->preempt_state,
		old, new);

	return (cur == old);
}

/*
 * Force the preemption state to the specified state.  This is used in cases
 * where the current state is known and won't change
 */
static inline void set_preempt_state(struct a5xx_gpu *gpu,
		enum preempt_state new)
{
	/*
	 * preempt_state may be read by other cores trying to trigger a
	 * preemption or in the interrupt handler so barriers are needed
	 * before...
	 */
	smp_mb__before_atomic();
	atomic_set(&gpu->preempt_state, new);
	/* ... and after*/
	smp_mb__after_atomic();
}

/* Write the most recent wptr for the given ring into the hardware */
static inline void update_wptr(struct msm_gpu *gpu, struct msm_ringbuffer *ring)
{
	unsigned long flags;
	uint32_t wptr;

	if (!ring)
		return;

	spin_lock_irqsave(&ring->preempt_lock, flags);
	wptr = get_wptr(ring);
	gpu_write(gpu, REG_A5XX_CP_RB_WPTR, wptr);
	spin_unlock_irqrestore(&ring->preempt_lock, flags);
}

/* Return the highest priority ringbuffer with something in it */
static struct msm_ringbuffer *get_next_ring(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	unsigned long flags;
	int i;

	for (i = 0; i < gpu->nr_rings; i++) {
		bool empty;
		struct msm_ringbuffer *ring = gpu->rb[i];

		spin_lock_irqsave(&ring->preempt_lock, flags);
		empty = (get_wptr(ring) == gpu->funcs->get_rptr(gpu, ring));
		if (!empty && ring == a5xx_gpu->cur_ring)
			empty = ring->memptrs->fence == a5xx_gpu->last_seqno[i];
		spin_unlock_irqrestore(&ring->preempt_lock, flags);

		if (!empty)
			return ring;
	}

	return NULL;
}

static void a5xx_preempt_timer(struct timer_list *t)
{
	struct a5xx_gpu *a5xx_gpu = timer_container_of(a5xx_gpu, t,
						       preempt_timer);
	struct msm_gpu *gpu = &a5xx_gpu->base.base;
	struct drm_device *dev = gpu->dev;
	struct msm_ringbuffer *cur, *next;
	struct a5xx_preempt_record *record;
	struct msm_cp_state cp_state = {
		.ib1_base = gpu_read64(gpu, REG_A5XX_CP_IB1_BASE),
		.ib2_base = gpu_read64(gpu, REG_A5XX_CP_IB2_BASE),
		.ib1_rem  = gpu_read(gpu, REG_A5XX_CP_IB1_BUFSZ),
		.ib2_rem  = gpu_read(gpu, REG_A5XX_CP_IB2_BUFSZ),
	};
	u64 alu_cycles, baseline_alu_cycles;
	unsigned long progress_ms = 0;
	u32 int_status, int_mask;

	if (atomic_read(&a5xx_gpu->preempt_state) != PREEMPT_TRIGGERED)
		return;

	alu_cycles = gpu_read64(gpu, REG_A5XX_RBBM_PERFCTR_SP_0_LO);
	baseline_alu_cycles = READ_ONCE(a5xx_gpu->preempt_alu_cycles);

	if (preempt_progress_max_ms &&
	    alu_cycles != baseline_alu_cycles) {
		if (!a5xx_gpu->preempt_progress_start)
			a5xx_gpu->preempt_progress_start = jiffies;

		progress_ms = jiffies_to_msecs(jiffies -
				a5xx_gpu->preempt_progress_start);

		if (progress_ms < preempt_progress_max_ms) {
			WRITE_ONCE(a5xx_gpu->preempt_alu_cycles, alu_cycles);
			DRM_DEV_DEBUG(dev->dev,
				"%s: preemption pending, ALU progressing for %lu ms: extend\n",
				gpu->name, progress_ms);
			mod_timer(&a5xx_gpu->preempt_timer,
				  jiffies + msecs_to_jiffies(10000));
			return;
		}
	}

	if (!try_preempt_state(a5xx_gpu, PREEMPT_TRIGGERED, PREEMPT_FAULTED))
		return;

	cur = a5xx_gpu->cur_ring;
	next = a5xx_gpu->next_ring;
	record = next ? a5xx_gpu->preempt[next->id] : NULL;
	/* Preserve the state that explains why the context switch timed out. */
	int_status = gpu_read(gpu, REG_A5XX_RBBM_INT_0_STATUS);
	int_mask = gpu_read(gpu, REG_A5XX_RBBM_INT_0_MASK);
	DRM_DEV_ERROR(dev->dev,
		"%s: preemption timed out: cur=%d next=%d switch=%08x "
		"rbbm=%08x/%08x int=%08x/%08x alu=%016llx baseline=%016llx "
		"alu_changed=%u cp_changed=%u progress_ms=%lu\n",
		gpu->name, cur ? cur->id : -1, next ? next->id : -1,
		gpu_read(gpu, REG_A5XX_CP_CONTEXT_SWITCH_CNTL),
		gpu_read(gpu, REG_A5XX_RBBM_STATUS),
		gpu_read(gpu, REG_A5XX_RBBM_STATUS3), int_status, int_mask,
		(unsigned long long)alu_cycles,
		(unsigned long long)baseline_alu_cycles,
		alu_cycles != baseline_alu_cycles,
		memcmp(&cp_state, &a5xx_gpu->preempt_cp_state,
		       sizeof(cp_state)) != 0,
		progress_ms);
	DRM_DEV_ERROR(dev->dev,
		"%s: preemption rings: cur_fence=%u/%u rptr/wptr=%u/%u "
		"ib1=%016llx/%08x ib2=%016llx/%08x\n",
		gpu->name, cur ? cur->memptrs->fence : 0,
		cur ? cur->fctx->last_fence : 0,
		cur ? gpu->funcs->get_rptr(gpu, cur) : 0,
		cur ? get_wptr(cur) : 0,
		(unsigned long long)cp_state.ib1_base, cp_state.ib1_rem,
		(unsigned long long)cp_state.ib2_base, cp_state.ib2_rem);
	if (record)
		DRM_DEV_ERROR(dev->dev,
			"%s: preemption record: magic=%08x info=%08x "
			"data=%08x rptr/wptr=%u/%u\n",
			gpu->name, record->magic, record->info, record->data,
			record->rptr, record->wptr);

	kthread_queue_work(gpu->worker, &gpu->recover_work);
}

/* Try to trigger a preemption switch */
void a5xx_preempt_trigger(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	unsigned long flags;
	struct msm_ringbuffer *ring;

	if (gpu->nr_rings == 1)
		return;

	/*
	 * Serialize preemption start to ensure that we always make
	 * decision on latest state. Otherwise we can get stuck in
	 * lower priority or empty ring.
	 */
	spin_lock_irqsave(&a5xx_gpu->preempt_start_lock, flags);

	/*
	 * Try to start preemption by moving from NONE to START. If
	 * unsuccessful, a preemption is already in flight
	 */
	if (!try_preempt_state(a5xx_gpu, PREEMPT_NONE, PREEMPT_START))
		goto out;

	/* Get the next ring to preempt to */
	ring = get_next_ring(gpu);

	/*
	 * If no ring is populated or the highest priority ring is the current
	 * one do nothing except to update the wptr to the latest and greatest
	 */
	if (!ring || (a5xx_gpu->cur_ring == ring)) {
		/*
		 * Its possible that while a preemption request is in progress
		 * from an irq context, a user context trying to submit might
		 * fail to update the write pointer, because it determines
		 * that the preempt state is not PREEMPT_NONE.
		 *
		 * Close the race by introducing an intermediate
		 * state PREEMPT_ABORT to let the submit path
		 * know that the ringbuffer is not going to change
		 * and can safely update the write pointer.
		 */

		set_preempt_state(a5xx_gpu, PREEMPT_ABORT);
		update_wptr(gpu, a5xx_gpu->cur_ring);
		set_preempt_state(a5xx_gpu, PREEMPT_NONE);
		goto out;
	}

	spin_unlock_irqrestore(&a5xx_gpu->preempt_start_lock, flags);

	/* Make sure the wptr doesn't update while we're in motion */
	spin_lock_irqsave(&ring->preempt_lock, flags);
	a5xx_gpu->preempt[ring->id]->wptr = get_wptr(ring);
	spin_unlock_irqrestore(&ring->preempt_lock, flags);

	/* Set the address of the incoming preemption record */
	gpu_write64(gpu, REG_A5XX_CP_CONTEXT_SWITCH_RESTORE_ADDR_LO,
		a5xx_gpu->preempt_iova[ring->id]);

	a5xx_gpu->next_ring = ring;
	a5xx_gpu->preempt_progress_start = jiffies;
	WRITE_ONCE(a5xx_gpu->preempt_alu_cycles,
		gpu_read64(gpu, REG_A5XX_RBBM_PERFCTR_SP_0_LO));
	a5xx_gpu->preempt_cp_state = (struct msm_cp_state) {
		.ib1_base = gpu_read64(gpu, REG_A5XX_CP_IB1_BASE),
		.ib2_base = gpu_read64(gpu, REG_A5XX_CP_IB2_BASE),
		.ib1_rem  = gpu_read(gpu, REG_A5XX_CP_IB1_BUFSZ),
		.ib2_rem  = gpu_read(gpu, REG_A5XX_CP_IB2_BUFSZ),
	};

	/* Start a timer to catch a stuck preemption */
	mod_timer(&a5xx_gpu->preempt_timer, jiffies + msecs_to_jiffies(10000));

	/* Set the preemption state to triggered */
	set_preempt_state(a5xx_gpu, PREEMPT_TRIGGERED);

	/* Make sure everything is written before hitting the button */
	wmb();

	/* And actually start the preemption */
	gpu_write(gpu, REG_A5XX_CP_CONTEXT_SWITCH_CNTL, 1);
	return;

out:
	spin_unlock_irqrestore(&a5xx_gpu->preempt_start_lock, flags);
}

void a5xx_preempt_irq(struct msm_gpu *gpu)
{
	uint32_t status;
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	struct drm_device *dev = gpu->dev;

	if (!try_preempt_state(a5xx_gpu, PREEMPT_TRIGGERED, PREEMPT_PENDING))
		return;

	/* Delete the preemption watchdog timer */
	timer_delete(&a5xx_gpu->preempt_timer);
	a5xx_gpu->preempt_progress_start = 0;

	/*
	 * The hardware should be setting CP_CONTEXT_SWITCH_CNTL to zero before
	 * firing the interrupt, but there is a non zero chance of a hardware
	 * condition or a software race that could set it again before we have a
	 * chance to finish. If that happens, log and go for recovery
	 */
	status = gpu_read(gpu, REG_A5XX_CP_CONTEXT_SWITCH_CNTL);
	if (unlikely(status)) {
		set_preempt_state(a5xx_gpu, PREEMPT_FAULTED);
		DRM_DEV_ERROR(dev->dev, "%s: Preemption failed to complete\n",
			gpu->name);
		kthread_queue_work(gpu->worker, &gpu->recover_work);
		return;
	}

	a5xx_gpu->cur_ring = a5xx_gpu->next_ring;
	a5xx_gpu->next_ring = NULL;

	update_wptr(gpu, a5xx_gpu->cur_ring);

	set_preempt_state(a5xx_gpu, PREEMPT_NONE);

	/*
	 * Try to trigger preemption again in case there was a submit or
	 * retire during ring switch
	 */
	a5xx_preempt_trigger(gpu);
}

void a5xx_preempt_hw_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	int i;

	/* Always come up on rb 0 */
	a5xx_gpu->cur_ring = gpu->rb[0];

	/* No preemption if we only have one ring */
	if (gpu->nr_rings == 1)
		return;

	for (i = 0; i < gpu->nr_rings; i++) {
		a5xx_gpu->preempt[i]->data = 0;
		a5xx_gpu->preempt[i]->info = 0;
		a5xx_gpu->preempt[i]->wptr = 0;
		a5xx_gpu->preempt[i]->rptr = 0;
		a5xx_gpu->preempt[i]->rbase = gpu->rb[i]->iova;
		a5xx_gpu->preempt[i]->rptr_addr = shadowptr(a5xx_gpu, gpu->rb[i]);
	}

	/* Write a 0 to signal that we aren't switching pagetables */
	gpu_write64(gpu, REG_A5XX_CP_CONTEXT_SWITCH_SMMU_INFO_LO, 0);

	/* Reset the preemption state */
	set_preempt_state(a5xx_gpu, PREEMPT_NONE);
}

static int preempt_init_ring(struct a5xx_gpu *a5xx_gpu,
		struct msm_ringbuffer *ring)
{
	struct adreno_gpu *adreno_gpu = &a5xx_gpu->base;
	struct msm_gpu *gpu = &adreno_gpu->base;
	struct a5xx_preempt_record *ptr;
	void *counters;
	struct drm_gem_object *bo = NULL, *counters_bo = NULL;
	u64 iova = 0, counters_iova = 0;

	ptr = msm_gem_kernel_new(gpu->dev,
		A5XX_PREEMPT_RECORD_SIZE + A5XX_PREEMPT_COUNTER_SIZE,
		MSM_BO_WC | MSM_BO_MAP_PRIV, gpu->vm, &bo, &iova);

	if (IS_ERR(ptr))
		return PTR_ERR(ptr);

	/* The buffer to store counters needs to be unprivileged */
	counters = msm_gem_kernel_new(gpu->dev,
		A5XX_PREEMPT_COUNTER_SIZE,
		MSM_BO_WC, gpu->vm, &counters_bo, &counters_iova);
	if (IS_ERR(counters)) {
		msm_gem_kernel_put(bo, gpu->vm);
		return PTR_ERR(counters);
	}

	msm_gem_object_set_name(bo, "preempt");
	msm_gem_object_set_name(counters_bo, "preempt_counters");

	a5xx_gpu->preempt_bo[ring->id] = bo;
	a5xx_gpu->preempt_counters_bo[ring->id] = counters_bo;
	a5xx_gpu->preempt_iova[ring->id] = iova;
	a5xx_gpu->preempt[ring->id] = ptr;

	/* Set up the defaults on the preemption record */

	ptr->magic = A5XX_PREEMPT_RECORD_MAGIC;
	ptr->info = 0;
	ptr->data = 0;
	ptr->cntl = MSM_GPU_RB_CNTL_DEFAULT | AXXX_CP_RB_CNTL_NO_UPDATE;

	ptr->counter = counters_iova;

	return 0;
}

void a5xx_preempt_fini(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	int i;

	for (i = 0; i < gpu->nr_rings; i++) {
		msm_gem_kernel_put(a5xx_gpu->preempt_bo[i], gpu->vm);
		msm_gem_kernel_put(a5xx_gpu->preempt_counters_bo[i], gpu->vm);
	}
}

void a5xx_preempt_init(struct msm_gpu *gpu)
{
	struct adreno_gpu *adreno_gpu = to_adreno_gpu(gpu);
	struct a5xx_gpu *a5xx_gpu = to_a5xx_gpu(adreno_gpu);
	int i;

	/* No preemption if we only have one ring */
	if (gpu->nr_rings <= 1)
		return;

	for (i = 0; i < gpu->nr_rings; i++) {
		if (preempt_init_ring(a5xx_gpu, gpu->rb[i])) {
			/*
			 * On any failure our adventure is over. Clean up and
			 * set nr_rings to 1 to force preemption off
			 */
			a5xx_preempt_fini(gpu);
			gpu->nr_rings = 1;

			return;
		}
	}

	spin_lock_init(&a5xx_gpu->preempt_start_lock);
	timer_setup(&a5xx_gpu->preempt_timer, a5xx_preempt_timer, 0);
}
