/* SPDX-License-Identifier: GPL-2.0 */
/*
 * IPA v2 portability shim for Linux 6.x.
 *
 * Force-included into every TU via Makefile `ccflags-y += -include ...`.
 *
 * What this header still does (after sed patches in apply-port-patches.sh
 * and stub headers in include/linux/ took over most of the work):
 *
 *   - SSR notifier wrappers: SUBSYS_* code defines + inline wrappers
 *     around qcom_register_ssr_notifier(). Used by rmnet_ipa.c.
 *   - dma_zalloc_coherent fallback macro (defensive — sed also rewrites).
 *
 *   - (history) an iommu_domain_get_attr() no-op stub lived here while
 *     the SMMU attribute reads were unresolved; the call sites now read
 *     per-CB DT properties instead (20-apply-port-fixes.sh, "smmu").
 *
 * What this header explicitly does NOT do anymore (and why):
 *
 *   - debugfs_create_uN/xN/bool/size_t/atomic_t macros: they shadowed
 *     the kernel's prototypes and broke <linux/debugfs.h> parsing.
 *     Now handled directly by sed in apply-port-patches.sh: the script
 *     drops the LHS assignment from each call site and pre-initializes
 *     the relevant dentry pointers to a non-NULL non-IS_ERR sentinel.
 *
 *   - IPC logging stubs (ipc_log_context_create / _destroy /
 *     ipc_log_string): provided by include/linux/ipc_logging.h
 *     (either kernel's own or our stub at the same path). Defining
 *     them here would conflict.
 *
 * See PORTING_NOTES.md for the full break catalog.
 */

#ifndef _IPA_COMPAT_H_
#define _IPA_COMPAT_H_

#include <linux/version.h>
#include <linux/notifier.h>
#include <linux/panic_notifier.h>	/* panic_notifier_list (5.18+) */
#include <linux/sched/clock.h>		/* local_clock() */
#include <linux/net.h>			/* kernel_connect() */
#include <linux/printk.h>
#include <linux/types.h>

/*=========================================================================
 * SSR (Sub-System Restart) notifier shim.
 *
 * Downstream uses subsys_notif_register_notifier(SUBSYS_MODEM, &nb) and
 * codes SUBSYS_BEFORE_SHUTDOWN / SUBSYS_AFTER_SHUTDOWN /
 * SUBSYS_BEFORE_POWERUP / SUBSYS_AFTER_POWERUP.
 *
 * Mainline 6.x replaces this with qcom_register_ssr_notifier("mpss", &nb)
 * from <linux/remoteproc/qcom_rproc.h>, with codes from
 * enum qcom_ssr_notify_type.
 *
 * Note: SUBSYS_MODEM is defined in vendor's ipa_qmi_service.h with the
 * downstream value "modem". The apply-port-patches.sh script rewrites it
 * to "mpss" so that qcom_register_ssr_notifier() finds the modem
 * remoteproc instance — mainline names it "mpss".
 *=========================================================================*/

/*
 * Map vendor SUBSYS_* notification codes onto mainline qcom_ssr_notify_type
 * enum values (same numerical order in qcom_rproc.h):
 *   QCOM_SSR_BEFORE_POWERUP = 0
 *   QCOM_SSR_AFTER_POWERUP  = 1
 *   QCOM_SSR_BEFORE_SHUTDOWN = 2
 *   QCOM_SSR_AFTER_SHUTDOWN = 3
 */
#define SUBSYS_BEFORE_POWERUP	0
#define SUBSYS_AFTER_POWERUP	1
#define SUBSYS_BEFORE_SHUTDOWN	2
#define SUBSYS_AFTER_SHUTDOWN	3

#if IS_ENABLED(CONFIG_QCOM_RPROC_COMMON)
#include <linux/remoteproc/qcom_rproc.h>

static inline void *
subsys_notif_register_notifier(const char *subsys_name,
			       struct notifier_block *nb)
{
	return qcom_register_ssr_notifier(subsys_name, nb);
}

static inline int
subsys_notif_unregister_notifier(void *notif_handle,
				 struct notifier_block *nb)
{
	if (IS_ERR_OR_NULL(notif_handle))
		return 0;
	return qcom_unregister_ssr_notifier(notif_handle, nb);
}
#else
/* No qcom_rproc available: SSR is a no-op. The driver still loads but
 * modem-restart events will not be delivered.
 */
static inline void *
subsys_notif_register_notifier(const char *subsys_name,
			       struct notifier_block *nb)
{
	return NULL;
}

static inline int
subsys_notif_unregister_notifier(void *notif_handle,
				 struct notifier_block *nb)
{
	return 0;
}
#endif /* CONFIG_QCOM_RPROC_COMMON */

/*=========================================================================
 * dma_zalloc_coherent removal (kernel 5.0).
 * dma_alloc_coherent always zeroes since 5.0.
 *
 * Belt-and-braces with the sed in apply-port-patches.sh — either alone
 * suffices, both together protects against stragglers.
 *=========================================================================*/

#ifndef dma_zalloc_coherent
#define dma_zalloc_coherent(dev, size, dma_handle, gfp) \
	dma_alloc_coherent((dev), (size), (dma_handle), (gfp))
#endif

#endif /* _IPA_COMPAT_H_ */
