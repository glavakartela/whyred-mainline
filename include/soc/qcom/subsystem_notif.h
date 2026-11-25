/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <soc/qcom/subsystem_notif.h> for the IPA v2 port to Linux 6.x.
 *
 * Downstream Qualcomm SSR (Sub-System Restart) notification framework.
 * The symbols rmnet_ipa.c actually uses from this header are:
 *   - subsys_notif_register_notifier()
 *   - subsys_notif_unregister_notifier()
 *
 * Both are already provided as inline wrappers in ipa_compat.h that
 * forward to mainline's qcom_register_ssr_notifier() /
 * qcom_unregister_ssr_notifier() from <linux/remoteproc/qcom_rproc.h>.
 *
 * So this header just needs to exist (empty) for the unconditional
 * `#include <soc/qcom/subsystem_notif.h>` in rmnet_ipa.c to resolve.
 */

#ifndef _SOC_QCOM_SUBSYSTEM_NOTIF_H_
#define _SOC_QCOM_SUBSYSTEM_NOTIF_H_

#endif /* _SOC_QCOM_SUBSYSTEM_NOTIF_H_ */
