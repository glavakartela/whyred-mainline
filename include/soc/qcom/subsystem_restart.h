/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <soc/qcom/subsystem_restart.h> for the IPA v2 port to Linux 6.x.
 *
 * The downstream Qualcomm tree shipped a custom subsystem-restart (SSR)
 * framework at drivers/soc/qcom/subsystem_restart.c that exposed APIs
 * like subsys_get(), subsys_put(), subsystem_restart(), enum subsys_event.
 * Mainline 6.x replaces all of this with the qcom_rproc framework
 * (qcom_register_ssr_notifier, etc.) — already shimmed in ipa_compat.h.
 *
 * rmnet_ipa.c includes this header but does NOT actually reference any
 * symbol from it (verified by grep). So an empty stub is sufficient.
 */

#ifndef _SOC_QCOM_SUBSYSTEM_RESTART_H_
#define _SOC_QCOM_SUBSYSTEM_RESTART_H_

#endif /* _SOC_QCOM_SUBSYSTEM_RESTART_H_ */
