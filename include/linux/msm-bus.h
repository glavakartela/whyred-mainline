/* SPDX-License-Identifier: GPL-2.0 */
/*
 * <linux/msm-bus.h> — vendor msm_bus_scale API shim backed by mainline
 * interconnect framework. Implementation in
 * drivers/platform/msm/ipa/ipa_v2/msm_bus_compat.c.
 *
 * The original downstream Qualcomm tree shipped a custom MSM bus scaling
 * framework (drivers/soc/qcom/msm_bus/...). Mainline 6.x replaces this with
 * the interconnect framework. This header keeps the vendor IPA v2 source
 * compiling unchanged by exporting the same function signatures and struct
 * layouts; the msm_bus_compat.c translator routes the calls through icc_*.
 */

#ifndef _LINUX_MSM_BUS_H
#define _LINUX_MSM_BUS_H

#include <linux/types.h>
#include <linux/platform_device.h>

struct msm_bus_vectors {
	int src;	/* MSM_BUS_MASTER_*  */
	int dst;	/* MSM_BUS_SLAVE_*   */
	u64 ab;		/* average bandwidth (bytes/sec) */
	u64 ib;		/* instantaneous bandwidth */
};

struct msm_bus_paths {
	int num_paths;
	struct msm_bus_vectors *vectors;
};

struct msm_bus_scale_pdata {
	struct msm_bus_paths *usecase;
	int num_usecases;
	const char *name;
	bool active_only;
};

/*
 * msm_bus_compat_set_device() — call ONCE during IPA driver probe with
 * the platform_device whose DT node carries the named "interconnects"
 * properties. Must precede any msm_bus_scale_register_client() call.
 */
void msm_bus_compat_set_device(struct device *dev);

/* Vendor API surface — implemented by msm_bus_compat.c */
struct msm_bus_scale_pdata *
msm_bus_cl_get_pdata(struct platform_device *pdev);

void msm_bus_cl_clear_pdata(struct msm_bus_scale_pdata *pdata);

u32 msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata);

void msm_bus_scale_unregister_client(u32 cl);

int msm_bus_scale_client_update_request(u32 cl, unsigned int index);

#endif /* _LINUX_MSM_BUS_H */
