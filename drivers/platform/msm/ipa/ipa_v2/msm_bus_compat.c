// SPDX-License-Identifier: GPL-2.0
/*
 * msm_bus_compat — translate vendor msm_bus_scale_* API to mainline
 * interconnect (icc) framework, for IPA v2 driver port from Android 4.19
 * to mainline Linux 6.x.
 *
 * Vendor IPA driver describes bus bandwidth needs as a 2-D table:
 *   pdata = { usecase[0..N-1], each usecase = { vectors[0..M-1] } }
 *   each vector = { src, dst, ab, ib }   // bandwidth in bytes/sec
 *
 * Mainline icc instead has named per-device paths declared in DT:
 *   interconnects = <&a2noc MASTER_IPA &bimc SLAVE_EBI>, ...
 *   interconnect-names = "ipa-mem", "ipa-imem", "cpu-cfg";
 *
 * This compat layer:
 *   1. Translates (src, dst) numeric pairs into mainline icc path names.
 *   2. Acquires one icc_path per unique pair at register_client time.
 *   3. On update_request, walks the chosen usecase's vectors and
 *      issues icc_set_bw(path, ab_kBps, ib_kBps) for each.
 *
 * Bandwidth unit conversion: vendor stores Bps (e.g. 100_000_000 = 100 MB/s),
 * icc takes KBps. Divide by 1024 (rounded up).
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/interconnect.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#define MSM_BUS_COMPAT_MAX_HANDLES   4
#define MSM_BUS_COMPAT_MAX_PATHS     6

struct msm_bus_compat_path {
	int src;
	int dst;
	struct icc_path *path;
};

struct msm_bus_compat_handle {
	struct msm_bus_scale_pdata *pdata;
	struct msm_bus_compat_path paths[MSM_BUS_COMPAT_MAX_PATHS];
	unsigned int num_paths;
	bool in_use;
};

static struct device *msm_bus_compat_dev;
static struct msm_bus_compat_handle handles[MSM_BUS_COMPAT_MAX_HANDLES];
static DEFINE_MUTEX(msm_bus_compat_lock);

/**
 * msm_bus_compat_set_device() - register the IPA platform device whose
 * DT node carries the named "interconnects" properties. Must be called
 * once before msm_bus_scale_register_client().
 */
void msm_bus_compat_set_device(struct device *dev)
{
	mutex_lock(&msm_bus_compat_lock);
	msm_bus_compat_dev = dev;
	mutex_unlock(&msm_bus_compat_lock);
	dev_info(dev, "msm_bus_compat: registered as icc device\n");
}
EXPORT_SYMBOL(msm_bus_compat_set_device);

/* Map a vendor (src, dst) pair to the mainline icc path name in DT.
 * Returns NULL if no mapping (we silently skip that vector). */
static const char *map_src_dst_to_icc_name(int src, int dst)
{
	if (src == MSM_BUS_MASTER_IPA &&
	    dst == MSM_BUS_SLAVE_EBI_CH0)
		return "ipa-mem";
	if (src == MSM_BUS_MASTER_IPA &&
	    dst == MSM_BUS_SLAVE_OCIMEM)
		return "ipa-imem";
	/* BAM_DMA traffic on v1.1 goes through the same paths; alias to IPA */
	if (src == MSM_BUS_MASTER_BAM_DMA &&
	    dst == MSM_BUS_SLAVE_EBI_CH0)
		return "ipa-mem";
	if (src == MSM_BUS_MASTER_BAM_DMA &&
	    dst == MSM_BUS_SLAVE_OCIMEM)
		return "ipa-imem";
	return NULL;
}

/* Find an already-acquired path with this (src, dst) pair in the handle.
 * Returns NULL if not found. */
static struct icc_path *
find_path(struct msm_bus_compat_handle *h, int src, int dst)
{
	unsigned int i;

	for (i = 0; i < h->num_paths; i++) {
		if (h->paths[i].src == src && h->paths[i].dst == dst)
			return h->paths[i].path;
	}
	return NULL;
}

/* B/s → KB/s, rounded up to avoid silently dropping small votes to 0. */
static u32 bps_to_kbps(u64 bps)
{
	return (u32)DIV_ROUND_UP(bps, 1024);
}

/*
 * Default SDM660 IPA bus pdata — mirrors vendor ipa_nominal_perf_vectors_v2_0
 * from drivers/platform/msm/ipa/ipa_v2/ipa_utils.c. usecase[0] is the idle
 * vote (0/0), usecase[1] is nominal (100 MB/s avg, 1.3 GB/s peak per path).
 */
static struct msm_bus_vectors compat_init_vectors[] = {
	{ MSM_BUS_MASTER_IPA, MSM_BUS_SLAVE_EBI_CH0, 0, 0 },
	{ MSM_BUS_MASTER_IPA, MSM_BUS_SLAVE_OCIMEM,  0, 0 },
};

static struct msm_bus_vectors compat_nominal_vectors[] = {
	{ MSM_BUS_MASTER_IPA, MSM_BUS_SLAVE_EBI_CH0, 100000000, 1300000000 },
	{ MSM_BUS_MASTER_IPA, MSM_BUS_SLAVE_OCIMEM,  100000000, 1300000000 },
};

static struct msm_bus_paths compat_usecases[] = {
	{ ARRAY_SIZE(compat_init_vectors),    compat_init_vectors    },
	{ ARRAY_SIZE(compat_nominal_vectors), compat_nominal_vectors },
};

static struct msm_bus_scale_pdata compat_default_pdata = {
	.usecase = compat_usecases,
	.num_usecases = ARRAY_SIZE(compat_usecases),
	.name = "ipa-compat",
};

/*
 * msm_bus_cl_get_pdata() — vendor IPA driver calls this expecting to read
 * "qcom,msm-bus,vectors-KBps" from DT. Our DT instead uses mainline
 * "interconnects = ..." syntax (per icc), so there is nothing to parse.
 * We return a static default pdata so the IPA probe path doesn't bail
 * out with EPROBE_DEFER. The vectors match vendor's hardcoded
 * ipa_nominal_perf_vectors_v2_0.
 */
struct msm_bus_scale_pdata *
msm_bus_cl_get_pdata(struct platform_device *pdev)
{
	(void)pdev;
	return &compat_default_pdata;
}
EXPORT_SYMBOL(msm_bus_cl_get_pdata);

void msm_bus_cl_clear_pdata(struct msm_bus_scale_pdata *pdata)
{
	(void)pdata;
}
EXPORT_SYMBOL(msm_bus_cl_clear_pdata);

u32 msm_bus_scale_register_client(struct msm_bus_scale_pdata *pdata)
{
	struct msm_bus_compat_handle *h;
	unsigned int slot, i;
	struct icc_path *p;
	int unique_count = 0;

	if (!pdata || !pdata->num_usecases || !pdata->usecase) {
		pr_warn("msm_bus_compat: NULL pdata in register_client\n");
		return 0;
	}

	mutex_lock(&msm_bus_compat_lock);

	if (!msm_bus_compat_dev) {
		pr_err("msm_bus_compat: register_client called before set_device\n");
		mutex_unlock(&msm_bus_compat_lock);
		return 0;
	}

	/* Find a free handle slot. */
	for (slot = 0; slot < MSM_BUS_COMPAT_MAX_HANDLES; slot++)
		if (!handles[slot].in_use)
			break;
	if (slot == MSM_BUS_COMPAT_MAX_HANDLES) {
		pr_err("msm_bus_compat: handle table full\n");
		mutex_unlock(&msm_bus_compat_lock);
		return 0;
	}
	h = &handles[slot];
	memset(h, 0, sizeof(*h));
	h->pdata = pdata;

	/* Walk usecase[0] (init vectors) as the canonical set of paths. */
	for (i = 0; i < (unsigned int)pdata->usecase[0].num_paths; i++) {
		struct msm_bus_vectors *v = &pdata->usecase[0].vectors[i];
		const char *name = map_src_dst_to_icc_name(v->src, v->dst);

		if (!name) {
			pr_warn("msm_bus_compat: unknown src=%d dst=%d, skipping\n",
				v->src, v->dst);
			continue;
		}
		/* Skip duplicates (e.g. v1.1 has both IPA and BAM_DMA → EBI). */
		if (find_path(h, v->src, v->dst))
			continue;

		if (h->num_paths >= MSM_BUS_COMPAT_MAX_PATHS) {
			pr_warn("msm_bus_compat: per-handle path table full\n");
			break;
		}

		p = devm_of_icc_get(msm_bus_compat_dev, name);
		if (IS_ERR(p)) {
			pr_warn("msm_bus_compat: icc_get('%s') failed: %ld — skipping\n",
				name, PTR_ERR(p));
			continue;
		}

		h->paths[h->num_paths].src = v->src;
		h->paths[h->num_paths].dst = v->dst;
		h->paths[h->num_paths].path = p;
		h->num_paths++;
		unique_count++;
	}

	if (h->num_paths == 0) {
		pr_err("msm_bus_compat: no icc paths acquired for pdata '%s'\n",
		       pdata->name ? pdata->name : "?");
		mutex_unlock(&msm_bus_compat_lock);
		return 0;
	}

	h->in_use = true;
	pr_info("msm_bus_compat: registered client '%s' handle=%u with %u path(s)\n",
		pdata->name ? pdata->name : "?", slot + 1, h->num_paths);
	mutex_unlock(&msm_bus_compat_lock);
	return slot + 1;  /* non-zero handle */
}
EXPORT_SYMBOL(msm_bus_scale_register_client);

void msm_bus_scale_unregister_client(u32 cl)
{
	struct msm_bus_compat_handle *h;

	if (cl == 0 || cl > MSM_BUS_COMPAT_MAX_HANDLES)
		return;

	mutex_lock(&msm_bus_compat_lock);
	h = &handles[cl - 1];
	if (h->in_use) {
		/* Drop votes; devm-acquired paths are released at device tear-down. */
		unsigned int i;

		for (i = 0; i < h->num_paths; i++)
			(void)icc_set_bw(h->paths[i].path, 0, 0);
		h->in_use = false;
		h->num_paths = 0;
		h->pdata = NULL;
		pr_info("msm_bus_compat: unregistered handle=%u\n", cl);
	}
	mutex_unlock(&msm_bus_compat_lock);
}
EXPORT_SYMBOL(msm_bus_scale_unregister_client);

int msm_bus_scale_client_update_request(u32 cl, unsigned int index)
{
	struct msm_bus_compat_handle *h;
	struct msm_bus_paths *uc;
	int ret = 0;
	unsigned int i;

	if (cl == 0 || cl > MSM_BUS_COMPAT_MAX_HANDLES)
		return -EINVAL;

	mutex_lock(&msm_bus_compat_lock);
	h = &handles[cl - 1];
	if (!h->in_use || !h->pdata) {
		mutex_unlock(&msm_bus_compat_lock);
		return -EINVAL;
	}
	if ((int)index >= h->pdata->num_usecases) {
		pr_warn("msm_bus_compat: usecase %u out of range (max %d)\n",
			index, h->pdata->num_usecases - 1);
		mutex_unlock(&msm_bus_compat_lock);
		return -EINVAL;
	}

	uc = &h->pdata->usecase[index];
	for (i = 0; i < (unsigned int)uc->num_paths; i++) {
		struct msm_bus_vectors *v = &uc->vectors[i];
		struct icc_path *p = find_path(h, v->src, v->dst);
		u32 ab_kbps, ib_kbps;
		int rc;

		if (!p)
			continue;  /* path wasn't acquired (unknown mapping) */
		ab_kbps = bps_to_kbps(v->ab);
		ib_kbps = bps_to_kbps(v->ib);
		rc = icc_set_bw(p, ab_kbps, ib_kbps);
		if (rc) {
			pr_warn("msm_bus_compat: icc_set_bw(%d→%d, ab=%u, ib=%u) ret=%d\n",
				v->src, v->dst, ab_kbps, ib_kbps, rc);
			if (!ret)
				ret = rc;
		}
	}

	mutex_unlock(&msm_bus_compat_lock);
	return ret;
}
EXPORT_SYMBOL(msm_bus_scale_client_update_request);
