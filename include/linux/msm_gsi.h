/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <linux/msm_gsi.h> for the IPA v2 port to Linux 6.x.
 *
 * GSI (Generic Software Interface) is the BAM/SPS replacement that
 * Qualcomm introduced for IPA v3 and later. IPA v2 hardware (used on
 * SDM636/660 etc.) does NOT use GSI — it uses BAM2BAM via SPS instead.
 *
 * However, the public IPA header <linux/ipa.h> and the parent
 * dispatcher source ipa_api.c are shared between IPA v2 and v3+, so
 * they reference GSI types in function signatures and struct fields
 * that the IPA v2 driver never actually invokes at runtime.
 *
 * For Phase 1 of the port this stub provides JUST ENOUGH type
 * declarations for those shared sources to parse:
 *   - enum gsi_prefetch_mode      (used as field type only)
 *   - struct gsi_chan_info        (forward decl, used only as pointer)
 *   - struct gsi_chan_err_notify  (forward decl, used only as pointer)
 *   - struct gsi_evt_err_notify   (forward decl, used only as pointer)
 *   - struct gsi_mhi_channel_scratch (forward decl, used only as pointer)
 *   - union  gsi_channel_scratch  (full definition; used by VALUE in
 *                                  ipa_mhi_resume_channels_internal())
 *
 * The dispatch logic in ipa_api.c only forwards into v3-specific code
 * via function pointers that are NULL on a v2-only build, so these
 * never actually execute. The size of `union gsi_channel_scratch` is
 * therefore irrelevant for correctness — but it must be SOME size for
 * by-value passing to compile.
 *
 * Phase 2 (IPA v3 support, if ever) will replace this with the real
 * GSI driver headers from a future port of drivers/platform/msm/gsi/.
 */

#ifndef _LINUX_MSM_GSI_H
#define _LINUX_MSM_GSI_H

#include <linux/types.h>

/*
 * Prefetch mode enum. Only used as a field type in
 *   struct ipa_gsi_ep_config { ... enum gsi_prefetch_mode prefetch_mode; ... };
 * No specific value is referenced anywhere in IPA v2 source.
 */
enum gsi_prefetch_mode {
	GSI_USE_PREFETCH_BUFS = 0,
	GSI_ESCAPE_BUF_ONLY,
	GSI_SMART_PRE_FETCH,
	GSI_FREE_PRE_FETCH,
};

/*
 * Forward declarations — used only as pointer-to-struct in IPA v2
 * function signatures, never dereferenced.
 */
struct gsi_chan_info;
struct gsi_chan_err_notify;
struct gsi_evt_err_notify;
struct gsi_mhi_channel_scratch;

/*
 * Channel scratch union — passed by VALUE in
 *   int ipa_mhi_resume_channels_internal(...,
 *       union gsi_channel_scratch ch_scratch, u8 index);
 *
 * The compiler needs a complete type with a known size. We make it
 * 32 bytes (8 × u32) which is roughly what real GSI hardware has.
 * Field internals are never accessed by IPA v2 code.
 */
union gsi_channel_scratch {
	struct {
		u32 scratch[8];
	} data;
	u32 value[8];
};

#endif /* _LINUX_MSM_GSI_H */
