/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <linux/ipa_wigig.h> for the IPA v2 port to Linux 6.x.
 *
 * WiGig (802.11ad / 60 GHz WiFi) offload is an IPA v3+ feature. IPA v2
 * hardware does not support it. The shared headers reference WiGig
 * types only in dispatcher function signatures and field types; the
 * v2-only build never invokes any of these paths because the
 * corresponding function pointers in the API control struct are NULL.
 *
 * What this stub provides:
 *   - struct ipa_wigig_conn_out_params       (forward decl, pointer only)
 *   - struct ipa_wigig_pipe_setup_info_smmu  (forward decl, pointer only)
 *   - typedef ipa_wigig_misc_int_cb          (callback type, never invoked)
 *
 * Phase 2 will replace this stub if/when WiGig support is needed (it
 * isn't for SDM660).
 */

#ifndef _LINUX_IPA_WIGIG_H
#define _LINUX_IPA_WIGIG_H

#include <linux/types.h>

/*
 * Forward declarations — used only as pointers in IPA v2 build path.
 */
struct ipa_wigig_conn_out_params;
struct ipa_wigig_pipe_setup_info_smmu;

/*
 * WiGig miscellaneous interrupt callback type.
 *
 * Used as a parameter type in:
 *   int ipa_wigig_internal_init(struct ipa_wdi_uc_ready_params *inout,
 *                               ipa_wigig_misc_int_cb int_notify,
 *                               phys_addr_t *uc_db_pa);
 *
 * In IPA v2 builds the dispatcher's function pointer for
 * ipa_wigig_internal_init() is NULL, so this callback is never invoked.
 * The exact signature is therefore irrelevant — we use a plain
 * `void (*)(void *priv)` to match the IPA notification-callback
 * convention of the rest of the API.
 */
typedef void (*ipa_wigig_misc_int_cb)(void *priv);

#endif /* _LINUX_IPA_WIGIG_H */
