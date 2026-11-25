/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stub <linux/ipa_wdi3.h> for the IPA v2 port to Linux 6.x.
 *
 * WDI3 (WLAN Data Interface v3) is the IPA v3+ WLAN offload framework
 * used to offload Tx/Rx packet processing to the IPA microcontroller
 * for hardware-accelerated WiFi. IPA v2 hardware (SDM636/660) does NOT
 * use WDI3 — its WLAN offload (WDI2) is in ipa_uc_wdi.c which is OFF
 * in the MVP build (`CONFIG_IPA_UC_OFFLOAD_WDI=n`).
 *
 * However, the public IPA API headers reference WDI3 struct types in
 * function signatures shared between IPA v2 and v3+:
 *   - ipa_api.h         (dispatcher prototypes)
 *   - ipa_common_i.h    (shared internal API)
 *
 * Each of these uses the WDI3 struct types ONLY through pointers, so
 * forward declarations suffice — the compiler doesn't need to know
 * the layout of structs that the IPA v2 driver never dereferences.
 *
 * Phase 2 will replace this stub if/when WDI3 functionality is needed.
 * For "modem dostane internet" MVP it is irrelevant.
 */

#ifndef _LINUX_IPA_WDI3_H
#define _LINUX_IPA_WDI3_H

#include <linux/types.h>

/*
 * Forward declarations of WDI3 struct types referenced by name in
 * function signatures shared between v2 and v3+. Pointers only —
 * no by-value usage in the IPA v2 build path, so no need for layout.
 */
struct ipa_wdi_in_params;
struct ipa_wdi_out_params;
struct ipa_wdi_buffer_info;
struct ipa_wdi_bw_info;
struct ipa_wdi_conn_in_params;
struct ipa_wdi_conn_out_params;
struct ipa_wdi_db_params;
struct ipa_wdi_tx_info;
struct ipa_wdi_uc_ready_params;

/*
 * Constant used as array size in ipa_uc_offload_i.h's WDI3 event-ring
 * descriptor template struct field. The struct itself is dead code on
 * IPA v2, but the file is included transitively by ipa_i.h so the
 * constant must be visible. 8 × u32 = 32 bytes is roughly what the
 * downstream header used.
 */
#define IPA_HW_WDI3_MAX_ER_DESC_SIZE	8

#endif /* _LINUX_IPA_WDI3_H */
