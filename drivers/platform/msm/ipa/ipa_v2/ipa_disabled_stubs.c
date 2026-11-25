// SPDX-License-Identifier: GPL-2.0-only
/*
 * Stub implementations for IPA optional features that are disabled in the
 * current build (CONFIG_IPA_UC_OFFLOAD_WDI=n, CONFIG_IPA_UC_OFFLOAD_NTN=n,
 * CONFIG_IPA_TETH_BRIDGE=n, CONFIG_IPA_MHI=n).
 *
 * IPA's parent dispatcher (ipa_api.c) maintains a table of function
 * pointers (ipa_api_ctrl) and pre-populates it with the per-HW-version
 * implementations during ipa_utils.c::ipa_init_api(). Many of those
 * implementations live in ipa_uc_wdi.c, ipa_uc_ntn.c, teth_bridge.c, and
 * ipa_uc_mhi.c, which are NOT compiled when the corresponding CONFIG_*
 * is unset. Without these the link fails with hundreds of "undefined
 * symbol" errors.
 *
 * Why we don't include "ipa_i.h" here:
 *   ipa_i.h and ipa_uc_offload_i.h have a circular include relationship
 *   (each #includes the other). This works in vendor source files like
 *   ipa.c only by accident — because ipa.c includes ipa_i.h before any
 *   other path can pre-activate the ipa_uc_offload_i.h include guard.
 *
 *   A small TU like this one trips that fragile ordering. To avoid the
 *   issue we forward-declare exactly the struct/enum/typedef tags we
 *   need for our function signatures, and skip the vendor headers
 *   entirely. C considers "struct foo *" and "struct foo *" the same
 *   pointer type whether `struct foo` is forward-declared or fully
 *   defined, so the linker still resolves these symbols correctly
 *   against the real declarations in ipa_i.h that other TUs include.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/compiler.h>

/*
 * <linux/ipa.h> is the UAPI-style header. It pulls in <linux/msm_ipa.h>
 * (which defines enum ipa_client_type, enum ipa_dp_evt_type, ...) and
 * declares typedefs ipa_notify_cb and ipa_uc_ready_cb that we need for
 * stub function signatures. Crucially, it does NOT include any of the
 * vendor private headers that have the circular include problem
 * (ipa_i.h <-> ipa_uc_offload_i.h).
 */
#include <linux/ipa.h>

/* ---------------------------------------------------------------------------
 * Forward declarations for opaque struct and union tags used in stub
 * function signatures. Their full definitions live in vendor private
 * headers (ipa_uc_offload_i.h, ipa_mhi.h, etc.) which we do not include
 * here.
 *
 * C considers `struct foo *` and `struct foo *` the same pointer type
 * whether `struct foo` is forward-declared or fully defined, so the
 * linker resolves these symbols correctly against declarations in
 * other TUs that pull in the full headers.
 * --------------------------------------------------------------------------- */

struct IpaHwStatsWDIInfoData_t;
struct IpaHwStatsNTNInfoData_t;
union  IpaHwMhiDlUlSyncCmdData_t;

struct ipa_wdi_in_params;
struct ipa_wdi_out_params;
struct ipa_wdi_uc_ready_params;
struct ipa_wdi_db_params;
struct ipa_wdi_buffer_info;
struct ipa_wdi_conn_in_params;
struct ipa_wdi_conn_out_params;

struct ipa_ntn_conn_in_params;
struct ipa_ntn_conn_out_params;

struct teth_bridge_init_params;
struct teth_bridge_connect_params;

struct ipa_mhi_msi_info;

/* MHI top-level driver (ipa_mhi.c) types — defined in ipa_common_i.h
 * and ipa_qmi_service_v01.h. */
struct ipa_mhi_init_engine;
struct ipa_mhi_connect_params_internal;
struct ipa_config_req_msg_v01;

/* ---------------------------------------------------------------------------
 * Explicit prototypes for every stub. This serves three purposes:
 *   1) Silences -Wmissing-prototypes (these are extern by default).
 *   2) Asserts our stub signature matches the vendor declaration in
 *      ipa_i.h, since both prototypes must be type-compatible to coexist
 *      after linking.
 *   3) Documents which symbols this file resolves for the linker.
 * --------------------------------------------------------------------------- */

#if !IS_ENABLED(CONFIG_IPA_UC_OFFLOAD_WDI)
int ipa2_get_wdi_stats(struct IpaHwStatsWDIInfoData_t *stats);
int ipa2_wdi_init(void);
int ipa2_connect_wdi_pipe(struct ipa_wdi_in_params *in,
			  struct ipa_wdi_out_params *out);
int ipa2_disconnect_wdi_pipe(u32 clnt_hdl);
int ipa2_enable_wdi_pipe(u32 clnt_hdl);
int ipa2_disable_wdi_pipe(u32 clnt_hdl);
int ipa2_resume_wdi_pipe(u32 clnt_hdl);
int ipa2_suspend_wdi_pipe(u32 clnt_hdl);
int ipa2_broadcast_wdi_quota_reach_ind(uint32_t fid, uint64_t num_bytes);
int ipa_write_qmapid_wdi_pipe(u32 clnt_hdl, u8 qmap_id);
int ipa2_uc_reg_rdyCB(struct ipa_wdi_uc_ready_params *param);
int ipa2_uc_dereg_rdyCB(void);
int ipa2_uc_wdi_get_dbpa(struct ipa_wdi_db_params *out);
int ipa2_create_wdi_mapping(u32 num_buffers,
			    struct ipa_wdi_buffer_info *info);
int ipa2_release_wdi_mapping(u32 num_buffers,
			     struct ipa_wdi_buffer_info *info);
int ipa2_conn_wdi3_pipes(struct ipa_wdi_conn_in_params *in,
			 struct ipa_wdi_conn_out_params *out,
			 ipa_wdi_meter_notifier_cb wdi_notify);
int ipa2_disconn_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx);
int ipa2_enable_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx);
int ipa2_disable_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx);
#endif

#if !IS_ENABLED(CONFIG_IPA_UC_OFFLOAD_NTN)
int ipa2_get_ntn_stats(struct IpaHwStatsNTNInfoData_t *stats);
int ipa2_register_ipa_ready_cb(void (*ipa_ready_cb)(void *), void *user_data);
int ipa2_ntn_uc_reg_rdyCB(void (*ipauc_ready_cb)(void *), void *priv);
void ipa2_ntn_uc_dereg_rdyCB(void);
int ipa_ntn_init(void);
int ipa2_setup_uc_ntn_pipes(struct ipa_ntn_conn_in_params *inp,
			    ipa_notify_cb notify, void *priv,
			    u8 hdr_len,
			    struct ipa_ntn_conn_out_params *outp);
int ipa2_tear_down_uc_offload_pipes(int ipa_ep_idx_ul, int ipa_ep_idx_dl,
				    struct ipa_ntn_conn_in_params *params);
#endif

#if !IS_ENABLED(CONFIG_IPA_TETH_BRIDGE)
int teth_bridge_driver_init(void);
int ipa2_teth_bridge_init(struct teth_bridge_init_params *params);
int ipa2_teth_bridge_disconnect(enum ipa_client_type client);
int ipa2_teth_bridge_connect(struct teth_bridge_connect_params *connect_params);
#endif

#if !IS_ENABLED(CONFIG_IPA_MHI)
int ipa2_uc_mhi_init(void (*ready_cb)(void),
		     void (*wakeup_request_cb)(void));
void ipa2_uc_mhi_cleanup(void);
int ipa_uc_mhi_init_engine(struct ipa_mhi_msi_info *msi, u32 mmio_addr,
			   u32 host_ctrl_addr, u32 host_data_addr,
			   u32 first_ch_idx, u32 first_evt_idx);
int ipa_uc_mhi_init_channel(int ipa_ep_idx, int channelHandle,
			    int contexArrayIndex, int channelDirection);
int ipa2_uc_mhi_reset_channel(int channelHandle);
int ipa2_uc_mhi_suspend_channel(int channelHandle);
int ipa_uc_mhi_resume_channel(int channelHandle, bool LPTransitionRejected);
int ipa2_uc_mhi_stop_event_update_channel(int channelHandle);
int ipa2_uc_mhi_send_dl_ul_sync_info(union IpaHwMhiDlUlSyncCmdData_t *cmd);
int ipa2_uc_mhi_print_stats(char *dbg_buff, int size);

/* Top-level MHI driver (ipa_mhi.c) — gated by the same CONFIG_IPA_MHI. */
int ipa2_mhi_init_engine(struct ipa_mhi_init_engine *params);
int ipa2_connect_mhi_pipe(struct ipa_mhi_connect_params_internal *in,
			  u32 *clnt_hdl);
int ipa2_disconnect_mhi_pipe(u32 clnt_hdl);
bool ipa2_mhi_sps_channel_empty(enum ipa_client_type client);
int ipa2_disable_sps_pipe(enum ipa_client_type client);
int ipa2_mhi_reset_channel_internal(enum ipa_client_type client);
int ipa2_mhi_start_channel_internal(enum ipa_client_type client);
#endif

/*
 * ipa_mhi_handle_ipa_config_req — declared in ipa_common_i.h:395 and
 * called from ipa_qmi_service.c:210, but vendor source ships no
 * implementation in our uploads. Provide an unconditional weak stub
 * that returns 0 (success no-op). With MHI disabled, the dispatch path
 * never has anything useful to do here anyway.
 */
int ipa_mhi_handle_ipa_config_req(struct ipa_config_req_msg_v01 *config_req);

/* ===========================================================================
 * WDI (WLAN Data Interface) stubs.
 * Real implementations live in ipa_uc_wdi.c.
 * Enabled by CONFIG_IPA_UC_OFFLOAD_WDI.
 * =========================================================================== */

#if !IS_ENABLED(CONFIG_IPA_UC_OFFLOAD_WDI)

int __weak ipa2_get_wdi_stats(struct IpaHwStatsWDIInfoData_t *stats)
{
	return -ENODEV;
}

int __weak ipa2_wdi_init(void)
{
	return 0;	/* "successfully no-op'd" */
}

int __weak ipa2_connect_wdi_pipe(struct ipa_wdi_in_params *in,
				 struct ipa_wdi_out_params *out)
{
	return -ENODEV;
}

int __weak ipa2_disconnect_wdi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_enable_wdi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_disable_wdi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_resume_wdi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_suspend_wdi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_broadcast_wdi_quota_reach_ind(uint32_t fid, uint64_t num_bytes)
{
	return -ENODEV;
}

int __weak ipa_write_qmapid_wdi_pipe(u32 clnt_hdl, u8 qmap_id)
{
	return -ENODEV;
}

int __weak ipa2_uc_reg_rdyCB(struct ipa_wdi_uc_ready_params *param)
{
	return -ENODEV;
}

int __weak ipa2_uc_dereg_rdyCB(void)
{
	return -ENODEV;
}

int __weak ipa2_uc_wdi_get_dbpa(struct ipa_wdi_db_params *out)
{
	return -ENODEV;
}

int __weak ipa2_create_wdi_mapping(u32 num_buffers,
				   struct ipa_wdi_buffer_info *info)
{
	return -ENODEV;
}

int __weak ipa2_release_wdi_mapping(u32 num_buffers,
				    struct ipa_wdi_buffer_info *info)
{
	return -ENODEV;
}

#endif /* !CONFIG_IPA_UC_OFFLOAD_WDI */

/* ===========================================================================
 * WDI3 stubs (real implementations in ipa_wdi3_i.c, gated by same CONFIG).
 * =========================================================================== */

#if !IS_ENABLED(CONFIG_IPA_UC_OFFLOAD_WDI)

int __weak ipa2_conn_wdi3_pipes(struct ipa_wdi_conn_in_params *in,
				struct ipa_wdi_conn_out_params *out,
				ipa_wdi_meter_notifier_cb wdi_notify)
{
	return -ENODEV;
}

int __weak ipa2_disconn_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx)
{
	return -ENODEV;
}

int __weak ipa2_enable_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx)
{
	return -ENODEV;
}

int __weak ipa2_disable_wdi3_pipes(int ipa_ep_idx_tx, int ipa_ep_idx_rx)
{
	return -ENODEV;
}

#endif /* !CONFIG_IPA_UC_OFFLOAD_WDI */

/* ===========================================================================
 * NTN (Ethernet offload) stubs.
 * Real implementations live in ipa_uc_ntn.c.
 *
 * NOTE: ipa2_register_ipa_ready_cb lives in ipa_uc_ntn.c (vendor design
 * mishap — it's actually a core function, not NTN-specific) and
 * ipa_utils.c::ipa_init_api() wires it into the dispatch table. We
 * provide a fallback stub here that returns -EEXIST, which signals the
 * caller "IPA is already ready, run your callback synchronously."
 * =========================================================================== */

#if !IS_ENABLED(CONFIG_IPA_UC_OFFLOAD_NTN)

int __weak ipa2_get_ntn_stats(struct IpaHwStatsNTNInfoData_t *stats)
{
	return -ENODEV;
}

int __weak ipa2_register_ipa_ready_cb(void (*ipa_ready_cb)(void *),
				      void *user_data)
{
	/* Pretend IPA is already ready so callers proceed synchronously.
	 * Returning -EEXIST is the "IPA already ready" signal that
	 * rmnet_ipa_probe() and similar callers handle correctly.
	 */
	return -EEXIST;
}

int __weak ipa2_ntn_uc_reg_rdyCB(void (*ipauc_ready_cb)(void *), void *priv)
{
	return -ENODEV;
}

void __weak ipa2_ntn_uc_dereg_rdyCB(void)
{
}

int __weak ipa_ntn_init(void)
{
	return 0;
}

int __weak ipa2_setup_uc_ntn_pipes(struct ipa_ntn_conn_in_params *inp,
				   ipa_notify_cb notify, void *priv,
				   u8 hdr_len,
				   struct ipa_ntn_conn_out_params *outp)
{
	return -ENODEV;
}

/* Vendor declaration in ipa_i.h:
 *   int ipa2_tear_down_uc_offload_pipes(int ipa_ep_idx_ul, int ipa_ep_idx_dl,
 *                                       struct ipa_ntn_conn_in_params *params);
 */
int __weak ipa2_tear_down_uc_offload_pipes(int ipa_ep_idx_ul,
					   int ipa_ep_idx_dl,
					   struct ipa_ntn_conn_in_params *params)
{
	return -ENODEV;
}

#endif /* !CONFIG_IPA_UC_OFFLOAD_NTN */

/* ===========================================================================
 * Tethering bridge stubs.
 * Real implementations live in teth_bridge.c.
 * =========================================================================== */

#if !IS_ENABLED(CONFIG_IPA_TETH_BRIDGE)

int __weak teth_bridge_driver_init(void)
{
	return 0;
}

int __weak ipa2_teth_bridge_init(struct teth_bridge_init_params *params)
{
	return -ENODEV;
}

int __weak ipa2_teth_bridge_disconnect(enum ipa_client_type client)
{
	return -ENODEV;
}

int __weak ipa2_teth_bridge_connect(
	struct teth_bridge_connect_params *connect_params)
{
	return -ENODEV;
}

#endif /* !CONFIG_IPA_TETH_BRIDGE */

/* ===========================================================================
 * MHI (Mobile Hi-Speed Interface) stubs.
 * Real implementations live in ipa_uc_mhi.c.
 * =========================================================================== */

#if !IS_ENABLED(CONFIG_IPA_MHI)

int __weak ipa2_uc_mhi_init(void (*ready_cb)(void),
			    void (*wakeup_request_cb)(void))
{
	return -ENODEV;
}

void __weak ipa2_uc_mhi_cleanup(void)
{
}

int __weak ipa_uc_mhi_init_engine(struct ipa_mhi_msi_info *msi, u32 mmio_addr,
				  u32 host_ctrl_addr, u32 host_data_addr,
				  u32 first_ch_idx, u32 first_evt_idx)
{
	return -ENODEV;
}

int __weak ipa_uc_mhi_init_channel(int ipa_ep_idx, int channelHandle,
				   int contexArrayIndex, int channelDirection)
{
	return -ENODEV;
}

int __weak ipa2_uc_mhi_reset_channel(int channelHandle)
{
	return -ENODEV;
}

int __weak ipa2_uc_mhi_suspend_channel(int channelHandle)
{
	return -ENODEV;
}

int __weak ipa_uc_mhi_resume_channel(int channelHandle, bool LPTransitionRejected)
{
	return -ENODEV;
}

int __weak ipa2_uc_mhi_stop_event_update_channel(int channelHandle)
{
	return -ENODEV;
}

int __weak ipa2_uc_mhi_send_dl_ul_sync_info(union IpaHwMhiDlUlSyncCmdData_t *cmd)
{
	return -ENODEV;
}

int __weak ipa2_uc_mhi_print_stats(char *dbg_buff, int size)
{
	return 0;	/* zero bytes printed */
}

/* ===========================================================================
 * Top-level MHI driver stubs (real implementations in ipa_mhi.c).
 * Same CONFIG_IPA_MHI gating as the uC variants above.
 * =========================================================================== */

int __weak ipa2_mhi_init_engine(struct ipa_mhi_init_engine *params)
{
	return -ENODEV;
}

int __weak ipa2_connect_mhi_pipe(struct ipa_mhi_connect_params_internal *in,
				 u32 *clnt_hdl)
{
	return -ENODEV;
}

int __weak ipa2_disconnect_mhi_pipe(u32 clnt_hdl)
{
	return -ENODEV;
}

bool __weak ipa2_mhi_sps_channel_empty(enum ipa_client_type client)
{
	return true;	/* nothing in flight */
}

int __weak ipa2_disable_sps_pipe(enum ipa_client_type client)
{
	return -ENODEV;
}

int __weak ipa2_mhi_reset_channel_internal(enum ipa_client_type client)
{
	return -ENODEV;
}

int __weak ipa2_mhi_start_channel_internal(enum ipa_client_type client)
{
	return -ENODEV;
}

#endif /* !CONFIG_IPA_MHI */

/* ===========================================================================
 * ipa_mhi_handle_ipa_config_req — declared but not implemented anywhere
 * in the vendor source we received. Provide an unconditional weak stub.
 *
 * If a future Phase enables MHI and ships the real implementation, the
 * linker will prefer the strong symbol over our weak stub.
 * =========================================================================== */

int __weak ipa_mhi_handle_ipa_config_req(struct ipa_config_req_msg_v01 *config_req)
{
	return 0;	/* no-op: nothing to configure when MHI is dormant */
}
