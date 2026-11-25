# SPDX-License-Identifier: GPL-2.0-only
#
# Qualcomm IP Accelerator (IPA) v2 driver — monolithic build.
#
# All vendor-source files (parent IPA core, IPA RM, IPA v2 core, RMNET, QMI)
# are built into a SINGLE module: ipa-driver.ko.
#
# Why monolithic? Vendor downstream was monolithic (or built =y in
# defconfig), and the source has hundreds of cross-references between
# files in different sub-directories. Splitting them into separate
# modules would require sprinkling EXPORT_SYMBOL on >100 functions, which
# is fragile and adds a maintenance burden for a port whose primary goal
# is "modem dostane internet". Keeping it monolithic for Phase 1 is the
# pragmatic choice.
#
# Phase 2+ may break things back out into separate modules if needed for
# upstream submission.

obj-$(CONFIG_IPA) += ipa-driver.o

# ===========================================================================
# Parent dispatcher and IPA Resource Manager
# ===========================================================================

ipa-driver-y := \
	ipa_api.o \
	ipa_rm.o \
	ipa_rm_resource.o \
	ipa_rm_dependency_graph.o \
	ipa_rm_inactivity_timer.o \
	ipa_rm_peers_list.o

# ===========================================================================
# IPA v2 core (always-on for any IPA HW v2.x build)
# ===========================================================================

ipa-driver-y += \
	ipa_v2/ipa.o \
	ipa_v2/ipa_debugfs.o \
	ipa_v2/ipa_hdr.o \
	ipa_v2/ipa_flt.o \
	ipa_v2/ipa_rt.o \
	ipa_v2/ipa_dp.o \
	ipa_v2/msm_bus_compat.o \
	ipa_v2/ipa_client.o \
	ipa_v2/ipa_utils.o \
	ipa_v2/ipa_nat.o \
	ipa_v2/ipa_intf.o \
	ipa_v2/ipa_interrupts.o \
	ipa_v2/ipa_uc.o \
	ipa_v2/ipa_dma.o \
	ipa_v2/ipa_disabled_stubs.o

# ===========================================================================
# RMNET / QMI (link in unconditionally for the IPA-with-modem use case;
# it is the entire point of the driver). If you really want to omit RMNET,
# disable the whole IPA build instead.
# ===========================================================================

ipa-driver-$(CONFIG_RMNET_IPA) += \
	ipa_v2/rmnet_ipa.o \
	ipa_v2/rmnet_ipa_fd_ioctl.o \
	ipa_v2/ipa_qmi_service.o \
	ipa_v2/ipa_qmi_service_v01.o

# ===========================================================================
# Optional features. When enabled, real implementations get linked. When
# disabled, the disabled-feature stubs in ipa_compat.h provide
# `static inline { return 0; }` no-ops that resolve the symbols at
# compile time.
# ===========================================================================

ipa-driver-$(CONFIG_IPA_TETH_BRIDGE)    += ipa_v2/teth_bridge.o
ipa-driver-$(CONFIG_IPA_UC_OFFLOAD_WDI) += ipa_v2/ipa_uc_wdi.o ipa_v2/ipa_wdi3_i.o
ipa-driver-$(CONFIG_IPA_UC_OFFLOAD_NTN) += ipa_v2/ipa_uc_ntn.o
ipa-driver-$(CONFIG_IPA_MHI)            += ipa_v2/ipa_mhi.o ipa_v2/ipa_uc_mhi.o

# ===========================================================================
# Compiler flags
# ===========================================================================

# Absolute subtree paths (not $(src)) so this works both for the normal
# in-tree build and for `make M=drivers/platform/msm` partial builds.
IPA_DIR := $(srctree)/drivers/platform/msm/ipa
ccflags-y += -I$(IPA_DIR)
ccflags-y += -I$(IPA_DIR)/ipa_v2
ccflags-y += -include $(IPA_DIR)/ipa_v2/ipa_compat.h
ccflags-$(CONFIG_IPA_DEBUG) += -DIPA_DEBUG_BUILD
