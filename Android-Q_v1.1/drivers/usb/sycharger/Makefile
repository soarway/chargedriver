# SPDX-License-Identifier: GPL-2.0-only
#
# Makefile for USB Power Delivery drivers
#

# obj-$(CONFIG_USB_SYBC_BQ2589X)	+= bq2589x_dual.o
obj-$(CONFIG_USB_SY_MASTER)	+= symaster.o sy_dev_class.o

obj-$(CONFIG_USB_SY_BC7D)	+= bc7d.o

obj-$(CONFIG_USB_SY_UF7F)	+= ufcs7f.o

# -----------------------------------------------------
obj-$(CONFIG_USB_SY_PD7E)	+= pd7e.o
# obj-$(CONFIG_TCPC_RT1711H)	+= sp_tcpc_rt1711h.o

obj-$(CONFIG_RT_REGMAP)		+= sypd/sp_rt_regmap.o

obj-$(CONFIG_SWITCH)		+= sypd/sp_switch_class.o
obj-$(CONFIG_SWITCH_GPIO)	+= sypd/sp_switch_gpio.o
obj-$(CONFIG_SWITCH_HD3SS460)		+= sypd/sp_switch_hd3ss460.o

obj-$(CONFIG_DUAL_ROLE_USB_INTF)	+= sypd/sp_class_dual_role.o sypd/sp_tcpci_dual_role.o


obj-$(CONFIG_TCPC_CLASS)		+= sypd/sp_tcpci_core.o sypd/sp_tcpci_typec.o sypd/sp_tcpci_alert.o sypd/sp_tcpci_timer.o \
									sypd/sp_tcpm.o sypd/sp_tcpm_legacy.o sypd/sp_tcpm_block.o sypd/sp_pd_dbg_info.o

obj-$(CONFIG_USB_POWER_DELIVERY)	+= sypd/sp_tcpci_event.o \
					   sypd/sp_pd_core.o sypd/sp_pd_policy_engine.o sypd/sp_pd_process_evt.o sypd/sp_pd_dpm_core.o \
					   sypd/sp_pd_process_evt_snk.o sypd/sp_pd_process_evt_src.o sypd/sp_pd_process_evt_vdm.o \
					   sypd/sp_pd_process_evt_drs.o sypd/sp_pd_process_evt_prs.o sypd/sp_pd_process_evt_vcs.o \
					   sypd/sp_pd_policy_engine_src.o sypd/sp_pd_policy_engine_snk.o sypd/sp_pd_policy_engine_ufp.o \
					   sypd/sp_pd_policy_engine_vcs.o sypd/sp_pd_dpm_uvdm.o sypd/sp_pd_process_evt_dbg.o \
					   sypd/sp_pd_policy_engine_dbg.o sypd/sp_pd_policy_engine_dfp.o sypd/sp_pd_policy_engine_dr.o \
					   sypd/sp_pd_policy_engine_drs.o sypd/sp_pd_policy_engine_prs.o sypd/sp_pd_dpm_alt_mode_dp.o \
					   sypd/sp_pd_dpm_pdo_select.o










