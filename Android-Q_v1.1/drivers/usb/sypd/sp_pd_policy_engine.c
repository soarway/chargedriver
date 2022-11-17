/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * Power Delivery Policy Engine Driver
 *
 * Author: TH <tsunghan_tsai@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/usb/sypd/sp_pd_core.h>
#include <linux/usb/sypd/sp_pd_dpm_core.h>
#include <linux/usb/sypd/sp_tcpci.h>
#include <linux/usb/sypd/sp_pd_process_evt.h>
#include <linux/usb/sypd/sp_pd_policy_engine.h>


/* ---- Policy Engine State ---- */

#if PE_STATE_FULL_NAME
static const char *const pe_state_name[] = {
/******************* Source *******************/
#ifdef CONFIG_USB_PD_PE_SOURCE
	"PE_SRC_STARTUP",
	"PE_SRC_DISCOVERY",
	"PE_SRC_SEND_CAPABILITIES",
	"PE_SRC_NEGOTIATE_CAPABILITIES",
	"PE_SRC_TRANSITION_SUPPLY",
	"PE_SRC_TRANSITION_SUPPLY2",
	"PE_SRC_READY",
	"PE_SRC_DISABLED",
	"PE_SRC_CAPABILITY_RESPONSE",
	"PE_SRC_HARD_RESET",
	"PE_SRC_HARD_RESET_RECEIVED",
	"PE_SRC_TRANSITION_TO_DEFAULT",
	"PE_SRC_GET_SINK_CAP",
	"PE_SRC_WAIT_NEW_CAPABILITIES",
	"PE_SRC_SEND_SOFT_RESET",
	"PE_SRC_SOFT_RESET",
	"PE_SRC_PING",
#ifdef CONFIG_USB_PD_SRC_STARTUP_DISCOVER_ID
#ifdef CONFIG_PD_SRC_RESET_CABLE
	"PE_SRC_CBL_SEND_SOFT_RESET",
#endif	/* CONFIG_PD_SRC_RESET_CABLE */
	"PE_SRC_VDM_IDENTITY_REQUEST",
	"PE_SRC_VDM_IDENTITY_ACKED",
	"PE_SRC_VDM_IDENTITY_NAKED",
#endif	/* CONFIG_USB_PD_SRC_STARTUP_DISCOVER_ID */
#endif	/* CONFIG_USB_PD_PE_SOURCE */
/******************* Sink *******************/
#ifdef CONFIG_USB_PD_PE_SINK
	"PE_SNK_STARTUP",
	"PE_SNK_DISCOVERY",
	"PE_SNK_WAIT_FOR_CAPABILITIES",
	"PE_SNK_EVALUATE_CAPABILITY",
	"PE_SNK_SELECT_CAPABILITY",
	"PE_SNK_TRANSITION_SINK",
	"PE_SNK_READY",
	"PE_SNK_HARD_RESET",
	"PE_SNK_TRANSITION_TO_DEFAULT",
	"PE_SNK_GIVE_SINK_CAP",
	"PE_SNK_GET_SOURCE_CAP",
	"PE_SNK_SEND_SOFT_RESET",
	"PE_SNK_SOFT_RESET",
#endif	/* CONFIG_USB_PD_PE_SINK */
/******************* DR_SWAP *******************/
#ifdef CONFIG_USB_PD_DR_SWAP
	"PE_DRS_DFP_UFP_EVALUATE_DR_SWAP",
	"PE_DRS_DFP_UFP_ACCEPT_DR_SWAP",
	"PE_DRS_DFP_UFP_CHANGE_TO_UFP",
	"PE_DRS_DFP_UFP_SEND_DR_SWAP",
	"PE_DRS_DFP_UFP_REJECT_DR_SWAP",
	"PE_DRS_UFP_DFP_EVALUATE_DR_SWAP",
	"PE_DRS_UFP_DFP_ACCEPT_DR_SWAP",
	"PE_DRS_UFP_DFP_CHANGE_TO_DFP",
	"PE_DRS_UFP_DFP_SEND_DR_SWAP",
	"PE_DRS_UFP_DFP_REJECT_DR_SWAP",
#endif	/* CONFIG_USB_PD_DR_SWAP */
/******************* PR_SWAP *******************/
#ifdef CONFIG_USB_PD_PR_SWAP
	"PE_PRS_SRC_SNK_EVALUATE_PR_SWAP",
	"PE_PRS_SRC_SNK_ACCEPT_PR_SWAP",
	"PE_PRS_SRC_SNK_TRANSITION_TO_OFF",
	"PE_PRS_SRC_SNK_ASSERT_RD",
	"PE_PRS_SRC_SNK_WAIT_SOURCE_ON",
	"PE_PRS_SRC_SNK_SEND_SWAP",
	"PE_PRS_SRC_SNK_REJECT_PR_SWAP",
	"PE_PRS_SNK_SRC_EVALUATE_PR_SWAP",
	"PE_PRS_SNK_SRC_ACCEPT_PR_SWAP",
	"PE_PRS_SNK_SRC_TRANSITION_TO_OFF",
	"PE_PRS_SNK_SRC_ASSERT_RP",
	"PE_PRS_SNK_SRC_SOURCE_ON",
	"PE_PRS_SNK_SRC_SEND_SWAP",
	"PE_PRS_SNK_SRC_REJECT_SWAP",
/* get same role cap */
	"PE_DR_SRC_GET_SOURCE_CAP",
	"PE_DR_SRC_GIVE_SINK_CAP",
	"PE_DR_SNK_GET_SINK_CAP",
	"PE_DR_SNK_GIVE_SOURCE_CAP",
#endif	/* CONFIG_USB_PD_PR_SWAP */
/******************* VCONN_SWAP *******************/
#ifdef CONFIG_USB_PD_VCONN_SWAP
	"PE_VCS_SEND_SWAP",
	"PE_VCS_EVALUATE_SWAP",
	"PE_VCS_ACCEPT_SWAP",
	"PE_VCS_REJECT_VCONN_SWAP",
	"PE_VCS_WAIT_FOR_VCONN",
	"PE_VCS_TURN_OFF_VCONN",
	"PE_VCS_TURN_ON_VCONN",
	"PE_VCS_SEND_PS_RDY",
#endif	/* CONFIG_USB_PD_VCONN_SWAP */
/******************* UFP_VDM *******************/
	"PE_UFP_VDM_GET_IDENTITY",
	"PE_UFP_VDM_SEND_IDENTITY",
	"PE_UFP_VDM_GET_IDENTITY_NAK",
	"PE_UFP_VDM_GET_SVIDS",
	"PE_UFP_VDM_SEND_SVIDS",
	"PE_UFP_VDM_GET_SVIDS_NAK",
	"PE_UFP_VDM_GET_MODES",
	"PE_UFP_VDM_SEND_MODES",
	"PE_UFP_VDM_GET_MODES_NAK",
	"PE_UFP_VDM_EVALUATE_MODE_ENTRY",
	"PE_UFP_VDM_MODE_ENTRY_ACK",
	"PE_UFP_VDM_MODE_ENTRY_NAK",
	"PE_UFP_VDM_MODE_EXIT",
	"PE_UFP_VDM_MODE_EXIT_ACK",
	"PE_UFP_VDM_MODE_EXIT_NAK",
	"PE_UFP_VDM_ATTENTION_REQUEST",
#ifdef CONFIG_USB_PD_ALT_MODE
	"PE_UFP_VDM_DP_STATUS_UPDATE",
	"PE_UFP_VDM_DP_CONFIGURE",
#endif/* CONFIG_USB_PD_ALT_MODE */
/******************* DFP_VDM *******************/
	"PE_DFP_UFP_VDM_IDENTITY_REQUEST",
	"PE_DFP_UFP_VDM_IDENTITY_ACKED",
	"PE_DFP_UFP_VDM_IDENTITY_NAKED",
	"PE_DFP_CBL_VDM_IDENTITY_REQUEST",
	"PE_DFP_CBL_VDM_IDENTITY_ACKED",
	"PE_DFP_CBL_VDM_IDENTITY_NAKED",
	"PE_DFP_VDM_SVIDS_REQUEST",
	"PE_DFP_VDM_SVIDS_ACKED",
	"PE_DFP_VDM_SVIDS_NAKED",
	"PE_DFP_VDM_MODES_REQUEST",
	"PE_DFP_VDM_MODES_ACKED",
	"PE_DFP_VDM_MODES_NAKED",
	"PE_DFP_VDM_MODE_ENTRY_REQUEST",
	"PE_DFP_VDM_MODE_ENTRY_ACKED",
	"PE_DFP_VDM_MODE_ENTRY_NAKED",
	"PE_DFP_VDM_MODE_EXIT_REQUEST",
	"PE_DFP_VDM_MODE_EXIT_ACKED",
	"PE_DFP_VDM_ATTENTION_REQUEST",
	"PE_DFP_VDM_UNKNOWN",
#ifdef CONFIG_PD_DFP_RESET_CABLE
	"PE_DFP_CBL_SEND_SOFT_RESET",
	"PE_DFP_CBL_SEND_CABLE_RESET",
#endif	/* CONFIG_PD_DFP_RESET_CABLE */
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	"PE_DFP_VDM_DP_STATUS_UPDATE_REQUEST",
	"PE_DFP_VDM_DP_STATUS_UPDATE_ACKED",
	"PE_DFP_VDM_DP_STATUS_UPDATE_NAKED",
	"PE_DFP_VDM_DP_CONFIGURATION_REQUEST",
	"PE_DFP_VDM_DP_CONFIGURATION_ACKED",
	"PE_DFP_VDM_DP_CONFIGURATION_NAKED",
#endif/* CONFIG_USB_PD_ALT_MODE_DFP */
/******************* UVDM & SVDM *******************/
#ifdef CONFIG_USB_PD_UVDM
	"PE_UFP_UVDM_RECV",
	"PE_DFP_UVDM_SEND",
	"PE_DFP_UVDM_ACKED",
	"PE_DFP_UVDM_NAKED",
#endif/* CONFIG_USB_PD_UVDM */
/******************* PD30 for PPS *******************/
#ifdef CONFIG_USB_PD_REV30
	"PE_SRC_SEND_SOURCE_ALERT",
	"PE_SNK_SOURCE_ALERT_RECEIVED",
	"PE_SNK_GET_SOURCE_STATUS",
	"PE_SRC_GIVE_SOURCE_STATUS",
#ifdef CONFIG_USB_PD_REV30_PPS_SINK
	"PE_SNK_GET_PPS_STATUS",
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
#ifdef CONFIG_USB_PD_REV30_PPS_SOURCE
	"PE_SRC_GIVE_PPS_STATUS",
#endif	/* CONFIG_USB_PD_REV30_PPS_SOURCE */
#endif /* CONFIG_USB_PD_REV30 */
/******************* Others *******************/
#ifdef CONFIG_USB_PD_CUSTOM_DBGACC
	"PE_DBG_READY",
#endif/* CONFIG_USB_PD_CUSTOM_DBGACC */
#ifdef CONFIG_USB_PD_RECV_HRESET_COUNTER
	"PE_OVER_RECV_HRESET_LIMIT",
#endif/* CONFIG_USB_PD_RECV_HRESET_COUNTER */
	"PE_ERROR_RECOVERY",
	"PE_BIST_TEST_DATA",
	"PE_BIST_CARRIER_MODE_2",
/* Wait tx finished */
	"PE_IDLE1",
	"PE_IDLE2",
/******************* Virtual *******************/
};
#else
static const char *const pe_state_name[] = {
/******************* Source *******************/
#ifdef CONFIG_USB_PD_PE_SOURCE
	"SRC_START",
	"SRC_DISC",
	"SRC_SEND_CAP",
	"SRC_NEG_CAP",
	"SRC_TRANS_SUPPLY",
	"SRC_TRANS_SUPPLY2",
	"SRC_READY",
	"SRC_DISABLED",
	"SRC_CAP_RESP",
	"SRC_HRESET",
	"SRC_HRESET_RECV",
	"SRC_TRANS_DFT",
	"SRC_GET_CAP",
	"SRC_WAIT_CAP",
	"SRC_SEND_SRESET",
	"SRC_SRESET",
	"SRC_PING",
#ifdef CONFIG_USB_PD_SRC_STARTUP_DISCOVER_ID
#ifdef CONFIG_PD_SRC_RESET_CABLE
	"SRC_CBL_SEND_SRESET",
#endif	/* CONFIG_PD_SRC_RESET_CABLE */
	"SRC_VDM_ID_REQ",
	"SRC_VDM_ID_ACK",
	"SRC_VDM_ID_NAK",
#endif	/* PD_CAP_PE_SRC_STARTUP_DISCOVER_ID */
#endif	/* CONFIG_USB_PD_PE_SOURCE */
/******************* Sink *******************/
#ifdef CONFIG_USB_PD_PE_SINK
	"SNK_START",
	"SNK_DISC",
	"SNK_WAIT_CAP",
	"SNK_EVA_CAP",
	"SNK_SEL_CAP",
	"SNK_TRANS_SINK",
	"SNK_READY",
	"SNK_HRESET",
	"SNK_TRANS_DFT",
	"SNK_GIVE_CAP",
	"SNK_GET_CAP",
	"SNK_SEND_SRESET",
	"SNK_SRESET",
#endif	/* CONFIG_USB_PD_PE_SINK */
/******************* DR_SWAP *******************/
#ifdef CONFIG_USB_PD_DR_SWAP
	"D_DFP_EVA",
	"D_DFP_ACCEPT",
	"D_DFP_CHANGE",
	"D_DFP_SEND",
	"D_DFP_REJECT",
	"D_UFP_EVA",
	"D_UFP_ACCEPT",
	"D_UFP_CHANGE",
	"D_UFP_SEND",
	"D_UFP_REJECT",
#endif	/* CONFIG_USB_PD_DR_SWAP */
/******************* PR_SWAP *******************/
#ifdef CONFIG_USB_PD_PR_SWAP
	"P_SRC_EVA",
	"P_SRC_ACCEPT",
	"P_SRC_TRANS_OFF",
	"P_SRC_ASSERT",
	"P_SRC_WAIT_ON",
	"P_SRC_SEND",
	"P_SRC_REJECT",
	"P_SNK_EVA",
	"P_SNK_ACCEPT",
	"P_SNK_TRANS_OFF",
	"P_SNK_ASSERT",
	"P_SNK_SOURCE_ON",
	"P_SNK_SEND",
	"P_SNK_REJECT",
/* get same role cap */
	"DR_SRC_GET_CAP",
	"DR_SRC_GIVE_CAP",
	"DR_SNK_GET_CAP",
	"DR_SNK_GIVE_CAP",
#endif	/* CONFIG_USB_PD_PR_SWAP */
/******************* VCONN_SWAP *******************/
#ifdef CONFIG_USB_PD_VCONN_SWAP
	"V_SEND",
	"V_EVA",
	"V_ACCEPT",
	"V_REJECT",
	"V_WAIT_VCONN",
	"V_TURN_OFF",
	"V_TURN_ON",
	"V_PS_RDY",
#endif	/* CONFIG_USB_PD_VCONN_SWAP */
/******************* UFP_VDM *******************/
	"U_GET_ID",
	"U_SEND_ID",
	"U_GET_ID_N",
	"U_GET_SVID",
	"U_SEND_SVID",
	"U_GET_SVID_N",
	"U_GET_MODE",
	"U_SEND_MODE",
	"U_GET_MODE_N",
	"U_EVA_MODE",
	"U_MODE_EN_A",
	"U_MODE_EN_N",
	"U_MODE_EX",
	"U_MODE_EX_A",
	"U_MODE_EX_N",
	"U_ATTENTION",
#ifdef CONFIG_USB_PD_ALT_MODE
	"U_D_STATUS",
	"U_D_CONFIG",
#endif/* CONFIG_USB_PD_ALT_MODE */
/******************* DFP_VDM *******************/
	"D_UID_REQ",
	"D_UID_A",
	"D_UID_N",
	"D_CID_REQ",
	"D_CID_ACK",
	"D_CID_NAK",
	"D_SVID_REQ",
	"D_SVID_ACK",
	"D_SVID_NAK",
	"D_MODE_REQ",
	"D_MODE_ACK",
	"D_MODE_NAK",
	"D_MODE_EN_REQ",
	"D_MODE_EN_ACK",
	"D_MODE_EN_NAK",
	"D_MODE_EX_REQ",
	"D_MODE_EX_ACK",
	"D_ATTENTION",
	"D_UNKNOWN",
#ifdef CONFIG_PD_DFP_RESET_CABLE
	"D_C_SRESET",
	"D_C_CRESET",
#endif	/* CONFIG_PD_DFP_RESET_CABLE */
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	"D_DP_STATUS_REQ",
	"D_DP_STATUS_ACK",
	"D_DP_STATUS_NAK",
	"D_DP_CONFIG_REQ",
	"D_DP_CONFIG_ACK",
	"D_DP_CONFIG_NAK",
#endif/* CONFIG_USB_PD_ALT_MODE_DFP */
/******************* UVDM & SVDM *******************/
#ifdef CONFIG_USB_PD_UVDM
	"U_UVDM_RECV",
	"D_UVDM_SEND",
	"D_UVDM_ACKED",
	"D_UVDM_NAKED",
#endif/* CONFIG_USB_PD_UVDM */
/******************* PD30 for PPS *******************/
#ifdef CONFIG_USB_PD_REV30
	"SRC_ALERT",
	"SNK_RECV_ALERT",
	"SNK_GET_STATUS",
	"SRC_GIVE_STATUS",
#ifdef CONFIG_USB_PD_REV30_PPS_SINK
	"SNK_GET_PPS",
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
#ifdef CONFIG_USB_PD_REV30_PPS_SOURCE
	"SRC_GIVE_PPS",
#endif	/* CONFIG_USB_PD_REV30_PPS_SOURCE */
#endif /* CONFIG_USB_PD_REV30 */
/******************* Others *******************/
#ifdef CONFIG_USB_PD_CUSTOM_DBGACC
	"DBG_READY",
#endif/* CONFIG_USB_PD_CUSTOM_DBGACC */
#ifdef CONFIG_USB_PD_RECV_HRESET_COUNTER
	"OVER_HRESET_LIMIT",
#endif/* CONFIG_USB_PD_RECV_HRESET_COUNTER */
	"ERR_RECOVERY",
	"BIST_TD",
	"BIST_C2",
/* Wait tx finished */
	"IDLE1",
	"IDLE2",
/******************* Virtual *******************/
};
#endif	/* PE_STATE_FULL_NAME */

struct pe_state_actions {
	void (*entry_action)(struct pd_port *pd_port, struct pd_event *pd_event);
	/* const void (*exit_action)
	 * (struct pd_port *pd_port, struct pd_event *pd_event);
	 */
};

#define PE_STATE_ACTIONS(state) { .entry_action = state##_entry, }


/*
 * Policy Engine General State Activity
 */

static void pe_idle_reset_data(struct pd_port *pd_port)
{
	pd_reset_pe_timer(pd_port);
	pd_reset_svid_data(pd_port);

	pd_port->pd_prev_connected = false;
	pd_port->state_machine = PE_STATE_MACHINE_IDLE;

	pd_enable_bist_test_mode(pd_port, false);

#ifndef CONFIG_USB_PD_TRANSMIT_BIST2
	pd_disable_bist_mode2(pd_port);
#endif	/* CONFIG_USB_PD_TRANSMIT_BIST2 */

	pd_unlock_msg_output(pd_port);
}

void pe_idle1_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pe_idle_reset_data(pd_port);

	pd_try_put_pe_idle_event(pd_port);
}

void pe_idle2_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_set_rx_enable(pd_port, PD_RX_CAP_PE_IDLE);
	pd_notify_pe_idle(pd_port);
}

void pe_error_recovery_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pe_idle_reset_data(pd_port);

	pd_notify_pe_error_recovery(pd_port);
	pd_try_put_pe_idle_event(pd_port);
	pd_free_pd_event(pd_port, pd_event);
}

#ifdef CONFIG_USB_PD_RECV_HRESET_COUNTER
void pe_over_recv_hreset_limit_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	PE_INFO("OverHResetLimit++\r\n");
	pe_idle_reset_data(pd_port);
	pd_notify_pe_over_recv_hreset(pd_port);
	PE_INFO("OverHResetLimit--\r\n");
}
#endif	/* CONFIG_USB_PD_RECV_HRESET_COUNTER */

void pe_bist_test_data_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_enable_bist_test_mode(pd_port, true);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_bist_test_data_exit(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_enable_bist_test_mode(pd_port, false);
}

void pe_bist_carrier_mode_2_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_send_bist_mode2(pd_port);
	pd_enable_timer(pd_port, PD_TIMER_BIST_CONT_MODE);
	pd_free_pd_event(pd_port, pd_event);
}

void pe_bist_carrier_mode_2_exit(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_disable_timer(pd_port, PD_TIMER_BIST_CONT_MODE);
	pd_disable_bist_mode2(pd_port);
}

/*
 * Policy Engine Share State Activity
 */

void pe_power_ready_entry(struct pd_port *pd_port, struct pd_event *pd_event)
{
	pd_port->during_swap = false;
	pd_port->explicit_contract = true;

	if (pd_port->data_role == PD_ROLE_UFP)
		pd_set_rx_enable(pd_port, PD_RX_CAP_PE_READY_UFP);
	else
		pd_set_rx_enable(pd_port, PD_RX_CAP_PE_READY_DFP);

#ifdef CONFIG_USB_PD_UFP_FLOW_DELAY
	if (pd_port->data_role == PD_ROLE_UFP &&
		(!pd_port->dpm_ufp_flow_delay_done)) {
		DPM_DBG("Delay UFP Flow\r\n");
		pd_restart_timer(pd_port, PD_TIMER_UFP_FLOW_DELAY);
		pd_free_pd_event(pd_port, pd_event);
		return;
	}
#endif	/* CONFIG_USB_PD_UFP_FLOW_DELAY */

	pd_dpm_notify_pe_ready(pd_port, pd_event);
	pd_free_pd_event(pd_port, pd_event);
}

static const struct pe_state_actions pe_state_actions[] = {
/******************* Source *******************/
#ifdef CONFIG_USB_PD_PE_SOURCE
	PE_STATE_ACTIONS(pe_src_startup),
	PE_STATE_ACTIONS(pe_src_discovery),
	PE_STATE_ACTIONS(pe_src_send_capabilities),
	PE_STATE_ACTIONS(pe_src_negotiate_capabilities),
	PE_STATE_ACTIONS(pe_src_transition_supply),
	PE_STATE_ACTIONS(pe_src_transition_supply2),
	PE_STATE_ACTIONS(pe_src_ready),
	PE_STATE_ACTIONS(pe_src_disabled),
	PE_STATE_ACTIONS(pe_src_capability_response),
	PE_STATE_ACTIONS(pe_src_hard_reset),
	PE_STATE_ACTIONS(pe_src_hard_reset_received),
	PE_STATE_ACTIONS(pe_src_transition_to_default),
	PE_STATE_ACTIONS(pe_src_get_sink_cap),
	PE_STATE_ACTIONS(pe_src_wait_new_capabilities),
	PE_STATE_ACTIONS(pe_src_send_soft_reset),
	PE_STATE_ACTIONS(pe_src_soft_reset),
	PE_STATE_ACTIONS(pe_src_ping),
#ifdef CONFIG_USB_PD_SRC_STARTUP_DISCOVER_ID
#ifdef CONFIG_PD_SRC_RESET_CABLE
	PE_STATE_ACTIONS(pe_src_cbl_send_soft_reset),
#endif	/* CONFIG_PD_SRC_RESET_CABLE */
	PE_STATE_ACTIONS(pe_src_vdm_identity_request),
	PE_STATE_ACTIONS(pe_src_vdm_identity_acked),
	PE_STATE_ACTIONS(pe_src_vdm_identity_naked),
#endif	/* PD_CAP_PE_SRC_STARTUP_DISCOVER_ID */
#endif	/* CONFIG_USB_PD_PE_SOURCE */
/******************* Sink *******************/
#ifdef CONFIG_USB_PD_PE_SINK
	PE_STATE_ACTIONS(pe_snk_startup),
	PE_STATE_ACTIONS(pe_snk_discovery),
	PE_STATE_ACTIONS(pe_snk_wait_for_capabilities),
	PE_STATE_ACTIONS(pe_snk_evaluate_capability),
	PE_STATE_ACTIONS(pe_snk_select_capability),
	PE_STATE_ACTIONS(pe_snk_transition_sink),
	PE_STATE_ACTIONS(pe_snk_ready),
	PE_STATE_ACTIONS(pe_snk_hard_reset),
	PE_STATE_ACTIONS(pe_snk_transition_to_default),
	PE_STATE_ACTIONS(pe_snk_give_sink_cap),
	PE_STATE_ACTIONS(pe_snk_get_source_cap),
	PE_STATE_ACTIONS(pe_snk_send_soft_reset),
	PE_STATE_ACTIONS(pe_snk_soft_reset),
#endif	/* CONFIG_USB_PD_PE_SINK */
/******************* DR_SWAP *******************/
#ifdef CONFIG_USB_PD_DR_SWAP
	PE_STATE_ACTIONS(pe_drs_dfp_ufp_evaluate_dr_swap),
	PE_STATE_ACTIONS(pe_drs_dfp_ufp_accept_dr_swap),
	PE_STATE_ACTIONS(pe_drs_dfp_ufp_change_to_ufp),
	PE_STATE_ACTIONS(pe_drs_dfp_ufp_send_dr_swap),
	PE_STATE_ACTIONS(pe_drs_dfp_ufp_reject_dr_swap),
	PE_STATE_ACTIONS(pe_drs_ufp_dfp_evaluate_dr_swap),
	PE_STATE_ACTIONS(pe_drs_ufp_dfp_accept_dr_swap),
	PE_STATE_ACTIONS(pe_drs_ufp_dfp_change_to_dfp),
	PE_STATE_ACTIONS(pe_drs_ufp_dfp_send_dr_swap),
	PE_STATE_ACTIONS(pe_drs_ufp_dfp_reject_dr_swap),
#endif	/* CONFIG_USB_PD_DR_SWAP */
/******************* PR_SWAP *******************/
#ifdef CONFIG_USB_PD_PR_SWAP
	PE_STATE_ACTIONS(pe_prs_src_snk_evaluate_pr_swap),
	PE_STATE_ACTIONS(pe_prs_src_snk_accept_pr_swap),
	PE_STATE_ACTIONS(pe_prs_src_snk_transition_to_off),
	PE_STATE_ACTIONS(pe_prs_src_snk_assert_rd),
	PE_STATE_ACTIONS(pe_prs_src_snk_wait_source_on),
	PE_STATE_ACTIONS(pe_prs_src_snk_send_swap),
	PE_STATE_ACTIONS(pe_prs_src_snk_reject_pr_swap),
	PE_STATE_ACTIONS(pe_prs_snk_src_evaluate_pr_swap),
	PE_STATE_ACTIONS(pe_prs_snk_src_accept_pr_swap),
	PE_STATE_ACTIONS(pe_prs_snk_src_transition_to_off),
	PE_STATE_ACTIONS(pe_prs_snk_src_assert_rp),
	PE_STATE_ACTIONS(pe_prs_snk_src_source_on),
	PE_STATE_ACTIONS(pe_prs_snk_src_send_swap),
	PE_STATE_ACTIONS(pe_prs_snk_src_reject_swap),
/* get same role cap */
	PE_STATE_ACTIONS(pe_dr_src_get_source_cap),
	PE_STATE_ACTIONS(pe_dr_src_give_sink_cap),
	PE_STATE_ACTIONS(pe_dr_snk_get_sink_cap),
	PE_STATE_ACTIONS(pe_dr_snk_give_source_cap),
#endif	/* CONFIG_USB_PD_PR_SWAP */
/******************* VCONN_SWAP *******************/
#ifdef CONFIG_USB_PD_VCONN_SWAP
	PE_STATE_ACTIONS(pe_vcs_send_swap),
	PE_STATE_ACTIONS(pe_vcs_evaluate_swap),
	PE_STATE_ACTIONS(pe_vcs_accept_swap),
	PE_STATE_ACTIONS(pe_vcs_reject_vconn_swap),
	PE_STATE_ACTIONS(pe_vcs_wait_for_vconn),
	PE_STATE_ACTIONS(pe_vcs_turn_off_vconn),
	PE_STATE_ACTIONS(pe_vcs_turn_on_vconn),
	PE_STATE_ACTIONS(pe_vcs_send_ps_rdy),
#endif	/* CONFIG_USB_PD_VCONN_SWAP */
/******************* UFP_VDM *******************/
	PE_STATE_ACTIONS(pe_ufp_vdm_get_identity),
	PE_STATE_ACTIONS(pe_ufp_vdm_send_identity),
	PE_STATE_ACTIONS(pe_ufp_vdm_get_identity_nak),
	PE_STATE_ACTIONS(pe_ufp_vdm_get_svids),
	PE_STATE_ACTIONS(pe_ufp_vdm_send_svids),
	PE_STATE_ACTIONS(pe_ufp_vdm_get_svids_nak),
	PE_STATE_ACTIONS(pe_ufp_vdm_get_modes),
	PE_STATE_ACTIONS(pe_ufp_vdm_send_modes),
	PE_STATE_ACTIONS(pe_ufp_vdm_get_modes_nak),
	PE_STATE_ACTIONS(pe_ufp_vdm_evaluate_mode_entry),
	PE_STATE_ACTIONS(pe_ufp_vdm_mode_entry_ack),
	PE_STATE_ACTIONS(pe_ufp_vdm_mode_entry_nak),
	PE_STATE_ACTIONS(pe_ufp_vdm_mode_exit),
	PE_STATE_ACTIONS(pe_ufp_vdm_mode_exit_ack),
	PE_STATE_ACTIONS(pe_ufp_vdm_mode_exit_nak),
	PE_STATE_ACTIONS(pe_ufp_vdm_attention_request),
#ifdef CONFIG_USB_PD_ALT_MODE
	PE_STATE_ACTIONS(pe_ufp_vdm_dp_status_update),
	PE_STATE_ACTIONS(pe_ufp_vdm_dp_configure),
#endif/* CONFIG_USB_PD_ALT_MODE */
/******************* DFP_VDM *******************/
	PE_STATE_ACTIONS(pe_dfp_ufp_vdm_identity_request),
	PE_STATE_ACTIONS(pe_dfp_ufp_vdm_identity_acked),
	PE_STATE_ACTIONS(pe_dfp_ufp_vdm_identity_naked),
	PE_STATE_ACTIONS(pe_dfp_cbl_vdm_identity_request),
	PE_STATE_ACTIONS(pe_dfp_cbl_vdm_identity_acked),
	PE_STATE_ACTIONS(pe_dfp_cbl_vdm_identity_naked),
	PE_STATE_ACTIONS(pe_dfp_vdm_svids_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_svids_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_svids_naked),
	PE_STATE_ACTIONS(pe_dfp_vdm_modes_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_modes_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_modes_naked),
	PE_STATE_ACTIONS(pe_dfp_vdm_mode_entry_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_mode_entry_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_mode_entry_naked),
	PE_STATE_ACTIONS(pe_dfp_vdm_mode_exit_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_mode_exit_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_attention_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_unknown),
#ifdef CONFIG_PD_DFP_RESET_CABLE
	PE_STATE_ACTIONS(pe_dfp_cbl_send_soft_reset),
	PE_STATE_ACTIONS(pe_dfp_cbl_send_cable_reset),
#endif	/* CONFIG_PD_DFP_RESET_CABLE */
#ifdef CONFIG_USB_PD_ALT_MODE_DFP
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_status_update_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_status_update_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_status_update_naked),
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_configuration_request),
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_configuration_acked),
	PE_STATE_ACTIONS(pe_dfp_vdm_dp_configuration_naked),
#endif/* CONFIG_USB_PD_ALT_MODE_DFP */
/******************* UVDM & SVDM *******************/
#ifdef CONFIG_USB_PD_UVDM
	PE_STATE_ACTIONS(pe_ufp_uvdm_recv),
	PE_STATE_ACTIONS(pe_dfp_uvdm_send),
	PE_STATE_ACTIONS(pe_dfp_uvdm_acked),
	PE_STATE_ACTIONS(pe_dfp_uvdm_naked),
#endif/* CONFIG_USB_PD_UVDM */
/******************* PD30 for PPS *******************/
#ifdef CONFIG_USB_PD_REV30
	PE_STATE_ACTIONS(pe_src_send_source_alert),
	PE_STATE_ACTIONS(pe_snk_source_alert_received),
	PE_STATE_ACTIONS(pe_snk_get_source_status),
	PE_STATE_ACTIONS(pe_src_give_source_status),
#ifdef CONFIG_USB_PD_REV30_PPS_SINK
	PE_STATE_ACTIONS(pe_snk_get_pps_status),
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
#ifdef CONFIG_USB_PD_REV30_PPS_SOURCE
	PE_STATE_ACTIONS(pe_src_give_pps_status),
#endif	/* CONFIG_USB_PD_REV30_PPS_SOURCE */
#endif /* CONFIG_USB_PD_REV30 */
/******************* Others *******************/
#ifdef CONFIG_USB_PD_CUSTOM_DBGACC
	PE_STATE_ACTIONS(pe_dbg_ready),
#endif/* CONFIG_USB_PD_CUSTOM_DBGACC */
#ifdef CONFIG_USB_PD_RECV_HRESET_COUNTER
	PE_STATE_ACTIONS(pe_over_recv_hreset_limit),
#endif/* CONFIG_USB_PD_RECV_HRESET_COUNTER */
	PE_STATE_ACTIONS(pe_error_recovery),
	PE_STATE_ACTIONS(pe_bist_test_data),
	PE_STATE_ACTIONS(pe_bist_carrier_mode_2),
/* Wait tx finished */
	PE_STATE_ACTIONS(pe_idle1),
	PE_STATE_ACTIONS(pe_idle2),
/******************* Virtual *******************/
};

/* pd_state_action_fcn_t pe_get_exit_action(uint8_t pe_state) */
void (*pe_get_exit_action(uint8_t pe_state))(struct pd_port *, struct pd_event *)
{
	void (*retval)(struct pd_port *, struct pd_event *) = NULL;

	switch (pe_state) {
/******************* Source *******************/
#ifdef CONFIG_USB_PD_PE_SOURCE
	case PE_SRC_TRANSITION_SUPPLY:
		retval = pe_src_transition_supply_exit;
		break;
	case PE_SRC_TRANSITION_TO_DEFAULT:
		retval = pe_src_transition_to_default_exit;
		break;
	case PE_SRC_GET_SINK_CAP:
		retval = pe_src_get_sink_cap_exit;
		break;
#endif	/* CONFIG_USB_PD_PE_SOURCE */
/******************* Sink *******************/
#ifdef CONFIG_USB_PD_PE_SINK
	case PE_SNK_WAIT_FOR_CAPABILITIES:
		retval = pe_snk_wait_for_capabilities_exit;
		break;
	case PE_SNK_SELECT_CAPABILITY:
		retval = pe_snk_select_capability_exit;
		break;
	case PE_SNK_TRANSITION_SINK:
		retval = pe_snk_transition_sink_exit;
		break;
	case PE_SNK_TRANSITION_TO_DEFAULT:
		retval = pe_snk_transition_to_default_exit;
		break;
#endif	/* CONFIG_USB_PD_PE_SINK */

/******************* PR_SWAP *******************/
#ifdef CONFIG_USB_PD_PR_SWAP
	case PE_PRS_SRC_SNK_WAIT_SOURCE_ON:
		retval = pe_prs_src_snk_wait_source_on_exit;
		break;
	case PE_PRS_SNK_SRC_TRANSITION_TO_OFF:
		retval = pe_prs_snk_src_transition_to_off_exit;
		break;
	case PE_PRS_SNK_SRC_SOURCE_ON:
		retval = pe_prs_snk_src_source_on_exit;
		break;
/* get same role cap */
	case PE_DR_SRC_GET_SOURCE_CAP:
		retval = pe_dr_src_get_source_cap_exit;
		break;
	case PE_DR_SNK_GET_SINK_CAP:
		retval = pe_dr_snk_get_sink_cap_exit;
		break;
#endif	/* CONFIG_USB_PD_PR_SWAP */
/******************* VCONN_SWAP *******************/
#ifdef CONFIG_USB_PD_VCONN_SWAP
	case PE_VCS_WAIT_FOR_VCONN:
		retval = pe_vcs_wait_for_vconn_exit;
		break;
#endif	/* CONFIG_USB_PD_VCONN_SWAP */

/******************* PD30 for PPS *******************/
#ifdef CONFIG_USB_PD_REV30
	case PE_SNK_GET_SOURCE_STATUS:
		retval = pe_snk_get_source_status_exit;
		break;
#ifdef CONFIG_USB_PD_REV30_PPS_SINK
	case PE_SNK_GET_PPS_STATUS:
		retval = pe_snk_get_pps_status_exit;
		break;
#endif	/* CONFIG_USB_PD_REV30_PPS_SINK */
#endif /* CONFIG_USB_PD_REV30 */
/******************* Others *******************/

	case PE_BIST_TEST_DATA:
		retval = pe_bist_test_data_exit;
		break;
	case PE_BIST_CARRIER_MODE_2:
		retval = pe_bist_carrier_mode_2_exit;
		break;
	default:
		break;
	}

	return retval;
}

static inline void print_state(struct pd_port *pd_port, bool vdm_evt, uint8_t state)
{
	/*
	 * Source (P, Provider), Sink (C, Consumer)
	 * DFP (D), UFP (U)
	 * Vconn Source (Y/N)
	 */

#if PE_DBG_ENABLE
	PE_DBG("%s -> %s (%c%c%c)\r\n",
		vdm_evt ? "VDM" : "PD", pe_state_name[state],
		pd_port->power_role ? 'P' : 'C',
		pd_port->data_role ? 'D' : 'U',
		pd_port->vconn_source ? 'Y' : 'N');
#else
	PE_STATE_INFO("%s-> %s\r\n",
		vdm_evt ? "VDM" : "PD", pe_state_name[state]);
#endif	/* PE_DBG_ENABLE */
}

static void pd_pe_state_change(struct pd_port *pd_port, struct pd_event *pd_event, bool vdm_evt)
{
	void (*prev_exit_action)(struct pd_port *, struct pd_event *);
	void (*next_entry_action)(struct pd_port *, struct pd_event *);

	uint8_t old_state = pd_port->pe_state_curr;
	uint8_t new_state = pd_port->pe_state_next;

	PD_BUG_ON(old_state >= PD_NR_PE_STATES);
	PD_BUG_ON(new_state >= PD_NR_PE_STATES);

	if ((new_state == PE_IDLE1) || (new_state == PE_IDLE2))
		prev_exit_action = NULL;
	else
		prev_exit_action = pe_get_exit_action(old_state);

	next_entry_action = pe_state_actions[new_state].entry_action;

#if PE_STATE_INFO_VDM_DIS
	if (!vdm_evt)
#endif	/* PE_STATE_INFO_VDM_DIS */
		print_state(pd_port, vdm_evt, new_state);

	if (pd_port->pd_wait_sender_response) {
		pd_port->pd_wait_sender_response = false;
		pd_disable_timer(pd_port, PD_TIMER_SENDER_RESPONSE);
	}

	if (prev_exit_action)
		prev_exit_action(pd_port, pd_event);

	if (next_entry_action)
		next_entry_action(pd_port, pd_event);

	if (vdm_evt)
		pd_port->pe_vdm_state = new_state;
	else
		pd_port->pe_pd_state = new_state;
}

static int pd_handle_event(struct pd_port *pd_port, struct pd_event *pd_event, bool vdm_evt)
{
	if (vdm_evt) {
		if (pd_port->reset_vdm_state &&
			(!pd_port->dpm_ack_immediately)) {
			pd_port->reset_vdm_state = false;
			pd_port->pe_vdm_state = pd_port->pe_pd_state;
		}

		pd_port->pe_state_curr = pd_port->pe_vdm_state;
	} else {
		pd_port->pe_state_curr = pd_port->pe_pd_state;
	}

	if (pd_process_event(pd_port, pd_event, vdm_evt))
		pd_pe_state_change(pd_port, pd_event, vdm_evt);
	else
		pd_free_pd_event(pd_port, pd_event);

	return 1;
}

static inline int pd_put_dpm_ack_immediately(struct pd_port *pd_port, bool vdm_evt)
{
	struct pd_event pd_event = {
		.event_type = PD_EVT_DPM_MSG,
		.msg = PD_DPM_ACK,
		.pd_msg = NULL,
	};

	pd_handle_event(pd_port, &pd_event, vdm_evt);

	PE_DBG("ACK_Immediately\r\n");
	pd_port->dpm_ack_immediately = false;
	return 1;
}

static inline bool pd_try_get_tcp_deferred_event(struct tcpc_device *tcpc_dev, struct pd_event *pd_event, bool *vdm_evt)
{
	struct pd_port *pd_port = &tcpc_dev->pd_port;

	if (!pd_check_pe_state_ready(pd_port))
		return false;

	if (!pd_get_deferred_tcp_event(tcpc_dev, &pd_port->tcp_event))
		return false;

	pd_port->tcp_event_drop_reset_once = true;
	pd_port->tcp_event_id_1st = pd_port->tcp_event.event_id;

	pd_event->event_type = PD_EVT_TCP_MSG;
	pd_event->msg = pd_port->tcp_event_id_1st;
	pd_event->msg_sec = PD_TCP_FROM_TCPM;
	pd_event->pd_msg = NULL;

	if (pd_event->msg >= TCP_DPM_EVT_VDM_COMMAND) {
		*vdm_evt = true;
		pd_port->reset_vdm_state = true;
	}

	return true;
}

static inline bool pd_try_get_vdm_event(struct tcpc_device *tcpc_dev, struct pd_event *pd_event)
{
	bool vdm_evt = false;
	struct pd_port *pd_port = &tcpc_dev->pd_port;

	switch (pd_port->pe_pd_state) {
	case PE_SNK_READY:
	case PE_SRC_READY:
	case PE_SRC_STARTUP:
	case PE_SRC_DISCOVERY:
		vdm_evt = pd_get_vdm_event(tcpc_dev, pd_event);
		break;
#ifdef CONFIG_PD_SRC_RESET_CABLE
	case PE_SRC_CBL_SEND_SOFT_RESET:
		vdm_evt = pd_get_vdm_event(tcpc_dev, pd_event);
		break;
#endif	/* CONFIG_PD_SRC_RESET_CABLE */

#ifdef CONFIG_USB_PD_CUSTOM_DBGACC
	case PE_DBG_READY:
		vdm_evt = pd_get_vdm_event(tcpc_dev, pd_event);
		break;
#endif	/* CONFIG_USB_PD_CUSTOM_DBGACC */
	}

	return vdm_evt;
}

static inline bool pd_try_get_next_event(struct tcpc_device *tcpc_dev, struct pd_event *pd_event, bool *vdm_evt)
{
	struct pd_port *pd_port = &tcpc_dev->pd_port;
	*vdm_evt = false;

	if (pd_get_event(tcpc_dev, pd_event))
		return true;

	if (pd_try_get_vdm_event(tcpc_dev, pd_event)) {
		*vdm_evt = true;
		return true;
	}

	pd_port->pe_state_curr = pd_port->pe_pd_state;
	if (pd_try_get_tcp_deferred_event(tcpc_dev, pd_event, vdm_evt))
		return true;

	return false;
}

int pd_policy_engine_run(struct tcpc_device *tcpc_dev)
{
	bool vdm_evt = false;
	struct pd_event pd_event;
	struct pd_port *pd_port = &tcpc_dev->pd_port;

	if (!pd_try_get_next_event(tcpc_dev, &pd_event, &vdm_evt))
		return false;

#ifdef CONFIG_TCPC_IDLE_MODE
	tcpci_idle_poll_ctrl(tcpc_dev, true, 1);
#endif

	mutex_lock(&pd_port->pd_lock);

	pd_handle_event(pd_port, &pd_event, vdm_evt);

	if (pd_port->dpm_ack_immediately)
		pd_put_dpm_ack_immediately(pd_port, vdm_evt);

	mutex_unlock(&pd_port->pd_lock);

#ifdef CONFIG_TCPC_IDLE_MODE
	tcpci_idle_poll_ctrl(tcpc_dev, false, 1);
#endif

	return 1;
}
