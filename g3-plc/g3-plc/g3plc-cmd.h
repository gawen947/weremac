/* Copyright (c) 2017, Aurélien Van Laere <aurelien.vanlaere@gmail.com>
                       David Hauweele <david@hauweele.net>
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _G3PLC_CMD_H_
#define _G3PLC_CMD_H_

#include <stdint.h>

/* see G3-PLC Serial Command Spec. p29 */
struct g3plc_cmd {
  unsigned int reserved : 8; /* reserved byte */

  unsigned int type : 8; /* access target block */
  unsigned int idc  : 1; /* G3 channel index */
  unsigned int ida  : 3; /* type of SAP (req, confirm, indication) */
  unsigned int idp  : 4; /* target layer */
  unsigned int cmd  : 8; /* command ID */

  unsigned char data[];
};

/* Command type */
enum g3plc_type {
  G3PLC_TYPE_SYSTEM, /* system block */
  G3PLC_TYPE_G3,     /* G3 block */
};

/* G3 channel index */
enum g3plc_channel {
  G3PLC_CHAN0, /* channel 0 */
  G3PLC_CHAN1  /* channel 1 */
};

/* type of SAP */
enum g3plc_ida {
  G3PLC_IDA_REQUEST,    /* request */
  G3PLC_IDA_CONFIRM,    /* confirm */
  G3PLC_IDA_INDICATION, /* indication */
};

/* target layer */
enum g3plc_idp {
  G3PLC_IDP_G3CTR,      /* G3 controller */
  /* 0x1-0x2 reserved */
  G3PLC_IDP_UMAC = 0x3, /* UMAC layer */
  G3PLC_IDP_ADP,        /* ADP layer */
  G3PLC_IDP_EAP,        /* EAP layer */
};

/* command ID */
enum g3plc_cmd_ID {
  /* G3 controller commands */
  G3PLC_CMD_G3_INIT      = 0x00, /* G3 channel initialization */
  G3PLC_CMD_G3_DEINIT    = 0x01, /* G3 channel deletion */
  G3PLC_CMD_G3_GETCONFIG = 0x02, /* request G3 config. parameter */
  G3PLC_CMD_G3_SETCONFIG = 0x03, /* set G3 config. parameter */
  G3PLC_CMD_G3_CLEARINFO = 0x14, /* erase statistics and log */
  G3PLC_CMD_G3_GETINFO   = 0x15, /* read statistics and log */
  G3PLC_CMD_G3_EVENT     = 0x20, /* event notification */

  /* UMAC layer commands */
  G3PLC_CMD_MCPS_DATA          = 0x00, /* request/notify data */
  G3PLC_CMD_MLME_RESET         = 0x01, /* reset UMAC and lower layer */
  G3PLC_CMD_MLME_GET           = 0x02, /* obtain MAC PIB attribute */
  G3PLC_CMD_MLME_SET           = 0x03, /* set MAC PIB attribute */
  G3PLC_CMD_MLME_SCAN          = 0x04, /* search PAN */
  G3PLC_CMD_MLME_START         = 0x05, /* construct PAN */
  G3PLC_CMD_MLME_BEACON_NOTIFY = 0x06, /* notify beacon response */
  G3PLC_CMD_MLME_COMM_STATUS   = 0x07, /* notify communication status */
  G3PLC_CMD_MLME_FRAMECOUNT    = 0x08, /* notify frame counter */
  G3PLC_CMD_MLME_TMR_RECEIVE   = 0x09, /* notify reception of TMR */
  G3PLC_CMD_MLME_TMR_TRANSMIT  = 0x0a, /* notify transmission of TMR */

  /* ADP layer commands */
  G3PLC_CMD_ADPD_DATA            = 0x00, /* request/notify data */
  G3PLC_CMD_ADPM_RESET           = 0x01, /* reset ADP and lower layer */
  G3PLC_CMD_ADPM_DISCOVERY       = 0x02, /* search PAN */
  G3PLC_CMD_ADPM_NETWORK_START   = 0x03, /* start PAN */
  G3PLC_CMD_ADPM_NETWORK_JOIN    = 0x04, /* join PAN */
  G3PLC_CMD_ADPM_NETWORK_LEAVE   = 0x05, /* request/notify leaving PAN */
  G3PLC_CMD_ADPM_GET             = 0x06, /* obtain ADP IB attribute */
  G3PLC_CMD_ADPM_SET             = 0x07, /* set ADP IB attribute */
  G3PLC_CMD_ADPM_ROUTE_DISCOVERY = 0x08, /* search for a route */
  G3PLC_CMD_ADPM_PATH_DISCOVERY  = 0x09, /* search/notify network path */
  G3PLC_CMD_ADPM_LBP             = 0x0a, /* request/notify LBP */
  G3PLC_CMD_ADPM_NETWORK_STATUS  = 0x0b, /* notify reception of comm. status */
  G3PLC_CMD_ADPM_BUFFER          = 0x0c, /* internal data TX buffer state change */
  G3PLC_CMD_ADPM_KEY_STATE       = 0x0d, /* GMK-related processing granted */
  G3PLC_CMD_ADPM_ROUTE_ERROR     = 0x0e, /* reception of a route error packet */
  G3PLC_CMD_ADPM_EAP_KEY         = 0x0f, /* key generated in EAP PSK */
  G3PLC_CMD_ADPM_FRAME_COUNTER   = 0x10, /* notify reception of frame counter */
  G3PLC_CMD_ADPM_ROUTE_UPDATE    = 0x11, /* route table updated */

  /* EAP layer commands */
  G3PLC_CMD_EAPM_RESET         = 0x00, /* reset EAP layer */
  G3PLC_CMD_EAPM_START         = 0x01, /* start transmission/reception of LBP message */
  G3PLC_CMD_EAPM_GET           = 0x02, /* obtain EAP IB attribute */
  G3PLC_CMD_EAPM_SET           = 0x03, /* set EAP IB attribute */
  G3PLC_CMD_EAPM_NETWORK       = 0x04, /* transmission request of kick/GMK-related processing */
  G3PLC_CMD_EAPM_SETCLIENTINFO = 0x05, /* transmission request of operating client information */
  G3PLC_CMD_EAPM_NETWORK_JOIN  = 0x06, /* notify that peer participates in PAN */
  G3PLC_CMD_EAPM_NETWORK_LEAVE = 0x07, /* notify that peer leaves the PAN */
  G3PLC_CMD_EAPM_NEWDEVICE     = 0x08, /* notify that a new device is trying to join the PAN */
};

/* status code */
enum g3plc_status {
  /* G3 controller status */
  G3PLC_G3_SUCCESS              = 0x00,
  G3PLC_G3_INVALID_REQUEST      = 0x80, /* invalid parameter */
  G3PLC_G3_INSUFFICIENT_MEMSIZE = 0xa0, /* not enough memory to execute request */
  G3PLC_G3_NO_RESPONSE          = 0xa4, /* no response for request from G3 unit */
  G3PLC_G3_UNINITIALIZED_STATE  = 0xa5, /* G3 channel already in uninitialize state */
  G3PLC_G3_INVALID_PARAMETER    = 0xe8, /* parameter not supported or out of range */
  G3PLC_G3_INVALID_STATE        = 0xfd, /* G3 channel in invalid state */

  /* MAC layer status */
  G3PLC_MAC_SUCCESS                 = 0x00,
  G3PLC_MAC_COUNTER_ERROR           = 0xdb,
  G3PLC_MAC_IMPROPER_KEY_TYPE       = 0xdc,
  G3PLC_MAC_IMPROPER_SECURITY_LEVEL = 0xdd,
  G3PLC_MAC_UNSUPPORTED_LEGACY      = 0xde,
  G3PLC_MAC_UNSUPPORTED_SECURITY    = 0xdf,
  G3PLC_MAC_CHANNEL_ACCESS_FAILURE  = 0xe1,
  G3PLC_MAC_SECURITY_ERROR          = 0xe4,
  G3PLC_MAC_FRAME_TOO_LONG          = 0xe5,
  G3PLC_MAC_INVALID_HANDLE          = 0xe7,
  G3PLC_MAC_INVALID_PARAMETER       = 0xe8,
  G3PLC_MAC_NO_ACK                  = 0xe9,
  G3PLC_MAC_NO_BEACON               = 0xea,
  G3PLC_MAC_NO_DATA                 = 0xeb,
  G3PLC_MAC_NO_SHORT_ADDRESS        = 0xec,
  G3PLC_MAC_OUT_OF_CAP              = 0xed,
  G3PLC_MAC_PAN_ID_CONFLICT         = 0xee,
  G3PLC_MAC_UNAVAILABLE_KEY         = 0xf3,
  G3PLC_MAC_UNSUPPORTED_ATTRIBUTE   = 0xf4,
  G3PLC_MAC_INVALID_ADDRESS         = 0xf5,
  G3PLC_MAC_INVALID_INDEX           = 0xf9,
  G3PLC_MAC_LIMIT_REACHED           = 0xfa,
  G3PLC_MAC_READ_ONLY               = 0xfb,
  G3PLC_MAC_SCAN_IN_PROGRESS        = 0xfc,
  G3PLC_MAC_INVALID_STATE           = 0xfd,
  G3PLC_MAC_NO_RESPONSE             = 0xff,
  G3PLC_MAC_LML_ABORTED             = 0xc0,
  G3PLC_MAC_LML_NO_ACK              = 0xc1,
  G3PLC_MAC_LML_CSMA_FAILURE        = 0xc2,
  G3PLC_MAC_LML_BUFFER_FULL         = 0xc4,
  G3PLC_MAC_LML_INVALID_REQ         = 0xc5,
  G3PLC_MAC_LML_ABORT_ERROR         = 0xcd,
  G3PLC_MAC_LML_NO_RESPONSE         = 0xce,
  G3PLC_MAC_LML_FAILURE             = 0xcf,
  G3PLC_MAC_INSUFFICIENT_MEMSIZE    = 0xa0,
  G3PLC_MAC_IF_NO_RESPONSE          = 0xa1,

  /* ADP layer status */
  G3PLC_ADP_SUCCESS               = 0x00,
  G3PLC_ADP_INVALID_PARAMETER     = 0xe8,
  G3PLC_ADP_NO_BEACON             = 0xea,
  G3PLC_ADP_UNSUPPORTED_ATTRIBUTE = 0xf4,
  G3PLC_ADP_INVALID_INDEX         = 0xf9,
  G3PLC_ADP_READ_ONLY             = 0xfb,
  G3PLC_ADP_INVALID_REQUEST       = 0x80,
  G3PLC_ADP_INVALID_IPV6_FRAME    = 0x82,
  G3PLC_ADP_ROUTE_ERROR           = 0x83,
  G3PLC_ADP_NOT_PERMITTED         = 0x84,
  G3PLC_ADP_TIMEOUT               = 0x86,
  G3PLC_ADP_ALREADY_IN_PROGRESS   = 0x87,
  G3PLC_ADP_INCOMPLETE_PATH       = 0x88,
  G3PLC_ADP_REQ_QUEUE_FULL        = 0x92,
  G3PLC_ADP_FAILED                = 0x93,
  G3PLC_ADP_CONFIG_ERROR          = 0x95,
  G3PLC_ADP_INSUFFICIENT_MEMSIZE  = 0xa0,
  G3PLC_ADP_IF_NO_RESPONSE        = 0xa2,

  /* EAP layer status */
  G3PLC_EAP_SUCCESS               = 0x00,
  G3PLC_EAP_INVALID_PARAMETER     = 0xe8,
  G3PLC_EAP_UNSUPPORTED_ATTRIBUTE = 0xf4,
  G3PLC_EAP_INVALID_INDEX         = 0xf9,
  G3PLC_EAP_READ_ONLY             = 0xfb,
  G3PLC_EAP_INVALID_REQUEST       = 0x80,
  G3PLC_EAP_FAILED                = 0x93,
  G3PLC_EAP_CONFIG_ERROR          = 0x95,
  G3PLC_EAP_EAP_PSK_IN_PROGRESS   = 0x98,
  G3PLC_EAP_BLACKLISTED_DEVICE    = 0x99,
  G3PLC_EAP_EAP_PSK_FAILURE       = 0x9a,
  G3PLC_EAP_REQ_QUEUE_FULL        = 0x9b,
  G3PLC_EAP_TIMEOUT               = 0x9c,
  G3PLC_EAP_JOIN_DISCARD          = 0x9d,
  G3PLC_EAP_INSUFFICIENT_MEMSIZE  = 0xa0,
  G3PLC_EAP_IF_NO_RESPONSE        = 0xa3,
};

/* Band plan */
enum g3plc_bandplan {
  G3PLC_BP_CENELEC_A,
  G3PLC_BP_CENELEC_B,
  G3PLC_BP_ARIB,
  G3PLC_BP_FCC
};

/* IB attributes */
enum g3plc_attr {
  G3PLC_ATTR_PANID     = 0x0050, /* PAN ID */
  G3PLC_ATTR_SHORTADDR = 0x0053, /* short address */
  G3PLC_ATTR_RETRANS   = 0x0059, /* max retransmissions */
};

/* MAC status code */
enum g3plc_mac_status {
  R_G3MAC_STATUS_SUCCESS                = 0x00,
  R_G3MAC_STATUS_COUNTER_ERROR          = 0xdb,
  R_G3MAC_STATUS_UNSUPPORTED_SECURITY   = 0xdf,
  R_G3MAC_STATUS_CHANNEL_ACCESS_FAILURE = 0xe1,
  R_G3MAC_STATUS_SECURITY_ERROR         = 0xe4,
  R_G3MAC_STATUS_FRAME_TOO_LONG         = 0xe5,
  R_G3MAC_STATUS_INVALID_PARAMETER      = 0xe8,
  R_G3MAC_STATUS_NO_ACK                 = 0xe9,
  R_G3MAC_STATUS_OUT_OF_CAP             = 0xed,
  R_G3MAC_STATUS_INVALID_STATE          = 0xfd,
  R_G3MAC_STATUS_UNAVAILABLE_KEY        = 0xf3,
  R_G3MAC_STATUS_LML_NO_RESPONSE        = 0xce,
  R_G3MAC_STATUS_LML_ABORTED            = 0xc0,
  R_G3MAC_STATUS_LML_ABORT_ERROR        = 0xcd,
};

/* Convert a command structure into a literal.
   This is useful for defining common command as literals
   that we can use in switch cases such as the dissector. */
#define LITERAL_G3PLC_CMD(cmd)                  \
  (union {                                      \
    uint32_t         u32;                       \
    struct g3plc_cmd c;                         \
  }){ .c = cmd }.u32
#define INLINE_G3PLC_CMD(reserved, type, idc, ida, idp, cmd) \
  LITERAL_G3PLC_CMD( ((struct g3plc_cmd){ reserved, type, idc, ida, idp, cmd }) )

/* Common commands defined as literals for switch cases. */
#define G3PLC_G3_INIT_CONFIRM INLINE_G3PLC_CMD(0,                 \
                                               G3PLC_TYPE_G3,     \
                                               G3PLC_CHAN0,       \
                                               G3PLC_IDA_CONFIRM, \
                                               G3PLC_IDP_G3CTR,   \
                                               G3PLC_CMD_G3_INIT)
#define G3PLC_G3_SETCONFIG_CONFIRM INLINE_G3PLC_CMD(0,                      \
                                                    G3PLC_TYPE_G3,          \
                                                    G3PLC_CHAN0,            \
                                                    G3PLC_IDA_CONFIRM,      \
                                                    G3PLC_IDP_G3CTR,        \
                                                    G3PLC_CMD_G3_SETCONFIG)
#define G3PLC_MLME_START_CONFIRM INLINE_G3PLC_CMD(0,                      \
                                                  G3PLC_TYPE_G3,          \
                                                  G3PLC_CHAN0,            \
                                                  G3PLC_IDA_CONFIRM,      \
                                                  G3PLC_IDP_UMAC,         \
                                                  G3PLC_CMD_MLME_START)
#define G3PLC_MLME_SET_CONFIRM INLINE_G3PLC_CMD(0,                      \
                                                G3PLC_TYPE_G3,          \
                                                G3PLC_CHAN0,            \
                                                G3PLC_IDA_CONFIRM,      \
                                                G3PLC_IDP_UMAC,         \
                                                G3PLC_CMD_MLME_SET)
#define G3PLC_MLME_RESET_CONFIRM INLINE_G3PLC_CMD(0,                      \
                                                  G3PLC_TYPE_G3,          \
                                                  G3PLC_CHAN0,            \
                                                  G3PLC_IDA_CONFIRM,      \
                                                  G3PLC_IDP_UMAC,         \
                                                  G3PLC_CMD_MLME_RESET)
#define G3PLC_MCPS_DATA_CONFIRM INLINE_G3PLC_CMD(0,                      \
                                                 G3PLC_TYPE_G3,          \
                                                 G3PLC_CHAN0,            \
                                                 G3PLC_IDA_CONFIRM,      \
                                                 G3PLC_IDP_UMAC,         \
                                                 G3PLC_CMD_MCPS_DATA)
#define G3PLC_MCPS_DATA_INDICATION INLINE_G3PLC_CMD(0,                    \
                                                    G3PLC_TYPE_G3,        \
                                                    G3PLC_CHAN0,          \
                                                    G3PLC_IDA_INDICATION, \
                                                    G3PLC_IDP_UMAC,       \
                                                    G3PLC_CMD_MCPS_DATA)

/* G3 PLC command packets are represented with a bitfield structure.
   However the fields order is not defined in standard C, only its size.
   So we need two functions to convert the host command bitfield to/from
   network order.

   Since G3 PLC commands have variable length, this function updates
   the bit ordering directly inside the buffer. */
void hton_g3plc_cmd(struct g3plc_cmd *cmd);
void ntoh_g3plc_cmd(struct g3plc_cmd *cmd);

#endif /* _G3PLC_CMD_H_ */
