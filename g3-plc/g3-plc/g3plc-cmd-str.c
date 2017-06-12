/* Copyright (c) 2017, David Hauweele <david@hauweele.net>
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

#include <stdio.h>

#include "g3plc-cmd.h"
#include "../dump.h"

const char * g3plc_type2str(enum g3plc_type type)
{
  switch(type) {
  case G3PLC_TYPE_SYSTEM:
    return "system";
  case G3PLC_TYPE_G3:
    return "G3";
  default:
    return "unknown type";
  }
}

const char * g3plc_ida2str(enum g3plc_ida ida)
{
  switch(ida) {
  case G3PLC_IDA_REQUEST:
    return "request";
  case G3PLC_IDA_CONFIRM:
    return "confirm";
  case G3PLC_IDA_INDICATION:
    return "indication";
  default:
    return "unknown IDA";
  }
}

const char * g3plc_idp2str(enum g3plc_idp idp)
{
  switch(idp) {
  case G3PLC_IDP_G3CTR:
    return "control";
  case G3PLC_IDP_UMAC:
    return "UMAC";
  case G3PLC_IDP_ADP:
    return "ADP";
  case G3PLC_IDP_EAP:
    return "EAP";
  default:
    return "unknown IDP";
  }
}

const char * g3plc_cmdID2str(enum g3plc_idp idp, enum g3plc_cmd_ID cmd)
{
  switch(idp) {
  case G3PLC_IDP_G3CTR:
    switch(cmd) {
    case G3PLC_CMD_G3_INIT:
      return "G3 channel initialization";
    case G3PLC_CMD_G3_DEINIT:
      return "G3 channel deletion";
    case G3PLC_CMD_G3_GETCONFIG:
      return "get attribute";
    case G3PLC_CMD_G3_SETCONFIG:
      return "set attribute";
    case G3PLC_CMD_G3_CLEARINFO:
      return "erase statistics and log";
    case G3PLC_CMD_G3_GETINFO:
      return "read statistics and log";
    case G3PLC_CMD_G3_EVENT:
      return "event";
    default:
      return "unknown command for G3 controller";
    }
  case G3PLC_IDP_UMAC:
    switch(cmd) {
    case G3PLC_CMD_MCPS_DATA:
      return "data";
    case G3PLC_CMD_MLME_RESET:
      return "reset";
    case G3PLC_CMD_MLME_GET:
      return "get attribute";
    case G3PLC_CMD_MLME_SET:
      return "set attribute";
    case G3PLC_CMD_MLME_SCAN:
      return "search PAN";
    case G3PLC_CMD_MLME_START:
      return "construct PAN";
    case G3PLC_CMD_MLME_BEACON_NOTIFY:
      return "beacon response";
    case G3PLC_CMD_MLME_COMM_STATUS:
      return "communication status";
    case G3PLC_CMD_MLME_FRAMECOUNT:
      return "frame counter";
    case G3PLC_CMD_MLME_TMR_RECEIVE:
      return "reception of TMR";
    case G3PLC_CMD_MLME_TMR_TRANSMIT:
      return "transmission of TMR";
    default:
      return "unknown command for UMAC layer";
    }
  case G3PLC_IDP_ADP:
    switch(cmd) {
    case G3PLC_CMD_ADPD_DATA:
      return "data";
    case G3PLC_CMD_ADPM_RESET:
      return "reset";
    case G3PLC_CMD_ADPM_DISCOVERY:
      return "search PAN";
    case G3PLC_CMD_ADPM_NETWORK_START:
      return "start PAN";
    case G3PLC_CMD_ADPM_NETWORK_JOIN:
      return "join PAN";
    case G3PLC_CMD_ADPM_NETWORK_LEAVE:
      return "leave PAN";
    case G3PLC_CMD_ADPM_GET:
      return "get attribute";
    case G3PLC_CMD_ADPM_SET:
      return "set attribute";
    case G3PLC_CMD_ADPM_ROUTE_DISCOVERY:
      return "route discovery";
    case G3PLC_CMD_ADPM_PATH_DISCOVERY:
      return "network path";
    case G3PLC_CMD_ADPM_LBP:
      return "LBP";
    case G3PLC_CMD_ADPM_NETWORK_STATUS:
      return "communication status";
    case G3PLC_CMD_ADPM_BUFFER:
      return "internal data TX buffer state change";
    case G3PLC_CMD_ADPM_KEY_STATE:
      return "GMK-related processing granted";
    case G3PLC_CMD_ADPM_ROUTE_ERROR:
      return "route error packet";
    case G3PLC_CMD_ADPM_EAP_KEY:
      return "key generated in EAP PSK";
    case G3PLC_CMD_ADPM_FRAME_COUNTER:
      return "frame counter";
    case G3PLC_CMD_ADPM_ROUTE_UPDATE:
      return "route table updated";
    default:
      return "unknown command for ADP layer";
    }
  case G3PLC_IDP_EAP:
    switch(cmd) {
    case G3PLC_CMD_EAPM_RESET:
      return "reset";
    case G3PLC_CMD_EAPM_START:
      return "start transmission/reception of LBP";
    case G3PLC_CMD_EAPM_GET:
      return "get attribute";
    case G3PLC_CMD_EAPM_SET:
      return "set attribute";
    case G3PLC_CMD_EAPM_NETWORK:
      return "kick/GMK-related processing";
    case G3PLC_CMD_EAPM_SETCLIENTINFO:
      return "operating client information";
    case G3PLC_CMD_EAPM_NETWORK_JOIN:
      return "peer participates in PAN";
    case G3PLC_CMD_EAPM_NETWORK_LEAVE:
      return "peer leaves PAN";
    case G3PLC_CMD_EAPM_NEWDEVICE:
      return "peer tries to join PAN";
    default:
      return "unknown command for EAP layer";
    }
  default:
    return "unknown IDP for command";
  }
}

const char * g3plc_status2str(enum g3plc_idp idp, enum g3plc_status st)
{
  switch(idp) {
  case G3PLC_IDP_G3CTR:
    switch(st) {
    case G3PLC_G3_SUCCESS:
      return "success";
    case G3PLC_G3_INVALID_REQUEST:
      return "invalid request";
    case G3PLC_G3_INSUFFICIENT_MEMSIZE:
      return "not enough memory";
    case G3PLC_G3_NO_RESPONSE:
      return "no response from G3 unit";
    case G3PLC_G3_UNINITIALIZED_STATE:
      return "G3 already in uninitialize state";
    case G3PLC_G3_INVALID_PARAMETER:
      return "invalid parameter";
    case G3PLC_G3_INVALID_STATE:
      return "G3 channel in invalid state";
    default:
      return "unknown status for G3 controller";
    }
  case G3PLC_IDP_UMAC:
    switch(st) {
    case G3PLC_MAC_SUCCESS:
      return "success";
    case G3PLC_MAC_COUNTER_ERROR:
      return "counter error";
    case G3PLC_MAC_IMPROPER_KEY_TYPE:
      return "improper key type";
    case G3PLC_MAC_IMPROPER_SECURITY_LEVEL:
      return "improper security level";
    case G3PLC_MAC_UNSUPPORTED_LEGACY:
      return "unsupported legacy";
    case G3PLC_MAC_UNSUPPORTED_SECURITY:
      return "unsupported security";
    case G3PLC_MAC_CHANNEL_ACCESS_FAILURE:
      return "channel access failure";
    case G3PLC_MAC_SECURITY_ERROR:
      return "security error";
    case G3PLC_MAC_FRAME_TOO_LONG:
      return "frame too long";
    case G3PLC_MAC_INVALID_HANDLE:
      return "invalid handle";
    case G3PLC_MAC_INVALID_PARAMETER:
      return "invalid parameter";
    case G3PLC_MAC_NO_ACK:
      return "no ACK";
    case G3PLC_MAC_NO_BEACON:
      return "no beacon";
    case G3PLC_MAC_NO_DATA:
      return "no data";
    case G3PLC_MAC_NO_SHORT_ADDRESS:
      return "no short address";
    case G3PLC_MAC_OUT_OF_CAP:
      return "no room for internal buffer";
    case G3PLC_MAC_PAN_ID_CONFLICT:
      return "PAN ID conflict";
    case G3PLC_MAC_UNAVAILABLE_KEY:
      return "unavailable key";
    case G3PLC_MAC_UNSUPPORTED_ATTRIBUTE:
      return "unsupported attribute";
    case G3PLC_MAC_INVALID_ADDRESS:
      return "invalid address";
    case G3PLC_MAC_INVALID_INDEX:
      return "invalid index";
    case G3PLC_MAC_LIMIT_REACHED:
      return "limit reached";
    case G3PLC_MAC_READ_ONLY:
      return "read only";
    case G3PLC_MAC_SCAN_IN_PROGRESS:
      return "scan in progress";
    case G3PLC_MAC_INVALID_STATE:
      return "invalid state";
    case G3PLC_MAC_NO_RESPONSE:
      return "no response";
    case G3PLC_MAC_LML_ABORTED:
      return "LML aborted";
    case G3PLC_MAC_LML_NO_ACK:
      return "LML no ACK";
    case G3PLC_MAC_LML_CSMA_FAILURE:
      return "LML CSMA failure";
    case G3PLC_MAC_LML_BUFFER_FULL:
      return "LML buffer full";
    case G3PLC_MAC_LML_INVALID_REQ:
      return "LML invalid request";
    case G3PLC_MAC_LML_ABORT_ERROR:
      return "LML abort error";
    case G3PLC_MAC_LML_NO_RESPONSE:
      return "LML no response";
    case G3PLC_MAC_LML_FAILURE:
      return "LML failure";
    case G3PLC_MAC_INSUFFICIENT_MEMSIZE:
      return "not enough memory";
    case G3PLC_MAC_IF_NO_RESPONSE:
      return "no response from LMAC layer";
    default:
      return "unknown status for MAC layer";
    }
  case G3PLC_IDP_ADP:
    switch(st) {
    case G3PLC_ADP_SUCCESS:
      return "success";
    case G3PLC_ADP_INVALID_PARAMETER:
      return "invalid parameter";
    case G3PLC_ADP_NO_BEACON:
      return "no beacon";
    case G3PLC_ADP_UNSUPPORTED_ATTRIBUTE:
      return "unsupported attribute";
    case G3PLC_ADP_INVALID_INDEX:
      return "invalid index";
    case G3PLC_ADP_READ_ONLY:
      return "read only";
    case G3PLC_ADP_INVALID_REQUEST:
      return "invalid request";
    case G3PLC_ADP_INVALID_IPV6_FRAME:
      return "invalid IPv6 frame";
    case G3PLC_ADP_ROUTE_ERROR:
      return "route error";
    case G3PLC_ADP_NOT_PERMITTED:
      return "not permitted";
    case G3PLC_ADP_TIMEOUT:
      return "timeout";
    case G3PLC_ADP_ALREADY_IN_PROGRESS:
      return "already in progress";
    case G3PLC_ADP_INCOMPLETE_PATH:
      return "incomplete path";
    case G3PLC_ADP_REQ_QUEUE_FULL:
      return "no room for internal buffer";
    case G3PLC_ADP_FAILED:
      return "failed";
    case G3PLC_ADP_CONFIG_ERROR:
      return "configuration error";
    case G3PLC_ADP_INSUFFICIENT_MEMSIZE:
      return "not enough memory";
    case G3PLC_ADP_IF_NO_RESPONSE:
      return "no response from ADP layer";
    default:
      return "unknown status for ADP layer";
    }
  case G3PLC_IDP_EAP:
    switch(st) {
    case G3PLC_EAP_SUCCESS:
      return "success";
    case G3PLC_EAP_INVALID_PARAMETER:
      return "invalid parameter";
    case G3PLC_EAP_UNSUPPORTED_ATTRIBUTE:
      return "unsupported attribute";
    case G3PLC_EAP_INVALID_INDEX:
      return "invalid index";
    case G3PLC_EAP_READ_ONLY:
      return "read only";
    case G3PLC_EAP_INVALID_REQUEST:
      return "invalid request";
    case G3PLC_EAP_FAILED:
      return "failed";
    case G3PLC_EAP_CONFIG_ERROR:
      return "configuration error";
    case G3PLC_EAP_EAP_PSK_IN_PROGRESS:
      return "EAP PSK in progress";
    case G3PLC_EAP_BLACKLISTED_DEVICE:
      return "blacklisted device";
    case G3PLC_EAP_EAP_PSK_FAILURE:
      return "EAP PSK failure";
    case G3PLC_EAP_REQ_QUEUE_FULL:
      return "no room for internal buffer";
    case G3PLC_EAP_TIMEOUT:
      return "timeout";
    case G3PLC_EAP_JOIN_DISCARD:
      return "join discard";
    case G3PLC_EAP_INSUFFICIENT_MEMSIZE:
      return "not enough memory";
    case G3PLC_EAP_IF_NO_RESPONSE:
      return "no response from EAP layer";
    default:
      return "unknown status for EAP layer";
    }
  default:
    return "unknown IDP for status";
  }
}

const char * g3plc_attr2str(enum g3plc_attr attr)
{
  switch(attr) {
  case G3PLC_ATTR_PANID:
    return "PAN ID";
  case G3PLC_ATTR_SHORTADDR:
    return "short address";
  case G3PLC_ATTR_RETRANS:
    return "max. retransmissions";
  default:
    return "unknown attribute";
  }
}

const char * g3plc_macstatus2str(enum g3plc_mac_status st)
{
  switch(st) {
  case R_G3MAC_STATUS_SUCCESS:
    return "success";
  case R_G3MAC_STATUS_COUNTER_ERROR:
    return "counter error";
  case R_G3MAC_STATUS_UNSUPPORTED_SECURITY:
    return "unsupported security";
  case R_G3MAC_STATUS_CHANNEL_ACCESS_FAILURE:
    return "channel access failure";
  case R_G3MAC_STATUS_SECURITY_ERROR:
    return "security error";
  case R_G3MAC_STATUS_FRAME_TOO_LONG:
    return "frame too long";
  case R_G3MAC_STATUS_INVALID_PARAMETER:
    return "invalid parameter";
  case R_G3MAC_STATUS_NO_ACK:
    return "no ACK";
  case R_G3MAC_STATUS_OUT_OF_CAP:
    return "no room for internal buffer";
  case R_G3MAC_STATUS_INVALID_STATE:
    return "invalid state";
  case R_G3MAC_STATUS_UNAVAILABLE_KEY:
    return "unavailable key";
  case R_G3MAC_STATUS_LML_NO_RESPONSE:
    return "no response from LMAC layer";
  case R_G3MAC_STATUS_LML_ABORTED:
    return "LML aborted";
  case R_G3MAC_STATUS_LML_ABORT_ERROR:
    return "LML abort error";
  default:
    return "unknown MAC status";
  }
}

const char * g3plc_bandplan2str(enum g3plc_bandplan bp)
{
  switch(bp) {
  case G3PLC_BP_CENELEC_A:
    return "CENELEC A";
  case G3PLC_BP_CENELEC_B:
    return "CENELEC B";
  case G3PLC_BP_ARIB:
    return "ARIB";
  case G3PLC_BP_FCC:
    return "FCC";
  default:
    return "unknown band plan";
  }
}

static void field(const char *name, const char *value_s, unsigned int value)
{
  printf(" %s: %s (0x%02x)\n", name, value_s, value);
}

static void hex_field(const char *name, unsigned int value)
{
  printf(" %s: 0x%02x\n", name, value);
}

static void print_status(const struct g3plc_cmd *cmd, unsigned int size)
{
  unsigned int status;

  /* we need at least one status byte */
  if(size < 1)
    return;
  status = cmd->data[0];

  /* we only find status in confirmations */
  if(cmd->ida == G3PLC_IDA_CONFIRM) {
    /* for some commands status is in second byte */
    if((cmd->idp == G3PLC_IDP_UMAC && cmd->cmd == G3PLC_CMD_MCPS_DATA) || \
       (cmd->idp == G3PLC_IDP_ADP  && cmd->cmd == G3PLC_CMD_ADPM_PATH_DISCOVERY)) {
      if(size < 2)
        return;
      status = cmd->data[1];
    }

    field("Status", g3plc_status2str(cmd->idp, status), status);
  }
}

static void print_g3event_indication(const struct g3plc_cmd *cmd, unsigned int size)
{
  unsigned int event_code, length, param, ida, idp, cmd_ID;
  const unsigned char *d = cmd->data;

  /* only applies for G3-EVENT.indication */
  if(cmd->idp != G3PLC_IDP_G3CTR || cmd->cmd != G3PLC_CMD_G3_EVENT)
    return;

  if(size < 5)
    return;

  event_code = *d; d++;
  length     = *(uint16_t *)d; d++; /* FIXME: endianness */
  param      = *(uint16_t *)d; d++; /* FIXME: endianness */

  cmd_ID = param & 0xff;
  idp    = (param >> 8)  & 0xf; /* was ADP in datasheet, probably a typo */
  ida    = (param >> 12) & 0xf;

  hex_field("EvCode", event_code);
  hex_field("Length", length);
  field("Ev IDA", g3plc_ida2str(ida), ida);
  field("Ev IDP", g3plc_idp2str(idp), idp);
  field("Ev CMD", g3plc_cmdID2str(idp, cmd_ID), cmd_ID);
}

void g3plc_print_cmd(const struct g3plc_cmd *cmd, unsigned int size)
{
  size -= sizeof(struct g3plc_cmd);

  printf("command {\n");

  /* common values */
  field("Type  ", g3plc_type2str(cmd->type), cmd->type);
  field("IDA   ", g3plc_ida2str(cmd->ida), cmd->ida);
  field("IDP   ", g3plc_idp2str(cmd->idp), cmd->idp);
  field("CMD   ", g3plc_cmdID2str(cmd->idp, cmd->cmd), cmd->cmd);
  print_status(cmd, size);
  print_g3event_indication(cmd, size);

  /* display raw data */
  if(size > 0) {
    printf(" Data (%d bytes):\n", size);
    hex_dump(cmd->data, size);
  }

  printf("}\n");
}
