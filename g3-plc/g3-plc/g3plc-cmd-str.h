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

#ifndef _G3PLC_CMD_STR_H_
#define _G3PLC_CMD_STR_H_

#include "g3plc-cmd.h"

const char * g3plc_type2str(enum g3plc_type type);
const char * g3plc_ida2str(enum g3plc_ida ida);
const char * g3plc_idp2str(enum g3plc_idp idp);
const char * g3plc_cmdID2str(enum g3plc_idp idp, enum g3plc_cmd_ID cmd);
const char * g3plc_status2str(enum g3plc_idp idp, enum g3plc_status st);
const char * g3plc_attr2str(enum g3plc_attr attr);
const char * g3plc_macstatus2str(enum g3plc_mac_status st);
const char * g3plc_bandplan2str(enum g3plc_bandplan bp);

/* Display a command on stdout.
   The command must already (or still) be in host order (see hton_g3plc_cmd()).
   The size must be the size of the command (in host order) with its data. */
void g3plc_print_cmd(const struct g3plc_cmd *cmd, unsigned int size);

#endif /* _G3PLC_CMD_STR_H_ */
