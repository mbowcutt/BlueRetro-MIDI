/*
 * Copyright (c) 2019-2022, Jacques Gagnon
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _BT_ATT_CFG_H_
#define _BT_ATT_CFG_H_

void bt_att_cfg_hdlr(struct bt_dev *device, struct bt_hci_pkt *bt_hci_acl_pkt, uint32_t len);

void bt_att_cfg_midi_tx(uint8_t * buffer, uint32_t len);
uint16_t bt_att_cfg_get_mtu_size(void);

#endif /* _BT_ATT_CFG_H_ */
