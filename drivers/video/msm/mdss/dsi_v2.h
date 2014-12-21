/* Copyright (c) 2012-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef DSI_V2_H
#define DSI_V2_H

#include <linux/list.h>
#include <mach/scm-io.h>

#include "mdss_dsi.h"
#include "mdss_panel.h"

#define DSI_BUF_SIZE	1024
#define DSI_MRPS	0x04  /* Maximum Return Packet Size */

#define DSI_LEN 8 /* 4 x 4 - 6 - 2, bytes dcs header+crc-align  */


struct dsi_panel_private {
	struct dsi_buf dsi_panel_tx_buf;
	struct dsi_buf dsi_panel_rx_buf;

	int rst_gpio;
	int disp_en_gpio;
	int video_mode_gpio;
	int te_gpio;
	char bl_ctrl;

	struct regulator *vddio_vreg;
	struct regulator *vdda_vreg;

	struct dsi_panel_cmds_list *on_cmds_list;
	struct dsi_panel_cmds_list *off_cmds_list;
	struct mdss_dsi_phy_ctrl phy_params;

	char *on_cmds;
	char *off_cmds;
/* define cabc cmds struct */
#ifdef CONFIG_FB_AUTO_CABC
    struct dsi_panel_cmds_list *dsi_panel_cabc_ui_cmds;
    struct dsi_panel_cmds_list *dsi_panel_cabc_video_cmds;
#endif
/* Delete what we never use code */
};
/* dcs read/write */
#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

#define DTYPE_TEAR_ON		0x35	/* set tear on */
#define DTYPE_MAX_PKTSIZE	0x37	/* set max packet size */
#define DTYPE_NULL_PKT		0x09	/* null packet, no data */
#define DTYPE_BLANK_PKT		0x19	/* blankiing packet, no data */

#define DTYPE_CM_ON		0x02	/* color mode off */
#define DTYPE_CM_OFF		0x12	/* color mode on */
#define DTYPE_PERIPHERAL_OFF	0x22
#define DTYPE_PERIPHERAL_ON	0x32

/*
 * dcs response
 */
#define DTYPE_ACK_ERR_RESP      0x02
#define DTYPE_EOT_RESP          0x08    /* end of tx */
#define DTYPE_GEN_READ1_RESP    0x11    /* 1 parameter, short */
#define DTYPE_GEN_READ2_RESP    0x12    /* 2 parameter, short */
#define DTYPE_GEN_LREAD_RESP    0x1a
#define DTYPE_DCS_LREAD_RESP    0x1c
#define DTYPE_DCS_READ1_RESP    0x21    /* 1 parameter, short */
#define DTYPE_DCS_READ2_RESP    0x22    /* 2 parameter, short */



struct dsi_panel_cmds_list {
	struct dsi_cmd_desc *buf;
	int size;
	char ctrl_state;
};

struct dsi_panel_common_pdata {
	struct mdss_panel_info panel_info;
	int (*on) (struct mdss_panel_data *pdata);
	int (*off) (struct mdss_panel_data *pdata);
	void (*reset)(struct mdss_panel_data *pdata, int enable);
	void (*bl_fnc) (struct mdss_panel_data *pdata, u32 bl_level);
	struct dsi_panel_cmds_list *dsi_panel_on_cmds;
	struct dsi_panel_cmds_list *dsi_panel_off_cmds;
/* define cabc cmds struct */
#ifdef CONFIG_FB_AUTO_CABC
    int (*config_cabc) (struct mdss_panel_data *pdata,struct msmfb_cabc_config cabc_cfg);

    struct dsi_panel_cmds_list *dsi_panel_cabc_ui_cmds;
    struct dsi_panel_cmds_list *dsi_panel_cabc_video_cmds;
#endif

/* Delete what we never use code */
};

struct dsi_interface {
	int (*on)(struct mdss_panel_data *pdata);
	int (*off)(struct mdss_panel_data *pdata);
	int (*cont_on)(struct mdss_panel_data *pdata);
	void (*op_mode_config)(int mode, struct mdss_panel_data *pdata);
	int (*tx)(struct mdss_panel_data *pdata,
		struct dsi_buf *tp, struct dsi_cmd_desc *cmds, int cnt);
	int (*rx)(struct mdss_panel_data *pdata,
		 struct dsi_buf *tp, struct dsi_buf *rp,
		struct dsi_cmd_desc *cmds, int len);
	int index;
	void *private;
};

int dsi_panel_device_register_v2(struct platform_device *pdev,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata);

void dsi_register_interface(struct dsi_interface *intf);

int dsi_cmds_rx_v2(struct mdss_panel_data *pdata,
			struct dsi_buf *tp, struct dsi_buf *rp,
			struct dsi_cmd_desc *cmds, int len);

int dsi_cmds_tx_v2(struct mdss_panel_data *pdata,
			struct dsi_buf *tp, struct dsi_cmd_desc *cmds,
			int cnt);

char *dsi_buf_init(struct dsi_buf *dp);

int dsi_buf_alloc(struct dsi_buf *dp, int size);

int dsi_cmd_dma_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm);

int dsi_short_read1_resp(struct dsi_buf *rp);

int dsi_short_read2_resp(struct dsi_buf *rp);

int dsi_long_read_resp(struct dsi_buf *rp);

void dsi_set_tx_power_mode(int mode);

void dsi_ctrl_config_deinit(struct platform_device *pdev,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int dsi_ctrl_config_init(struct platform_device *pdev,
				struct mdss_dsi_ctrl_pdata *ctrl_pdata);

int dsi_ctrl_gpio_request(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

void dsi_ctrl_gpio_free(struct mdss_dsi_ctrl_pdata *ctrl_pdata);

struct mdss_panel_cfg *mdp3_panel_intf_type(int intf_val);

int mdp3_panel_get_boot_cfg(void);

#endif /* DSI_V2_H */
