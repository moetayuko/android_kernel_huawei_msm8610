/* Copyright (c) 2009, Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/* recreat lcd dts in fc1 baseline */
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

	 
#include<linux/init.h>
#include<linux/module.h>
	 
#include <linux/hw_lcd_common.h>
#include <mdss_dsi.h>
#include <linux/of.h>


//two gpios use to identify lcd module for 8x12 platform
#define LCD_ID_0_GPIO    76
#define LCD_ID_1_GPIO    93


#define GET_GPIO_FAIL  -1
#define GET_LCD_ID_FAIL  -1 
#define LCD_ID_PULL_UP  1
#define LCD_ID_PULL_DOWN  0
#define MIPI_MAX_BUFFER 128
static char mipi_packet_struct[MIPI_MAX_BUFFER];

struct of_device_id huawei_mdss_dsi_panel_match[] = {
	{.compatible = "huawei,lcd_panel_id"},
	{}
};
extern int dsi_cmds_tx_v2(struct mdss_panel_data *pdata,
            struct dsi_buf *tp, struct dsi_cmd_desc *cmds,
            int cnt);

void mipi_lcd_register_write(struct mdss_panel_data *pdata,struct dsi_buf *tp,
                                 u32 reg,u32 value,u32 time)

{
	static boolean packet_ok = FALSE;
	static u32 param_num = 0;
	static u32 last_datatype = 0;
	static u32 last_time = 0;
	u32 datatype = 0;

	struct dsi_cmd_desc dsi_cmd;
	struct dcs_cmd_req cmdreq;

	if (( (MIPI_DCS_COMMAND == last_datatype) || (MIPI_GEN_COMMAND == last_datatype) )
		&&( TYPE_PARAMETER != value ))
	{
		packet_ok = TRUE;
	}
	else
	{
		packet_ok = FALSE;
	}

	if(packet_ok)
	{
		switch (param_num)
		{
			case 1:
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/
					datatype = DTYPE_DCS_WRITE;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/
					datatype = DTYPE_GEN_WRITE1;
				}

				break;
			case 2:
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/
					datatype = DTYPE_DCS_WRITE1;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/
					datatype = DTYPE_GEN_WRITE2;
				}

				break;
			default:
				if (MIPI_DCS_COMMAND == last_datatype)
				{
					/*DCS MODE*/
					datatype = DTYPE_DCS_LWRITE;
				}
				else if (MIPI_GEN_COMMAND == last_datatype)
				{
					/*GCS MODE*/
					datatype = DTYPE_GEN_LWRITE;
				}

				break;
		}

		dsi_cmd.dchdr.dtype = datatype;
		dsi_cmd.dchdr.last = 1;
		dsi_cmd.dchdr.vc = 0;
		dsi_cmd.dchdr.ack = 0;
		dsi_cmd.dchdr.wait = last_time;
		dsi_cmd.dchdr.dlen = param_num;
		dsi_cmd.payload = mipi_packet_struct;
		cmdreq.cmds = &dsi_cmd;
		cmdreq.cmds_cnt = 1;
		cmdreq.flags = CMD_REQ_COMMIT | CMD_CLK_CTRL;
		cmdreq.rlen = 0;
		cmdreq.cb = NULL;

        	dsi_cmds_tx_v2(pdata, tp, &dsi_cmd, 1);

		packet_ok = FALSE;
		param_num = 0;
		last_datatype = 0;

	}

     switch (value)
    {
        case MIPI_DCS_COMMAND:
        case MIPI_GEN_COMMAND:
            last_datatype = value;
            last_time = time;
            mipi_packet_struct[param_num] = reg;
            param_num ++;
            break;
        case TYPE_PARAMETER:
            mipi_packet_struct[param_num] = reg;
            param_num ++;
            break;
        case MIPI_TYPE_END:
            packet_ok = FALSE;
            param_num = 0;
            last_datatype = 0;
            break;
        default :
            break;

    }

}

void process_mipi_table(struct mdss_panel_data *pdata,struct dsi_buf *tp,
					const struct sequence *table, size_t count)
{
	unsigned int i = 0;
	u32 reg = 0;
	u32 value = 0;
	u32 time = 0;

	for (i = 0; i < count; i++)
	{
		reg = table[i].reg;
		value = table[i].value;
		time = table[i].time;

		mipi_lcd_register_write(pdata,tp,reg,value,0);
		if (time != 0)
		{
           		 LCD_MDELAY(time);
		}
	}

}

/****************************************************************
function: get lcd id by gpio

*data structure*
*	   ID1	  ID0   * 
 *	----------------- * 
 *	|   |   |   |   | * 
 *	|   |   |   |   | * 
 *	----------------- *
 For each Gpio :
		00 means low  ,
		01 means high ,
		10 means float,
		11 is not defined,

 lcd id(hex):
 0	:ID0 low,	ID1 low
 1	:ID0 high,	ID1 low
 2	:ID0 float,	ID1 low

 4	:ID0 low,	ID1 high
 5	:ID0 high,	ID1 high
 6	:ID0 float,	ID1 high

 8	:ID0 low,	ID1 float
 9	:ID0 high,	ID1 float
 A	:ID0 float,	ID1 float, used for emulator
 ***************************************************************/
/*
G730:
BOE:     ID[1:0]   {1:1} --->is is 5
tianma:  ID[1:0]   {0:0} --->id is 0
youda:   ID[1:0]   {1:0} --->id is 4
BYD:     ID[1:0]   {0:1} --->id is 1


BOE    IC     NT35517 
tianma IC     HX8389-B 
youda  IC     RM68190 
BYD    IC     NT35517 
*/
int hw_get_lcd_id(void)
{
	int ret = 0;
	int id0,id1;
	int gpio_id0,gpio_id1;
	int pullup_read,pulldown_read;
	static int lcd_id = GET_LCD_ID_FAIL;
	
	id0=0;
	id1=0;
	pullup_read = 0;
	pulldown_read = 0;
	gpio_id0 = LCD_ID_0_GPIO;
	gpio_id1 = LCD_ID_1_GPIO;

	
	if( (lcd_id >= 0x0) && (lcd_id <= 0xA) )//if lcd_id had read successfully,just return lcd_id.
		return lcd_id;

	if(gpio_id0 <= GET_GPIO_FAIL ||gpio_id1 <= GET_GPIO_FAIL)
		return GET_LCD_ID_FAIL;

    LCD_LOG_INFO("gpio_lcd_id0:%d gpio_lcd_id1:%d\n",gpio_id0,gpio_id1);

    ret = gpio_request(gpio_id0, "lcd_id0");
      if (ret) {
         LCD_LOG_ERR("lcd_id0 gpio[%d] request failed\n", gpio_id0);
         goto lcd_id0_req_fail;
          }

    ret = gpio_request(gpio_id1, "lcd_id1");
	if (ret) {
	     LCD_LOG_ERR("lcd_id1 gpio[%d] request failed\n", gpio_id1);
         goto lcd_id1_req_fail;
	}

	/*config id0 to pull down and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	  if (ret) {
	     LCD_LOG_ERR("config id0 to pull down failed\n");
	     goto get_lcd_id_fail;
	    }
	udelay(10);
	pulldown_read = gpio_get_value(gpio_id0);

	/*config id0 to pull up and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_UP,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id0 to pull up failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pullup_read = gpio_get_value(gpio_id0);
	if(pulldown_read != pullup_read)//float
	{
		id0 = BIT(1);
		gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
	else//connect 
	{
		id0 = pullup_read;//pullup_read==pulldown_read
		switch(id0)
		{
			case LCD_ID_PULL_DOWN:
			case LCD_ID_PULL_UP:
			default:
				gpio_tlmm_config(GPIO_CFG(gpio_id0,0,GPIO_CFG_INPUT,GPIO_CFG_NO_PULL,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
				break;
		}

	}

	/*config id1 to pull down and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id1 to pull down failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pulldown_read = gpio_get_value(gpio_id1);

	/*config id1 to pull up and read*/
	ret = gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_UP,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	if (ret) {
		LCD_LOG_ERR("config id1 to pull up failed\n");
		goto get_lcd_id_fail;
		}
	udelay(10);
	pullup_read = gpio_get_value(gpio_id1);
	if(pulldown_read != pullup_read)//float
	{
		id1 = BIT(1);
		gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_PULL_DOWN,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
	}
	else//connect
	{
		id1 = pullup_read;//pullup_read==pulldown_read
		switch(id1)
		{
			case LCD_ID_PULL_DOWN:
			case LCD_ID_PULL_UP:
			default:
				gpio_tlmm_config(GPIO_CFG(gpio_id1,0,GPIO_CFG_INPUT,GPIO_CFG_NO_PULL,GPIO_CFG_2MA),GPIO_CFG_ENABLE);
				break;
		}
	}

	gpio_free(gpio_id0);
	gpio_free(gpio_id1);

	lcd_id = (id1<<2) | id0;
	return lcd_id;
get_lcd_id_fail:
	gpio_free(gpio_id1);
lcd_id1_req_fail:
	gpio_free(gpio_id0);
lcd_id0_req_fail:
	return GET_LCD_ID_FAIL;
}

void hw_get_lcd_panel(void)
{
	char *psKey = NULL;
	int id = 0;
	int ret = 0;
	psKey = kmalloc(30, GFP_KERNEL);
    if (NULL == psKey)  
    {
		ret = false;
		return ;
    }
    memset(psKey, 0, 30);

	id = hw_get_lcd_id();
	sprintf(psKey, "huawei,lcd_panel_id%X", id);
	strcpy(huawei_mdss_dsi_panel_match->compatible,psKey);

   kfree(psKey);
	return;
	
};
