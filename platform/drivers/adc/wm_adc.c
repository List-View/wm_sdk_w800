
/***************************************************************************** 
* 
* File Name : wm_adc.c 
* 
* Description: adc Driver Module 
* 
* Copyright (c) 2014 Winner Microelectronics Co., Ltd. 
* All rights reserved. 
* 
* Author : dave
* 
* Date : 2014-8-15
*****************************************************************************/ 
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "wm_regs.h"
#include "wm_adc.h"
#include "wm_dma.h"
#include "wm_io.h"
#include "wm_irq.h"

#define  ATTRIBUTE_ISR __attribute__((isr))


//TODO
#define HR_SD_ADC_CONFIG_REG 0
static u32 adc_offset = 0;

volatile ST_ADC gst_adc;

ATTRIBUTE_ISR void ADC_IRQHandler(void)
{
	u16 adcvalue;
	int reg;

	reg = tls_reg_read32(HR_SD_ADC_INT_STATUS);
	if(reg & ADC_INT_MASK)      //ADC中断
	{
	    tls_adc_clear_irq(ADC_INT_TYPE_ADC);
	    adcvalue = tls_read_adc_result();
	    if(gst_adc.adc_cb)
			gst_adc.adc_cb(&adcvalue,1);
	}
	if(reg & CMP_INT_MASK)
	{
	    tls_adc_clear_irq(ADC_INT_TYPE_ADC_COMP);
	    if(gst_adc.adc_bigger_cb)
			gst_adc.adc_bigger_cb(NULL, 0);
	}
	
}

static void adc_dma_isr_callbk(void)
{
	if(gst_adc.adc_dma_cb)
		gst_adc.adc_dma_cb((u16 *)(ADC_DEST_BUFFER_DMA), gst_adc.valuelen);	
}


void tls_adc_init(u8 ifusedma,u8 dmachannel)
{
	tls_reg_write32(HR_SD_ADC_CTRL, ANALOG_SWITCH_TIME_VAL(0x50)|ANALOG_INIT_TIME_VAL(0x50)|ADC_IRQ_EN_VAL(0x1));
	tls_reg_write32(HR_SD_ADC_PGA_CTRL, CLK_CHOP_SEL_PGA_VAL(1)|GAIN_CTRL_PGA_VAL(2)|PGA_BYPASS_VAL(0)|PGA_CHOP_ENP_VAL(0)|PGA_EN_VAL(1));
	tls_irq_enable(ADC_IRQn);

//注册中断和channel有关，所以需要先请求
	if(ifusedma)
	{
		gst_adc.dmachannel = tls_dma_request(dmachannel, 0);	//请求dma，不要直接指定，因为请求的dma可能会被别的任务使用
		tls_dma_irq_register(gst_adc.dmachannel, (void(*)(void*))adc_dma_isr_callbk, NULL, TLS_DMA_IRQ_TRANSFER_DONE);
	}

	//printf("\ndma channel = %d\n",gst_adc.dmachannel);
}

void tls_adc_clear_irq(int inttype)
{
    int reg;
    reg = tls_reg_read32(HR_SD_ADC_INT_STATUS);
	if(ADC_INT_TYPE_ADC == inttype)
	{
	    reg |= ADC_INT_MASK;
	    tls_reg_write32(HR_SD_ADC_INT_STATUS, reg);
	}
	else if(ADC_INT_TYPE_ADC_COMP== inttype)
	{
	    reg |= CMP_INT_MASK;
	    tls_reg_write32(HR_SD_ADC_INT_STATUS, reg);
	}
	else if(ADC_INT_TYPE_DMA == inttype)
	{
	    tls_dma_irq_clr(gst_adc.dmachannel, TLS_DMA_IRQ_TRANSFER_DONE);
	}
}

void tls_adc_irq_register(int inttype, void (*callback)(u16 *buf, u16 len))
{
	if(ADC_INT_TYPE_ADC == inttype)
	{
		gst_adc.adc_cb = callback;
	}
	else if(ADC_INT_TYPE_DMA == inttype)
	{
		gst_adc.adc_dma_cb = callback;
	}
	else if(ADC_INT_TYPE_ADC_COMP == inttype)
	{
	    gst_adc.adc_bigger_cb = callback;
	}
}

u32 tls_read_adc_result(void)
{
	u32 value;
	u32 ret;
	
	value = tls_reg_read32(HR_SD_ADC_RESULT_REG);
	ret = ADC_RESULT_VAL(value);
	
	return ret;
}

void tls_adc_start_with_cpu(int Channel)
{
	u32 value;
      
	/* Stop adc first */
	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value |= CONFIG_PD_ADC_VAL(1);
	value &= ~(CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));
	value &= ~(CONFIG_ADC_CHL_SEL_MASK);
	value |= CONFIG_ADC_CHL_SEL(Channel);

	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);
	
	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value &= ~(CONFIG_PD_ADC_VAL(1));
	value |= (CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);

}


void tls_adc_start_with_dma(int Channel, int Length)
{
	u32 value;
	int len;

	if(Channel < 0 || Channel > 11)
		return;
        
	if(Length > ADC_DEST_BUFFER_SIZE)
		len = ADC_DEST_BUFFER_SIZE;
	else
		len = Length;

	gst_adc.valuelen = len;

	Channel &= 0xF;

	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value |= CONFIG_PD_ADC_VAL(1);
	value &= ~(CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));	
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);	
	
	/* Stop dma if necessary */
	while(DMA_CHNLCTRL_REG(gst_adc.dmachannel) & 1)
	{
		DMA_CHNLCTRL_REG(gst_adc.dmachannel) = 2;
	}

	DMA_SRCADDR_REG(gst_adc.dmachannel) = HR_SD_ADC_RESULT_REG;
	DMA_DESTADDR_REG(gst_adc.dmachannel) = ADC_DEST_BUFFER_DMA;
	/* Hard, Normal, adc_req */
	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);		
	if (Channel == 8){
		DMA_MODE_REG(gst_adc.dmachannel) = (0x01 | (0+6)<<2);
		value |= (0x1 << 11); 		
	}
	else if (Channel == 9){
		DMA_MODE_REG(gst_adc.dmachannel) = (0x01 | (2+6)<<2);
		value |= (0x1 << 13); 		
	}
	else if (Channel == 10){
		DMA_MODE_REG(gst_adc.dmachannel) = (0x01 | (4+6)<<2);
		value |= (0x1 << 15); 		
	}
	else if (Channel == 11){
		DMA_MODE_REG(gst_adc.dmachannel) = (0x01 | (6+6)<<2);
		value |= (0x1 << 17); 		
	}
	else{
		DMA_MODE_REG(gst_adc.dmachannel) = (0x01 | (Channel+6)<<2);
		value |= (0x1 << (11 + Channel)); 		
	}	
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);
	/* Dest_add_inc, halfword,  */
	DMA_CTRL_REG(gst_adc.dmachannel) = (1<<3)|(1<<5)|((len*2)<<8);
	DMA_INTMASK_REG &= ~(0x01 << (gst_adc.dmachannel *2 + 1));
	DMA_CHNLCTRL_REG(gst_adc.dmachannel) = 1;		/* Enable dma */

	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value |= CONFIG_ADC_CHL_SEL(Channel);
	value &= ~(CONFIG_PD_ADC_VAL(1));
	value |= (CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));	
//	printf("config value==%x\n", value);
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);		/*start adc*/
}

void tls_adc_stop(int ifusedma)
{
	u32 value;

	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value |= CONFIG_PD_ADC_VAL(1);
	value &= ~(CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));	
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);

	if(ifusedma)
		tls_dma_free(gst_adc.dmachannel);
}

void tls_adc_config_cmp_reg(int cmp_data, int cmp_pol)
{
    u32 value;

    tls_reg_write32(HR_SD_ADC_CMP_VALUE, CONFIG_ADC_INPUT_CMP_VAL(cmp_data));

    value = tls_reg_read32(HR_SD_ADC_CTRL);
	if(cmp_pol)
	{
		value |= CMP_POLAR_MASK;
	}
	else
	{
		value &= ~CMP_POLAR_MASK;
	}
    tls_reg_write32(HR_SD_ADC_CTRL, value);
}

void tls_adc_cmp_start(int Channel, int cmp_data, int cmp_pol)
{
	u32 value;
	
	/* Stop adc first */
	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value |= CONFIG_PD_ADC_VAL(1);
	value &= ~(CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));		
	value |= CONFIG_ADC_CHL_SEL(Channel);
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);

	tls_adc_config_cmp_reg(cmp_data, cmp_pol);
	
	value = tls_reg_read32(HR_SD_ADC_ANA_CTRL);
	value &= ~(CONFIG_PD_ADC_VAL(1));
	value |= (CONFIG_RSTN_ADC_VAL(1)|CONFIG_EN_LDO_ADC_VAL(1));	
	tls_reg_write32(HR_SD_ADC_ANA_CTRL, value);		/*start adc*/
}


void tls_adc_reference_sel(int ref)
{
    u32 value;
    
    value = tls_reg_read32(HR_SD_ADC_PGA_CTRL);
    if(ADC_REFERENCE_EXTERNAL == ref)
    {
		value |= BYPASS_INNER_REF_SEL;
    }
    else if(ADC_REFERENCE_INTERNAL == ref)
    {
		value &= ~BYPASS_INNER_REF_SEL;
    }
    tls_reg_write32(HR_SD_ADC_PGA_CTRL, value);    
}

void tls_adc_set_clk(int div)
{
    u32 value;

    value = tls_reg_read32(HR_CLK_SEL_CTL);
    value &= ~(0xFF<<8);
    value |=  (div&0xFF)<<8;
    tls_reg_write32(HR_CLK_SEL_CTL, value);
}

void signedToUnsignedData(u32 *adcValue)
{
	if (*adcValue &0x20000)
	{
		*adcValue = *adcValue &0x1FFFF;
	}
	else
	{
		*adcValue = *adcValue |0x20000;
	}
}

static void waitForAdcDone(void)
{
    while(1)
    {
        int reg = tls_reg_read32(HR_SD_ADC_INT_STATUS);
		//printf("reg:%x\r\n", reg);
        if(reg & ADC_INT_MASK)      //ADC中断
        {
            tls_adc_clear_irq(ADC_INT_TYPE_ADC);
            break;
        }

        if(reg & CMP_INT_MASK)      //ADC中断
        {
            tls_adc_clear_irq(ADC_INT_TYPE_ADC_COMP);
            break;
        }		
    }
}

u32 adc_get_offset(void)
{ 
    tls_adc_init(0, 0); 
	tls_adc_reference_sel(ADC_REFERENCE_INTERNAL);
	tls_adc_start_with_cpu(CONFIG_ADC_CHL_OFFSET);	

    waitForAdcDone();
	adc_offset = tls_read_adc_result(); //获取adc转换结果
	tls_adc_stop(0);

	printf("\r\noffset:%d\r\n", adc_offset);
    return adc_offset;
}

u32 adc_get_interTemp(void)
{
	u32 code1 = 0, code2 = 0;
    u32 tem;
	
	adc_get_offset();

    tls_adc_init(0, 0); 
	tls_adc_reference_sel(ADC_REFERENCE_INTERNAL);
	tls_adc_start_with_cpu(CONFIG_ADC_CHL_TEMP);
	tls_adc_set_clk(0x28);
	tls_reg_write32(HR_SD_ADC_TEMP_CTRL, ~(TEMP_CAL_OFFSET_MASK)|TEMP_EN_VAL(1));
    waitForAdcDone();
    code1 = tls_read_adc_result(); 

	tls_reg_write32(HR_SD_ADC_TEMP_CTRL, TEMP_CAL_OFFSET_MASK|TEMP_EN_VAL(1));
    waitForAdcDone();
    code2 = tls_read_adc_result(); 

    tls_adc_stop(0);
	//printf("\r\ncode:%x", (code2 - code1)/2 - adc_offset);
	tem = code1 - code2;
    return tem;
}

u16 adc_get_inputVolt(u8 channel)
{
    u32 average = 0;
    
    tls_adc_init(0, 0);
	tls_adc_reference_sel(ADC_REFERENCE_INTERNAL);
	tls_adc_start_with_cpu(channel);
	tls_adc_set_clk(0x28);	
    waitForAdcDone();
    average = tls_read_adc_result();
    tls_adc_stop(0);

	printf("\r\nch:%d,inputVolt:%d", channel, average);

    return average;
}

u16 adc_get_interVolt(void)
{
	u32 voltValue;
	adc_get_offset();

    tls_adc_init(0, 0); 
	tls_adc_reference_sel(ADC_REFERENCE_INTERNAL);
	tls_adc_start_with_cpu(CONFIG_ADC_CHL_VOLT);
	waitForAdcDone();
	voltValue = tls_read_adc_result();
	tls_adc_stop(0);
	printf("\r\ninterVolt:%d", voltValue);




    return voltValue;
}

/**
 * @brief          This function is used to get chip's internal work temperature
 *
 * @return         chip temperature, unit: 1/1000 degree
 *
 * @note           Only use to get chip's internal work temperature.
 */
u32 adc_temp(void)
{
	u32 code1 = 0, code2 = 0;
	u32 val = 0;
	int i = 0;
	u32 temptimes = 0;
	u32 gaintimes = 0;
	u32 average = 0;
	u32 averagecode1 = 0;
	u32 averagecode2 = 0;
	int temperature = 0;
	u32 temp_pga = 2;
	u32 gain_pga = 4;
	u32 num = 10;

	//printf("temp_pga:%d, gain_pga:%d, num:%d\r\n", temp_pga, gain_pga, num);
	if (temp_pga == 0)
	{
		temp_pga = 1;
	}
	if (gain_pga == 0)
	{
		gain_pga = 1;
	}
	
	temptimes = temp_pga/2 - 1;

	if (gain_pga < 16)
	{
		gaintimes = (gain_pga - 1)<<3;
	}

	tls_reg_write32(HR_SD_ADC_PGA_CTRL, CLK_CHOP_SEL_PGA_VAL(0)|GAIN_CTRL_PGA_VAL(gaintimes)|PGA_BYPASS_VAL(0)|PGA_CHOP_ENP_VAL(1)|PGA_EN_VAL(1));
	tls_adc_reference_sel(ADC_REFERENCE_INTERNAL);
	tls_adc_start_with_cpu(CONFIG_ADC_CHL_TEMP);
	tls_adc_set_clk(0x28);
	val = tls_reg_read32(HR_SD_ADC_TEMP_CTRL);
	val &= ~TEMP_GAIN_MASK;
	val |= TEMP_GAIN_VAL(temptimes);
	val |= TEMP_EN_VAL(1);

	for (i = 0; i < num; i++)
	{
		tls_adc_clear_irq(ADC_INT_TYPE_ADC);
		tls_reg_write32(HR_SD_ADC_CTRL, ANALOG_SWITCH_TIME_VAL(0x50)|ANALOG_INIT_TIME_VAL(0x50)|ADC_IRQ_EN_VAL(0x1));	
		val &= (~(TEMP_CAL_OFFSET_MASK));
		tls_reg_write32(HR_SD_ADC_TEMP_CTRL, val);		
	    tls_adc_clear_irq(ADC_INT_TYPE_ADC);
		tls_reg_write32(HR_SD_ADC_CTRL, ANALOG_SWITCH_TIME_VAL(0x50)|ANALOG_INIT_TIME_VAL(0x50));	
		tls_os_time_delay(2);
	    code1 = tls_read_adc_result(); 
		signedToUnsignedData(&code1);

		tls_adc_clear_irq(ADC_INT_TYPE_ADC);
		tls_reg_write32(HR_SD_ADC_CTRL, ANALOG_SWITCH_TIME_VAL(0x50)|ANALOG_INIT_TIME_VAL(0x50)|ADC_IRQ_EN_VAL(0x1));
		val |= TEMP_CAL_OFFSET_MASK;
		tls_reg_write32(HR_SD_ADC_TEMP_CTRL, val);

	    tls_adc_clear_irq(ADC_INT_TYPE_ADC);
		tls_reg_write32(HR_SD_ADC_CTRL, ANALOG_SWITCH_TIME_VAL(0x50)|ANALOG_INIT_TIME_VAL(0x50));	
		tls_os_time_delay(2);

	    code2 = tls_read_adc_result(); 
		signedToUnsignedData(&code2);

		averagecode1 = (code1 + averagecode1 *i)/(i+1);
		averagecode2 = (code2 + averagecode2 *i)/(i+1);
	}

	tls_adc_stop(0);

	temperature = ((int)averagecode1 - (int)averagecode2);
	temperature = ((temperature*1000/(int)(2*temp_pga*gain_pga)-4444100)*1000/15548);

	return temperature;
}

