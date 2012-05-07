/*****************************************************************
**
**  Copyright (C) 2009 Amlogic,Inc.
**  All rights reserved
**        Filename : avlfrontend.c
**
**  comment:
**        Driver for AVL6211 demodulator
**  author :
**	    Shijie.Rong@amlogic
**  version :
**	    v1.0	 12/3/30
*****************************************************************/

/*
    Driver for AVL6211 demodulator
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#ifdef ARC_700
#include <asm/arch/am_regs.h>
#else
#include <mach/am_regs.h>
#endif
#include <linux/i2c.h>
#include <linux/gpio.h>
#include "avlfrontend.h"
#include "LockSignal_Manual_source.h"



#if 1
#define pr_dbg	printk
//#define pr_dbg(fmt, args...) printk( KERN_DEBUG"DVB: " fmt, ## args)
#else
#define pr_dbg(fmt, args...)
#endif

#define pr_error(fmt, args...) printk( KERN_ERR"DVB: " fmt, ## args)

MODULE_PARM_DESC(frontend0_reset, "\n\t\t Reset GPIO of frontend0");
static int frontend0_reset = -1;
module_param(frontend0_reset, int, S_IRUGO);

MODULE_PARM_DESC(frontend0_i2c, "\n\t\t IIc adapter id of frontend0");
static int frontend0_i2c = -1;
module_param(frontend0_i2c, int, S_IRUGO);

MODULE_PARM_DESC(frontend0_tuner_addr, "\n\t\t Tuner IIC address of frontend0");
static int frontend0_tuner_addr = -1;
module_param(frontend0_tuner_addr, int, S_IRUGO);

MODULE_PARM_DESC(frontend0_demod_addr, "\n\t\t Demod IIC address of frontend0");
static int frontend0_demod_addr = -1;
module_param(frontend0_demod_addr, int, S_IRUGO);

static struct aml_fe avl6211_fe[FE_DEV_COUNT];

MODULE_PARM_DESC(frontend_reset, "\n\t\t Reset GPIO of frontend");
static int frontend_reset = -1;
module_param(frontend_reset, int, S_IRUGO);

MODULE_PARM_DESC(frontend_i2c, "\n\t\t IIc adapter id of frontend");
static int frontend_i2c = -1;
module_param(frontend_i2c, int, S_IRUGO);

MODULE_PARM_DESC(frontend_tuner_addr, "\n\t\t Tuner IIC address of frontend");
static int frontend_tuner_addr = -1;
module_param(frontend_tuner_addr, int, S_IRUGO);

MODULE_PARM_DESC(frontend_demod_addr, "\n\t\t Demod IIC address of frontend");
static int frontend_demod_addr = -1;
module_param(frontend_demod_addr, int, S_IRUGO);

static struct aml_fe avl6211_fe[FE_DEV_COUNT];

extern struct AVL_Tuner *avl6211pTuner;
extern struct AVL_DVBSx_Chip * pAVLChip_all;
#define	avl6211_support	1


static int	avl6211_diseqc_reset_overload(struct dvb_frontend* fe)
{

}

static int	avl6211_diseqc_send_master_cmd(struct dvb_frontend* fe, struct dvb_diseqc_master_cmd* cmd)
{
	AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
	AVL_uchar ucData[8];
	ucData[8]=cmd->msg;
	AVL6211_DiseqcSendCmd(ucData,cmd->msg_len);
	return r;
}

static int	avl6211_diseqc_recv_slave_reply(struct dvb_frontend* fe, struct dvb_diseqc_slave_reply* reply)
{

}

static int	avl6211_diseqc_send_burst(struct dvb_frontend* fe, fe_sec_mini_cmd_t minicmd)
{
	
	AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
	if(minicmd == SEC_MINI_A){
		r=AVL6211_SetToneOut(1);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("Send ToneBurst 1,OK\n");
		}
	}
	if(minicmd == SEC_MINI_B){
		r=AVL6211_SetToneOut(0);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("Send ToneBurst 0,OK\n");
		}
	}
	return r;

}

static int	avl6211_set_tone(struct dvb_frontend* fe, fe_sec_tone_mode_t tone)
{
	AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
	if(tone == SEC_TONE_ON){
		r=AVL6211_22K_Control(1);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("Set 22K On,OK\n");
			return r;
		}
	}
	if(tone == SEC_TONE_OFF){
		r=AVL6211_22K_Control(0);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("Set 22K Off,OK\n");
			return r;
		}
	}
	return r;
	
}

static int	avl6211_set_voltage(struct dvb_frontend* fe, fe_sec_voltage_t voltage)
{

}

static int	avl6211_enable_high_lnb_voltage(struct dvb_frontend* fe, long arg)
{
	AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
	r=AVL6211_LNB_PIO_Control(LNB0_PIN_59,1);
	if(r== AVL_DVBSx_EC_OK)
	{
		printf("Set PIO 59 to 1,OK\n");
	}
	return r;
}

#if avl6211_support



static int avl6211_blindscan_scan(struct dvb_frontend* fe, struct dvbsx_blindscanpara *pbspara)
{
		AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
		struct AVL_DVBSx_BlindScanPara * pbsParaZ ;
		pbsParaZ->m_uiStartFreq_100kHz = pbspara->m_uistartfreq_100khz;
		pbsParaZ->m_uiStopFreq_100kHz = pbspara->m_uistopfreq_100khz;
		pbsParaZ->m_uiMinSymRate_kHz = pbspara->m_uiminsymrate_khz;
		pbsParaZ->m_uiMaxSymRate_kHz = pbspara->m_uimaxsymrate_khz;
		
		r=AVL_DVBSx_IBlindScan_Scan(pbsParaZ,pbspara->m_uitunerlpf_100khz, pAVLChip_all);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("AVL_DVBSx_IBlindScan_Scan,OK\n");
		}
		return r;
}


static int avl6211_blindscan_getscanstatus(struct dvb_frontend* fe, struct dvbsx_blindscaninfo *pbsinfo)
{
		AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
		r=AVL_DVBSx_IBlindScan_GetScanStatus(pbsinfo, pAVLChip_all);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("AVL_DVBSx_IBlindScan_GetScanStatus,OK\n");
		}
		return r;
}
static int avl6211_blindscan_cancel(struct dvb_frontend* fe)
{
		
}


static int avl6211_blindscan_readchannelinfo(struct dvb_frontend* fe, struct dvb_frontend_parameters *pchannel)
{
		struct AVL_DVBSx_BlindScanAPI_Setting * pBSsetting;
		int pChannelCount;
		int i1;
		AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
		r=AVL_DVBSx_IBlindScan_ReadChannelInfo(0, pChannelCount, pBSsetting->channels_Temp, pAVLChip_all);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("AVL_DVBSx_IBlindScan_ReadChannelInfo,OK\n");
		
		}
		for( i1=0; i1<pChannelCount; i1++ ){
		pchannel[i1].frequency=pBSsetting->channels_Temp[i1].m_uiFrequency_kHz*100;
		pchannel[i1].u.qam.symbol_rate=pBSsetting->channels_Temp[i1].m_uiSymbolRate_Hz;
		}
		return r;
}


static int avl6211_blindscan_reset(struct dvb_frontend* fe)
{
		AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
		AVL_DVBSx_IBlindScan_Reset(pAVLChip_all);
		if(r== AVL_DVBSx_EC_OK)
		{
			printf("AVL_DVBSx_IBlindScan_Reset,OK\n");
		}
		return r;

}

#endif

static int avl6211_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	//struct avl6211_state *state = fe->demodulator_priv;

	return 0;
}

static int avl6211_init(struct dvb_frontend *fe)
{
	printk("frontend_reset is %d\n",frontend_reset);

	//reset
	gpio_direction_output(frontend_reset, 0);
	msleep(300);
	gpio_direction_output(frontend_reset, 1);
	struct avl6211_state *state = fe->demodulator_priv;
	//init
	AVL6211_LockSignal_Manual();
	pr_dbg("0x%x(ptuner),0x%x(pavchip)=========================demod init\r\n",avl6211pTuner->m_uiSlaveAddress,pAVLChip_all->m_SlaveAddr);
	msleep(200);
	return 0;
}

static int avl6211_sleep(struct dvb_frontend *fe)
{
	struct avl6211_state *state = fe->demodulator_priv;

//	GX_Set_Sleep(state, 1);

	return 0;
}

static int avl6211_read_status(struct dvb_frontend *fe, fe_status_t * status)
{
	struct avl6211_state *state = fe->demodulator_priv;
	unsigned char s=0;
	int ber,snr,strength;
	msleep(1000);
	s=AVL6211_GETLockStatus();
	AVL6211_GETSnr();
	AVL6211_GETPer();
//	pr_dbg("s is %d\n",s);
	//s=1;
	if(s==1)
	{
		*status = FE_HAS_LOCK|FE_HAS_SIGNAL|FE_HAS_CARRIER|FE_HAS_VITERBI|FE_HAS_SYNC;
	}
	else
	{
		*status = FE_TIMEDOUT;
	}
	
	return  0;
}

static int avl6211_read_ber(struct dvb_frontend *fe, u32 * ber)
{
	struct avl6211_state *state = fe->demodulator_priv;
	unsigned int uiBER;
	uiBER=AVL6211_GETBer();
	return 0;
}

static int avl6211_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct avl6211_state *state = fe->demodulator_priv;	
//	*strength=MxL101SF_GetSigStrength();
	*strength=AVL6211_GETSignalLevel();
	return 0;
}

static int avl6211_read_snr(struct dvb_frontend *fe, u16 * snr)
{
	struct avl6211_state *state = fe->demodulator_priv;
	unsigned int uiSNR;
	uiSNR=AVL6211_GETSnr();
	
	return 0;
}

static int avl6211_read_ucblocks(struct dvb_frontend *fe, u32 * ucblocks)
{
	ucblocks=NULL;
	return 0;
}

static int avl6211_set_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{
	struct avl6211_state *state = fe->demodulator_priv;
	struct AVL_DVBSx_Channel Channel;
	AVL_DVBSx_ErrorCode r = AVL_DVBSx_EC_OK;
	avl6211pTuner->m_uiFrequency_100kHz=p->frequency/100;
//	avl6211pTuner->m_uiFrequency_100kHz=15000;
	printk("avl6211pTuner m_uiFrequency_100kHz is %d",avl6211pTuner->m_uiFrequency_100kHz);
	
	 r = CPU_Halt(pAVLChip_all);
	if(AVL_DVBSx_EC_OK != r)
	{
		printf("CPU halt failed !\n");
		return (r);
	}

	//Change the value defined by macro and go back here when we want to lock a new channel.
//	avl6211pTuner->m_uiFrequency_100kHz = tuner_freq*10;      
	avl6211pTuner->m_uiSymbolRate_Hz = 30000000; //symbol rate of the channel to be locked.
	//This function should be called before locking the tuner to adjust the tuner LPF based on channel symbol rate.
	AVL_Set_LPF(avl6211pTuner, avl6211pTuner->m_uiSymbolRate_Hz);

	r=avl6211pTuner->m_pLockFunc(avl6211pTuner);
	if (AVL_DVBSx_EC_OK != r)
	{
 		printf("Tuner test failed !\n");
		return (r);
	}
	printf("Tuner test ok !\n");
	msleep(50);
	Channel.m_uiSymbolRate_Hz = 30000000;      //Change the value defined by macro when we want to lock a new channel.
	Channel.m_Flags = (CI_FLAG_MANUAL_LOCK_MODE) << CI_FLAG_MANUAL_LOCK_MODE_BIT;		//Manual lock Flag
									
	Channel.m_Flags |= (CI_FLAG_IQ_NO_SWAPPED) << CI_FLAG_IQ_BIT;   		//Auto IQ swap
	Channel.m_Flags |= (CI_FLAG_IQ_AUTO_BIT_AUTO) << CI_FLAG_IQ_AUTO_BIT;			//Auto IQ swap Flag
													//Support QPSK and 8PSK  dvbs2
	{
	#define Coderate				RX_DVBS2_2_3
	#define Modulation				AVL_DVBSx_MM_QPSK
	
		if (Coderate > 16 || Coderate < 6 || Modulation > 3)
		{			
			printf("Configure error !\n");
			return AVL_DVBSx_EC_GeneralFail;
		}
		Channel.m_Flags |= (CI_FLAG_DVBS2) << CI_FLAG_DVBS2_BIT;											//Disable automatic standard detection
		Channel.m_Flags |= (enum AVL_DVBSx_FecRate)(Coderate) << CI_FLAG_CODERATE_BIT;						//Manual config FEC code rate
		Channel.m_Flags |= ((enum AVL_DVBSx_ModulationMode)(Modulation)) << CI_FLAG_MODULATION_BIT;			//Manual config Modulation
	}
	//This function should be called after tuner locked to lock the channel.
	r = AVL_DVBSx_IRx_LockChannel(&Channel, pAVLChip_all);  
	if (AVL_DVBSx_EC_OK != r)
	{
		printf("Lock channel failed !\n");
		return (r);
	}

	r=AVL_DVBSx_IRx_ResetErrorStat(pAVLChip_all);
	if (AVL_DVBSx_EC_OK != r)
	{
		printf("Reset error status failed !\n");
		return (r);
	}
	
//	demod_connect(state, p->frequency,p->u.qam.modulation,p->u.qam.symbol_rate);
	state->freq=p->frequency;
	state->mode=p->u.qam.modulation ;
	state->symbol_rate=p->u.qam.symbol_rate; //these data will be writed to eeprom
//	Mxl101SF_Debug();
//	pr_dbg("avl6211=>frequency=%d,symbol_rate=%d\r\n",p->frequency,p->u.qam.symbol_rate);
	return  0;
}

static int avl6211_get_frontend(struct dvb_frontend *fe, struct dvb_frontend_parameters *p)
{//these content will be writed into eeprom .

	struct avl6211_state *state = fe->demodulator_priv;
	
	p->frequency=state->freq;
	p->u.qam.modulation=state->mode;
	p->u.qam.symbol_rate=state->symbol_rate;
	
	return 0;
}

static void avl6211_release(struct dvb_frontend *fe)
{
	struct avl6211_state *state = fe->demodulator_priv;
	
//	demod_deinit(state);
	kfree(state);
}

static struct dvb_frontend_ops avl6211_ops;

struct dvb_frontend *avl6211_attach(const struct avl6211_fe_config *config)
{
	struct avl6211_state *state = NULL;

	/* allocate memory for the internal state */
	
	state = kmalloc(sizeof(struct avl6211_state), GFP_KERNEL);
	if (state == NULL)
		return NULL;

	/* setup the state */
	state->config = *config;
	
	/* create dvb_frontend */
	memcpy(&state->fe.ops, &avl6211_ops, sizeof(struct dvb_frontend_ops));
	state->fe.demodulator_priv = state;
	
	return &state->fe;
}

static struct dvb_frontend_ops avl6211_ops = {


		.info = {
		 .name = "AMLOGIC DVB-S2",
		.type = FE_QPSK,
		.frequency_min = 800000,
		.frequency_max = 2100000,
		.frequency_stepsize = 166667,
		.frequency_tolerance = 0,
		.caps =
			FE_CAN_FEC_1_2 | FE_CAN_FEC_2_3 | FE_CAN_FEC_3_4 |
			FE_CAN_FEC_5_6 | FE_CAN_FEC_7_8 | FE_CAN_FEC_AUTO |
			FE_CAN_QPSK | FE_CAN_QAM_16 |
			FE_CAN_QAM_64 | FE_CAN_QAM_AUTO |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_RECOVER |
			FE_CAN_MUTE_TS
	},

	.release = avl6211_release,

	.init = avl6211_init,
	.sleep = avl6211_sleep,
	.i2c_gate_ctrl = avl6211_i2c_gate_ctrl,

	.set_frontend = avl6211_set_frontend,
	.get_frontend = avl6211_get_frontend,	
	.read_status = avl6211_read_status,
	.read_ber = avl6211_read_ber,
	.read_signal_strength =avl6211_read_signal_strength,
	.read_snr = avl6211_read_snr,
	.read_ucblocks = avl6211_read_ucblocks,


	.diseqc_reset_overload = avl6211_diseqc_reset_overload,
	.diseqc_send_master_cmd = avl6211_diseqc_send_master_cmd,
	.diseqc_recv_slave_reply = avl6211_diseqc_recv_slave_reply,
	.diseqc_send_burst = avl6211_diseqc_send_burst,
	.set_tone = avl6211_set_tone,
	.set_voltage = avl6211_set_voltage,
	.enable_high_lnb_voltage = avl6211_enable_high_lnb_voltage,
#if avl6211_support	


	.blindscan_scan	=	avl6211_blindscan_scan,
	.blindscan_getscanstatus	=	avl6211_blindscan_getscanstatus,
	.blindscan_cancel	=	avl6211_blindscan_cancel,
	.blindscan_readchannelinfo	=	avl6211_blindscan_readchannelinfo,
	.blindscan_reset	=	avl6211_blindscan_reset,
#endif

};

static void avl6211_fe_release(struct aml_dvb *advb, struct aml_fe *fe)
{
	if(fe && fe->fe) {
		pr_dbg("release avl6211 frontend %d\n", fe->id);
		dvb_unregister_frontend(fe->fe);
		dvb_frontend_detach(fe->fe);
		if(fe->cfg){
			kfree(fe->cfg);
			fe->cfg = NULL;
		}
		fe->id = -1;
	}
}

static int avl6211_fe_init(struct aml_dvb *advb, struct platform_device *pdev, struct aml_fe *fe, int id)
{
	struct dvb_frontend_ops *ops;
	int ret;
	struct resource *res;
	struct avl6211_fe_config *cfg;
	char buf[32];
	
	pr_dbg("init avl6211 frontend %d\n", id);


	cfg = kzalloc(sizeof(struct avl6211_fe_config), GFP_KERNEL);
	if (!cfg)
		return -ENOMEM;
	
	cfg->reset_pin = frontend_reset;
	if(cfg->reset_pin==-1) {
		snprintf(buf, sizeof(buf), "frontend%d_reset_pin", id);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, buf);
		if (!res) {
			pr_error("cannot get resource \"%s\"\n", buf);
			ret = -EINVAL;
			goto err_resource;
		}
		cfg->reset_pin = res->start;		
	}
	
	cfg->i2c_id = frontend_i2c;
	if(cfg->i2c_id==-1) {
		snprintf(buf, sizeof(buf), "frontend%d_i2c", id);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, buf);
		if (!res) {
			pr_error("cannot get resource \"%s\"\n", buf);
			ret = -EINVAL;
			goto err_resource;
		}
		cfg->i2c_id = res->start;
	}
	
	cfg->tuner_addr = frontend_tuner_addr;
	
	if(cfg->tuner_addr==-1) {
		snprintf(buf, sizeof(buf), "frontend%d_tuner_addr", id);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, buf);
		if (!res) {
			pr_error("cannot get resource \"%s\"\n", buf);
			ret = -EINVAL;
			goto err_resource;
		}
		cfg->tuner_addr = res->start>>1;
	}
	
	cfg->demod_addr = frontend_demod_addr;
	if(cfg->demod_addr==-1) {
		snprintf(buf, sizeof(buf), "frontend%d_demod_addr", id);
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, buf);
		if (!res) {
			pr_error("cannot get resource \"%s\"\n", buf);
			ret = -EINVAL;
			goto err_resource;
		}
		cfg->demod_addr = res->start>>1;
	}
	frontend_reset = cfg->reset_pin;
	frontend_i2c = cfg->i2c_id;
	frontend_tuner_addr = cfg->tuner_addr;
	frontend_demod_addr = cfg->demod_addr;	
	fe->fe = avl6211_attach(cfg);
	if (!fe->fe) {
		ret = -ENOMEM;
		goto err_resource;
	}

	if ((ret=dvb_register_frontend(&advb->dvb_adapter, fe->fe))) {
		pr_error("frontend registration failed!");
		ops = &fe->fe->ops;
		if (ops->release != NULL)
			ops->release(fe->fe);
		fe->fe = NULL;
		goto err_resource;
	}
	
	fe->id = id;
	fe->cfg = cfg;
	
	return 0;

err_resource:
	kfree(cfg);
	return ret;
}

int avl6211_get_fe_config(struct avl6211_fe_config *cfg)
{
	struct i2c_adapter *i2c_handle;
	cfg->i2c_id = frontend_i2c;
	cfg->demod_addr = frontend_demod_addr;
	cfg->tuner_addr = frontend_tuner_addr;
	cfg->reset_pin = frontend_reset;
//	printk("\n frontend_i2c is %d,,frontend_demod_addr is %x,frontend_tuner_addr is %x,frontend_reset is %d",frontend_i2c,frontend_demod_addr,frontend_tuner_addr,frontend_reset);
	i2c_handle = i2c_get_adapter(cfg->i2c_id);
	if (!i2c_handle) {
		printk("cannot get i2c adaptor\n");
		return 0;
	}
	cfg->i2c_adapter =i2c_handle;
	return 1;
	
//	printk("\n frontend0_i2c is %d, frontend_i2c is %d,",frontend0_i2c,frontend_i2c);
	
}

static int avl6211_fe_probe(struct platform_device *pdev)
{
	struct aml_dvb *dvb = aml_get_dvb_device();
	
	if(avl6211_fe_init(dvb, pdev, &avl6211_fe[0], 0)<0)
		return -ENXIO;

	platform_set_drvdata(pdev, &avl6211_fe[0]);
	
	return 0;
}

static int avl6211_fe_remove(struct platform_device *pdev)
{
	struct aml_fe *drv_data = platform_get_drvdata(pdev);
	struct aml_dvb *dvb = aml_get_dvb_device();

	platform_set_drvdata(pdev, NULL);
	
	avl6211_fe_release(dvb, drv_data);
	
	return 0;
}

static struct platform_driver aml_fe_driver = {
	.probe		= avl6211_fe_probe,
	.remove		= avl6211_fe_remove,	
	.driver		= {
		.name	= "avl6211",
		.owner	= THIS_MODULE,
	}
};

static int __init avlfrontend_init(void)
{
	return platform_driver_register(&aml_fe_driver);
}


static void __exit avlfrontend_exit(void)
{
	platform_driver_unregister(&aml_fe_driver);
}

module_init(avlfrontend_init);
module_exit(avlfrontend_exit);


MODULE_DESCRIPTION("avl6211 DVB-S2 Demodulator driver");
MODULE_AUTHOR("RSJ");
MODULE_LICENSE("GPL");


