//*--------------------------------------------------------------------------------------
//* Include Standard files
//*--------------------------------------------------------------------------------------
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "../aml_demod.h"
#include "../demod_func.h"

#include "tmNxTypes.h"
#include "tmCompId.h"
#include "tmFrontEnd.h"
#include "tmbslFrontEndTypes.h"
#include "tmsysFrontEndTypes.h"

struct aml_demod_i2c *I2C_adap;

//*--------------------------------------------------------------------------------------
//* Include Driver files
//*--------------------------------------------------------------------------------------
#include "tmsysOM3869.h"
#include "tmbslNT220HN.h"

//*--------------------------------------------------------------------------------------
//* Prototype of function to be provided by customer
//*--------------------------------------------------------------------------------------
tmErrorCode_t 	UserWrittenI2CRead(tmUnitSelect_t tUnit,UInt32 AddrSize, UInt8* pAddr,UInt32 ReadLen, UInt8* pData);
tmErrorCode_t 	UserWrittenI2CWrite (tmUnitSelect_t tUnit, UInt32 AddrSize, UInt8* pAddr,UInt32 WriteLen, UInt8* pData);
tmErrorCode_t 	UserWrittenWait(tmUnitSelect_t tUnit, UInt32 tms);
tmErrorCode_t 	UserWrittenPrint(UInt32 level, const char* format, ...);
tmErrorCode_t  	UserWrittenMutexInit(ptmbslFrontEndMutexHandle *ppMutexHandle);
tmErrorCode_t  	UserWrittenMutexDeInit( ptmbslFrontEndMutexHandle pMutex);
tmErrorCode_t  	UserWrittenMutexAcquire(ptmbslFrontEndMutexHandle pMutex, UInt32 timeOut);
tmErrorCode_t  	UserWrittenMutexRelease(ptmbslFrontEndMutexHandle pMutex);


//*--------------------------------------------------------------------------------------
//* Function Name       : Main
//* Object              : Software entry point
//* Input Parameters    : none.
//* Output Parameters   : none.
//*--------------------------------------------------------------------------------------
int set_tuner_fj2207(struct aml_demod_sta *demod_sta, 
		     struct aml_demod_i2c *adap)
{//* Begin
   /* Variable declarations */
   tmErrorCode_t err = TM_OK;
   tmbslFrontEndDependency_t sSrvTunerFunc;
   tmTunerOnlyRequest_t TuneRequest;
   tmsysFrontEndState_t  LockStatus;
   tmUnitSelect_t  Tuner_Master = 0;
#ifdef SLAVETUNER
   tmUnitSelect_t  Tuner_Slave= 1;
#endif
    unsigned long ch_freq;
    int ch_if;
    int ch_bw;

    ch_freq = demod_sta->ch_freq; // kHz
    ch_if   = demod_sta->ch_if;   // kHz 
    ch_bw   = demod_sta->ch_bw / 1000; // MHz

    printk("Set Tuner FJ2207 to %ld kHz\n", ch_freq);
   
   I2C_adap = adap;

/* Low layer struct set-up to link with user written functions */
   sSrvTunerFunc.sIo.Write             = UserWrittenI2CWrite;
   sSrvTunerFunc.sIo.Read              = UserWrittenI2CRead;
   sSrvTunerFunc.sTime.Get             = Null;
   sSrvTunerFunc.sTime.Wait            = UserWrittenWait;
   sSrvTunerFunc.sDebug.Print          = UserWrittenPrint;
   sSrvTunerFunc.sMutex.Init           = UserWrittenMutexInit;
   sSrvTunerFunc.sMutex.DeInit         = UserWrittenMutexDeInit;
   sSrvTunerFunc.sMutex.Acquire        = UserWrittenMutexAcquire;
   sSrvTunerFunc.sMutex.Release        = UserWrittenMutexRelease;
   sSrvTunerFunc.dwAdditionalDataSize  = 0;
   sSrvTunerFunc.pAdditionalData       = Null;
   
   /* OM3869 Master Driver low layer setup */
   err = tmsysOM3869Init(Tuner_Master, &sSrvTunerFunc);
   if(err != TM_OK)
       return err;
#ifdef SLAVETUNER
   /* OM3869 Slave Driver low layer setup */
   err = tmsysOM3869Init(Tuner_Slave, &sSrvTunerFunc);
   if(err != TM_OK)
       return err;
#endif   
   /* OM3869 Master Hardware power state */
   err = tmsysOM3869SetPowerState(Tuner_Master, tmPowerOn);

   /* OM3869 Master Hardware initialization */
   err = tmsysOM3869Reset(Tuner_Master);
   if(err != TM_OK)
       return err;
#ifdef SLAVETUNER   
   /* OM3869 Slave Hardware power state */
   err = tmsysOM3869SetPowerState(Tuner_Slave, tmPowerOn);

   /* OM3869 Slave Hardware initialization */
   err = tmsysOM3869Reset(Tuner_Slave);
   if(err != TM_OK)
       return err;
#endif   

   TuneRequest.dwFrequency = ch_freq * 1000; 
   TuneRequest.dwStandard = demod_sta->dvb_mode==0 ?
       tmNT220x_QAM_8MHz : tmNT220x_DVBT_8MHz;

   err = tmsysOM3869SendRequest(Tuner_Master,&TuneRequest,sizeof(TuneRequest), TRT_TUNER_ONLY );
   if(err != TM_OK)
       return err;
   /* OM3869 Master Get locked status */
   err = tmsysOM3869GetLockStatus(Tuner_Master,&LockStatus);
   if(err != TM_OK)
       return err;
  
#ifdef SLAVETUNER   
  /* OM3869 Slave Send Request 770 MHz standard DVB-C 8 Mhz */
   TuneRequest.dwFrequency = 770000000;
   TuneRequest.dwStandard = tmNT220x_QAM_8MHz;
   err = tmsysOM3869SendRequest(Tuner_Slave,&TuneRequest,sizeof(TuneRequest), TRT_TUNER_ONLY );
   if(err != TM_OK)
       return err;
   /* OM3869 Slave Get locked status */
   err = tmsysOM3869GetLockStatus(Tuner_Slave,&LockStatus);
   if(err != TM_OK)
       return err;

/* To clean OM3869 driver */

/* DeInitialize OM3869 Slave Driver */
   err = tmsysOM3869DeInit(Tuner_Slave);
#endif

/* DeInitialize OM3869 Master Driver */
//uncomment this if required//   err = tmsysOM3869DeInit(Tuner_Master);

   return err;

}//* End

//*--------------------------------------------------------------------------------------
//* Template of function to be provided by customer
//*--------------------------------------------------------------------------------------

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenI2CRead
//* Object              : 
//* Input Parameters    : 	tmUnitSelect_t tUnit
//* 						UInt32 AddrSize,
//* 						UInt8* pAddr,
//* 						UInt32 ReadLen,
//* 						UInt8* pData
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t UserWrittenI2CRead(tmUnitSelect_t tUnit,	UInt32 AddrSize, UInt8* pAddr,
UInt32 ReadLen, UInt8* pData)
{
    tmErrorCode_t err = TM_OK;
    int ret;
    struct i2c_msg msg[2];
    
    msg[0].addr = I2C_adap->addr;
    msg[0].flags = 0 | I2C_M_IGNORE_NAK;
    msg[0].len = AddrSize;
    msg[0].buf = pAddr;
       
    msg[1].addr = I2C_adap->addr;
    msg[1].flags = I2C_M_RD | I2C_M_IGNORE_NAK;
    msg[1].len = ReadLen;
    msg[1].buf = pData;
    
    ret = bit_xfer(I2C_adap, msg, 2);
    if (ret != 2) err = -1;

    return err;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenI2CWrite
//* Object              : 
//* Input Parameters    : 	tmUnitSelect_t tUnit
//* 						UInt32 AddrSize,
//* 						UInt8* pAddr,
//* 						UInt32 WriteLen,
//* 						UInt8* pData
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t UserWrittenI2CWrite (tmUnitSelect_t tUnit, 	UInt32 AddrSize, UInt8* pAddr,
UInt32 WriteLen, UInt8* pData)
{
    tmErrorCode_t err = TM_OK;
    int i, ret;
    struct i2c_msg msg;
    UInt8 buf[70];

    for (i=0; i<AddrSize; i++) buf[i] = pAddr[i];
    for (i=0; i<WriteLen; i++) buf[AddrSize+i] = pData[i];

    msg.addr = I2C_adap->addr;
    msg.flags = 0 | I2C_M_IGNORE_NAK;
    msg.len = AddrSize + WriteLen;
    msg.buf = buf;
    
    ret = bit_xfer(I2C_adap, &msg, 1);
    if (ret != 1) err = -1;

    return err;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenWait
//* Object              : 
//* Input Parameters    : 	tmUnitSelect_t tUnit
//* 						UInt32 tms
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t UserWrittenWait(tmUnitSelect_t tUnit, UInt32 tms)
{
    tmErrorCode_t err = TM_OK;

    mdelay(tms);

    return err;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenPrint
//* Object              : 
//* Input Parameters    : 	UInt32 level, const char* format, ...
//* 						
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t UserWrittenPrint(UInt32 level, const char* format, ...)
{
    /* Variable declarations */
    tmErrorCode_t err = TM_OK;

    printk("printk .... called!\n");

/* Customer code here */
/* ...*/

/* ...*/
/* End of Customer code here */

   return err;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenMutexInit
//* Object              : 
//* Input Parameters    : 	ptmbslFrontEndMutexHandle *ppMutexHandle
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t  UserWrittenMutexInit(ptmbslFrontEndMutexHandle *ppMutexHandle)
{
   /* Variable declarations */
   tmErrorCode_t err = TM_OK;

/* Customer code here */
/* ...*/

/* ...*/
/* End of Customer code here */

   return err;
}


//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenMutexDeInit
//* Object              : 
//* Input Parameters    : 	 ptmbslFrontEndMutexHandle pMutex
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t  UserWrittenMutexDeInit( ptmbslFrontEndMutexHandle pMutex)
{
   /* Variable declarations */
   tmErrorCode_t err = TM_OK;

/* Customer code here */
/* ...*/

/* ...*/
/* End of Customer code here */

   return err;
}



//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenMutexAcquire
//* Object              : 
//* Input Parameters    : 	ptmbslFrontEndMutexHandle pMutex, UInt32 timeOut
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t  UserWrittenMutexAcquire(ptmbslFrontEndMutexHandle pMutex, UInt32 timeOut)
{
   /* Variable declarations */
   tmErrorCode_t err = TM_OK;

/* Customer code here */
/* ...*/

/* ...*/
/* End of Customer code here */

   return err;
}

//*--------------------------------------------------------------------------------------
//* Function Name       : UserWrittenMutexRelease
//* Object              : 
//* Input Parameters    : 	ptmbslFrontEndMutexHandle pMutex
//* Output Parameters   : tmErrorCode_t.
//*--------------------------------------------------------------------------------------
tmErrorCode_t  UserWrittenMutexRelease(ptmbslFrontEndMutexHandle pMutex)
{
   /* Variable declarations */
   tmErrorCode_t err = TM_OK;

/* Customer code here */
/* ...*/

/* ...*/
/* End of Customer code here */

   return err;
}


