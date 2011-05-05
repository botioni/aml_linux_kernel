include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <mach/mt6516_pll.h>
#include <linux/time.h>
 
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif 
 
#include <linux/delay.h>
#include <mach/mt6516_gpio.h>
#include <mach/mt6516_devs.h>
#include <mach/mt6516_typedefs.h>
#include <mach/mt6516_boot.h>
 
 extern struct tpd_device *tpd;
 
 struct i2c_client *i2c_client = NULL;
 struct task_struct *thread = NULL;
 
 static DECLARE_WAIT_QUEUE_HEAD(waiter);
 
 struct early_suspend early_suspend;
 
#ifdef CONFIG_HAS_EARLYSUSPEND
 static void tpd_early_suspend(struct early_suspend *handler);
 static void tpd_late_resume(struct early_suspend *handler);
#endif 
 
 static void tpd_eint_interrupt_handler(void);
 
 
 extern void MT6516_EINTIRQUnmask(unsigned int line);
 extern void MT6516_EINTIRQMask(unsigned int line);
 extern void MT6516_EINT_Set_HW_Debounce(kal_uint8 eintno, kal_uint32 ms);
 extern kal_uint32 MT6516_EINT_Set_Sensitivity(kal_uint8 eintno, kal_bool sens);
 extern void MT6516_EINT_Registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                      kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                      kal_bool auto_umask);
 
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
 static int tpd_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
 static int __devexit tpd_remove(struct i2c_client *client);
 static int touch_event_handler(void *unused);
 

 static int tpd_flag = 0;
 static int point_num = 0;
 static int p_point_num = 0;
 
 #define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12

//register define

struct touch_info {
    int y[3];
    int x[3];
    int p[3];
    int count;
};
 
 static const struct i2c_device_id tpd_id[] = {{TPD_DEVICE,0},{}};
 unsigned short force[] = {2,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
 static const unsigned short * const forces[] = { force, NULL };
 static struct i2c_client_address_data addr_data = { .forces = forces, };
 
 
 static struct i2c_driver tpd_driver = {
  .driver = {
     .name = TPD_DEVICE,
     .owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = __devexit_p(tpd_remove),
  .id_table = tpd_id,
  .detect = tpd_detect,
  .address_data = &addr_data,
 };
 

 void tpd_down(int x, int y, int p) {
    // input_report_abs(tpd->dev, ABS_PRESSURE, p);
     input_report_key(tpd->dev, BTN_TOUCH, 1);
     input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
     input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
     input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
     //printk("D[%4d %4d %4d] ", x, y, p);
     input_mt_sync(tpd->dev);
     TPD_DOWN_DEBUG_TRACK(x,y);
 }
 
 int tpd_up(int x, int y,int *count) {
     if(*count>0) {
         input_report_abs(tpd->dev, ABS_PRESSURE, 0);
         input_report_key(tpd->dev, BTN_TOUCH, 0);
         input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
         input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
         input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
         //printk("U[%4d %4d %4d] ", x, y, 0);
         input_mt_sync(tpd->dev);
         TPD_UP_DEBUG_TRACK(x,y);
         (*count)--;
         return 1;
     } return 0;
 }

 int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
 {

    int i = 0;
    
    char data[30] = {0};

    u16 high_byte,low_byte;

    p_point_num = point_num;

    i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 8, &(data[0]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x08, 8, &(data[8]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0x10, 8, &(data[16]));
    i2c_smbus_read_i2c_block_data(i2c_client, 0xa6, 1, &(data[24]));
    TPD_DEBUG("FW version=%x]\n",data[24]);
    
    TPD_DEBUG("received raw data from touch panel as following:\n");
    TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
    TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
    TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

    
    /* Device Mode[2:0] == 0 :Normal operating Mode*/
    if(data[0] & 0x70 != 0) return false; 

    /*get the number of the touch points*/
    point_num= data[2] & 0x0f;
    
    TPD_DEBUG("point_num =%d\n",point_num);
    
//  if(point_num == 0) return false;

       TPD_DEBUG("Procss raw data...\n");

        
        for(i = 0; i < point_num; i++)
        {
            cinfo->p[i] = data[3+6*i] >> 6; //event flag 

           /*get the X coordinate, 2 bytes*/
            high_byte = data[3+6*i];
            high_byte <<= 8;
            high_byte &= 0x0f00;
            low_byte = data[3+6*i + 1];
            cinfo->x[i] = high_byte |low_byte;
            //add 1209
           cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibrate
           
            /*get the Y coordinate, 2 bytes*/
            
            high_byte = data[3+6*i+2];
            high_byte <<= 8;
            high_byte &= 0x0f00;
            low_byte = data[3+6*i+3];
            cinfo->y[i] = high_byte |low_byte;
            //add 1209
            cinfo->y[i]=  cinfo->y[i] * 800 >> 11; 
            
            cinfo->count++;
            
        }
        TPD_DEBUG(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);   
        TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);       
        TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);   
          
     return true;

 };

 static int touch_event_handler(void *unused)
 {
  
    struct touch_info cinfo, pinfo;

     struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
     sched_setscheduler(current, SCHED_RR, &param);
 
     do
     {
      MT6516_EINTIRQUnmask(CUST_EINT_TOUCH_PANEL_NUM); 
         set_current_state(TASK_INTERRUPTIBLE); 
          wait_event_interruptible(waiter,tpd_flag!=0);
                         
             tpd_flag = 0;
             
         set_current_state(TASK_RUNNING);
         

          if (tpd_touchinfo(&cinfo, &pinfo)) {
          printk("point_num = %d\n",point_num);
          
            if(point_num >0) {
                tpd_down(cinfo.x[0], cinfo.y[0], 1);
             if(point_num>1)
                {
                tpd_down(cinfo.x[1], cinfo.y[1], 1);
               if(point_num >2) tpd_down(cinfo.x[2], cinfo.y[2], 1);
                }
                input_sync(tpd->dev);
                printk("press --->\n");
                
            } else  {
            printk("release --->\n"); 
                input_mt_sync(tpd->dev);
                input_sync(tpd->dev);
            }
        }

 }while(!kthread_should_stop());
 
     return 0;
 }
 
 static int tpd_detect (struct i2c_client *client, int kind, struct i2c_board_info *info) 
 {
    int error;
 
     hwPowerDown(TPD_POWER_SOURCE,"TP");
     hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
 
     mt_set_gpio_mode(GPIO61, 0x01);
     mt_set_gpio_dir(GPIO61, GPIO_DIR_IN);
     mt_set_gpio_pull_enable(GPIO61, GPIO_PULL_ENABLE);
     mt_set_gpio_pull_select(GPIO61, GPIO_PULL_UP);
     msleep(100);
 
     strcpy(info->type, TPD_DEVICE);

     thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
      if (IS_ERR(thread))
         { 
          error = PTR_ERR(thread);
          TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", error);
        }
 
      return 0;
 }
 
 static void tpd_eint_interrupt_handler(void)
 {
     TPD_DEBUG("TPD interrupt has been triggered\n");
     tpd_flag = 1;
     wake_up_interruptible(&waiter);
     
 }
 static int __devinit tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {  
    int retval = TPD_OK;
    i2c_client = client;
    
#ifdef CONFIG_HAS_EARLYSUSPEND
          if (!(retval < TPD_OK)) 
          {
              early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
              early_suspend.suspend = tpd_early_suspend;
              early_suspend.resume = tpd_late_resume;
              register_early_suspend(&early_suspend);
          }
#endif
  
      MT6516_EINT_Set_Sensitivity(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
      MT6516_EINT_Set_HW_Debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
      MT6516_EINT_Registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
      MT6516_EINTIRQUnmask(CUST_EINT_TOUCH_PANEL_NUM);
 
    msleep(100);
 
    TPD_DMESG("Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
      
   return retval;
   
 }

 static int __devexit tpd_remove(struct i2c_client *client)
 
 {
  #ifdef CONFIG_HAS_EARLYSUSPEND
     unregister_early_suspend(&early_suspend);
  #endif /* CONFIG_HAS_EARLYSUSPEND */
   
     TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 
 int tpd_local_init(void)
 {
   int retval;
 
  TPD_DMESG("Focaltech FT5206 I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);
 
 
   retval = i2c_add_driver(&tpd_driver);
 
   return retval;
 
 }

 int tpd_resume(struct i2c_client *client)
 {
  int retval = TPD_OK;
 
   TPD_DEBUG("TPD wake up\n");
   
     hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP"); 
   MT6516_EINTIRQUnmask(CUST_EINT_TOUCH_PANEL_NUM);  
    
     return retval;
 }
 
 int tpd_suspend(struct i2c_client *client, pm_message_t message)
 {
     int retval = TPD_OK;
 
     TPD_DEBUG("TPD enter sleep\n");
     MT6516_EINTIRQMask(CUST_EINT_TOUCH_PANEL_NUM);
     hwPowerDown(TPD_POWER_SOURCE,"TP");
     return retval;
 }
 
 
 
 
#ifdef CONFIG_HAS_EARLYSUSPEND
 static void tpd_early_suspend(struct early_suspend *handler)
 {
     tpd_suspend(i2c_client, PMSG_SUSPEND);
 }
 
 static void tpd_late_resume(struct early_suspend *handler)
 {
     tpd_resume(i2c_client);
 }
#endif


/* ---------------------------------------------------------------------
*
*   Focal Touch panel upgrade related driver
*
*
----------------------------------------------------------------------*/
#define CONFIG_SUPPORT_FTS_CTP_UPG


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70


void delay_ms(FTS_WORD  w_ms)
{
    //platform related, please implement this function
    msleep( w_ms );
}


/*
[function]: 
    callback: read data from ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[out]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    
    ret=i2c_master_recv(i2c_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("i2c_read_interface error\n");
        return FTS_FALSE;
    }
  
    return FTS_TRUE;
}

/*
[function]: 
    callback: write data to ctpm by i2c interface,implemented by special user;
[parameters]:
    bt_ctpm_addr[in]    :the address of the ctpm;
    pbt_buf[in]        :data buffer;
    dw_lenth[in]        :the length of the data buffer;
[return]:
    FTS_TRUE     :success;
    FTS_FALSE    :fail;
*/
FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(i2c_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("i2c_write_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}


/***************************************************************************************/

/*
[function]: 
    read out the register value.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[out]    :the returned register value;
    bt_len[in]        :length of pbt_buf, should be set to 2;        
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BYTE fts_register_read(FTS_BYTE e_reg_name, FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    FTS_BYTE read_cmd[3]= {0};
    FTS_BYTE cmd_len     = 0;

    read_cmd[0] = e_reg_name;
    cmd_len = 1;    

    /*call the write callback function*/
    if(!i2c_write_interface(I2C_CTPM_ADDRESS, &read_cmd, cmd_len))
    {
        return FTS_FALSE;
    }

    /*call the read callback function to get the register value*/        
    if(!i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len))
    {
        return FTS_FALSE;
    }
    return FTS_TRUE;
}

/*
[function]: 
    write a value to register.
[parameters]:
    e_reg_name[in]    :register name;
    pbt_buf[in]        :the returned register value;
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL fts_register_write(FTS_BYTE e_reg_name, FTS_BYTE bt_value)
{
    FTS_BYTE ecc = 0;
    FTS_BYTE write_cmd[2] = {0};

    write_cmd[0] = e_reg_name;
    write_cmd[1] = bt_value;

    /*call the write callback function*/
    return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, 2);
}

/*
[function]: 
    send a command to ctpm.
[parameters]:
    btcmd[in]        :command code;
    btPara1[in]    :parameter 1;    
    btPara2[in]    :parameter 2;    
    btPara3[in]    :parameter 3;    
    num[in]        :the valid input parameter numbers, if only command code needed and no parameters followed,then the num is 1;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, &write_cmd, num);
}

/*
[function]: 
    write data to ctpm , the destination address is 0.
[parameters]:
    pbt_buf[in]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

/*
[function]: 
    read out data from ctpm,the destination address is 0.
[parameters]:
    pbt_buf[out]    :point to data buffer;
    bt_len[in]        :the data numbers;    
[return]:
    FTS_TRUE    :success;
    FTS_FALSE    :io fail;
*/
FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}


/*
[function]: 
    burn the FW to ctpm.
[parameters]:(ref. SPEC)
    pbt_buf[in]    :point to Head+FW ;
    dw_lenth[in]:the length of the FW + 6(the Head length);    
    bt_ecc[in]    :the ECC of the FW
[return]:
    ERR_OK        :no error;
    ERR_MODE    :fail to switch to UPDATE mode;
    ERR_READID    :read id fail;
    ERR_ERASE    :erase chip fail;
    ERR_STATUS    :status error;
    ERR_ECC        :ecc error.
*/


#define    FTS_PACKET_LENGTH        2

static unsigned char CTPM_FW[]=
{
#include "ft_app.i"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE cmd_len     = 0;
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;
    FTS_BYTE ecc = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    fts_register_write(0xfc,0xaa);
    delay_ms(50);
     /*write 0x55 to register 0xfc*/
    fts_register_write(0xfc,0x55);
    printk("Step 1: Reset CTPM test\n");

    delay_ms(40);

    /*********Step 2:Enter upgrade mode *****/
     auc_i2c_write_buf[0] = 0x55;
     auc_i2c_write_buf[1] = 0xaa;
     i2c_write_interface(I2C_CTPM_ADDRESS, auc_i2c_write_buf, 2);
     printk("Step 2: Enter update mode. \n");

    /*********Step 3:check READ-ID***********************/
    /*send the opration head*/
    do{
        if(i > 3)
        {
            return ERR_READID; 
        }
        /*read out the CTPM ID*/
        
        cmd_write(0x90,0x00,0x00,0x00,4);
        byte_read(reg_val,2);
        i++;
        printk("Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }while(reg_val[0] != 0x79 || reg_val[1] != 0x03);

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);
    delay_ms(1500);
    printk("Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        delay_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        delay_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(&reg_val,1);
    printk("Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
   }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}


#endif

