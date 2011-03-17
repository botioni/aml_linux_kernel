# wgt7310
EXTRA_CFLAGS += -DLINUX -DHOST_COMPILE -DFW_RUNNING_AS_STA -DSTATIC_LIB -DDEBUG -DTI_DBG -DREPORT_LOG  -DO2 \
			-D__BYTE_ORDER_LITTLE_ENDIAN  -DTNETW1273 -DSUPPL_WPS_SUPPORT -D__LINUX__\
			-Idrivers/amlogic/wifi/ti_wgt7310/inc/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/AirLink_Managment/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Application/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Connection_Managment/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Ctrl_Interface/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Data_link/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Sta_Management/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Test/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/Ctrl/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/Ctrl/Export_Inc/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/Data_Service/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/Data_Service/Export_Inc/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/FirmwareApi/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/FW_Transfer/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/FW_Transfer/Export_Inc/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/MacServices/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/MacServices/Export_Inc/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/TWDriver/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/TWD/TwIf/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/Txn/ \
			-Idrivers/amlogic/wifi/ti_wgt7310/utils/

OFILES = amlogic-wifi.o SdioDrv.o WlanDrvIf.o osapi.o osmemapi.o osRgstry_parser.o ipc_k.o \
	WlanDrvWext.o CmdInterpretWext.o RxBuf.o host_platform.o \
	SdioAdapter.o osRgstry.o

AirLinkOFILES = AirLink_Managment/measurementMgr.o AirLink_Managment/measurementMgrSM.o AirLink_Managment/regulatoryDomain.o \
			AirLink_Managment/requestHandler.o AirLink_Managment/SoftGemini.o AirLink_Managment/spectrumMngmntMgr.o AirLink_Managment/SwitchChannel.o

ApplicationOFILES = Application/roamingMngr.o Application/roamingMngr_autoSM.o  Application/roamingMngr_manualSM.o Application/scanMngr.o

ConnectionOFILES = Connection_Managment/mainSecKeysOnly.o Connection_Managment/mainSecNull.o Connection_Managment/mainSecSm.o \
		Connection_Managment/rsn.o Connection_Managment/sme.o Connection_Managment/smeSelect.o Connection_Managment/smeSm.o \
		 Connection_Managment/unicastKey802_1x.o Connection_Managment/unicastKeyNone.o Connection_Managment/unicastKeySM.o \
		 Connection_Managment/admCtrl.o Connection_Managment/admCtrlNone.o Connection_Managment/admCtrlWep.o \
		 Connection_Managment/admCtrlWpa2.o Connection_Managment/admCtrlWpa.o Connection_Managment/apConn.o \
		 Connection_Managment/broadcastKey802_1x.o Connection_Managment/broadcastKeyNone.o Connection_Managment/broadcastKeySM.o \
		 Connection_Managment/conn.o Connection_Managment/connIbss.o Connection_Managment/connInfra.o \
		 Connection_Managment/externalSec.o Connection_Managment/keyDerive.o Connection_Managment/keyDeriveAes.o \
		 Connection_Managment/keyDeriveCkip.o Connection_Managment/keyDeriveTkip.o Connection_Managment/keyDeriveWep.o \
		 Connection_Managment/keyParser.o Connection_Managment/keyParserExternal.o Connection_Managment/keyParserWep.o \
		 Connection_Managment/mainKeysSm.o

Ctrl_InterfaceOFILES = Ctrl_Interface/CmdDispatcher.o Ctrl_Interface/CmdHndlr.o Ctrl_Interface/DrvMain.o Ctrl_Interface/EvHandler.o

Data_linkOFILES = Data_link/Ctrl.o Data_link/GeneralUtil.o  Data_link/rx.o Data_link/TrafficMonitor.o Data_link/txCtrl.o \
		 Data_link/txCtrlParams.o Data_link/txCtrlServ.o Data_link/TxDataClsfr.o Data_link/txDataQueue.o \
		 Data_link/txMgmtQueue.o Data_link/txPort.o

Sta_ManagementOFILES = Sta_Management/openAuthSm.o Sta_Management/PowerMgr.o  Sta_Management/PowerMgrDbgPrint.o Sta_Management/PowerMgrKeepAlive.o \
		 Sta_Management/qosMngr.o Sta_Management/roamingInt.o Sta_Management/ScanCncn.o Sta_Management/ScanCncnApp.o \
		 Sta_Management/ScanCncnOsSm.o Sta_Management/ScanCncnSm.o Sta_Management/ScanCncnSmSpecific.o Sta_Management/scanResultTable.o \
		 Sta_Management/scr.o Sta_Management/sharedKeyAuthSm.o Sta_Management/siteHash.o Sta_Management/siteMgr.o \
		 Sta_Management/StaCap.o Sta_Management/systemConfig.o Sta_Management/templates.o Sta_Management/trafficAdmControl.o \
		 Sta_Management/assocSM.o Sta_Management/authSm.o Sta_Management/currBss.o Sta_Management/healthMonitor.o \
		 Sta_Management/mlmeBuilder.o Sta_Management/mlmeParser.o Sta_Management/mlmeSm.o

TestOFILES = Test/connDebug.o Test/ctrlDbg.o  Test/debug.o Test/fwdriverdebug.o Test/HealthMonitorDbg.o Test/measurementDbg.o \
			 Test/MibDbg.o Test/PowerMgrDebug.o Test/qosMngrDbg.o Test/roamingMgrDebug.o Test/rsnDbg.o Test/RxDbg.o \
			 Test/scanCncnDbg.o Test/ScanMngrDbg.o Test/scrDbg.o Test/siteMgrDebug.o Test/smeDebug.o \
			 Test/SoftGeminiDbg.o Test/TWD_Debug.o Test/TwIfDebug.o Test/TxDbg.o

TWDOFILES = TWD/Ctrl/CmdBld.o TWD/Ctrl/CmdBldCfg.o TWD/Ctrl/CmdBldCfgIE.o TWD/Ctrl/CmdBldCmd.o TWD/Ctrl/CmdBldCmdIE.o \
		 	TWD/Ctrl/CmdBldItr.o TWD/Ctrl/CmdBldItrIE.o TWD/Ctrl/CmdQueue.o \
		 	TWD/Data_Service/RxQueue.o TWD/Data_Service/txCtrlBlk.o TWD/Data_Service/txHwQueue.o \
		 	TWD/FW_Transfer/CmdMBox.o TWD/FW_Transfer/eventMbox.o TWD/FW_Transfer/fwDebug.o \
		 	TWD/FW_Transfer/FwEvent.o TWD/FW_Transfer/HwInit.o TWD/FW_Transfer/RxXfer.o \
		 	TWD/FW_Transfer/txResult.o TWD/FW_Transfer/txXfer.o \
		 	TWD/MacServices/MacServices.o TWD/MacServices/MeasurementSrv.o TWD/MacServices/measurementSrvDbgPrint.o \
		 	TWD/MacServices/MeasurementSrvSM.o TWD/MacServices/PowerSrv.o TWD/MacServices/PowerSrvSM.o \
		 	TWD/MacServices/ScanSrv.o TWD/MacServices/ScanSrvSM.o \
		 	TWD/TWDriver/TWDriver.o TWD/TWDriver/TWDriverCtrl.o TWD/TWDriver/TWDriverRadio.o TWD/TWDriver/TWDriverTx.o \
		 	TWD/TwIf/TwIf.o \

TxnOFILES = Txn/SdioBusDrv.o Txn/TxnQueue.o

utilsOFILES = utils/context.o utils/freq.o utils/fsm.o utils/GenSM.o utils/queue.o \
			utils/rate.o utils/report.o utils/timer.o

obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(OFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(AirLinkOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(ApplicationOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(ConnectionOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(Ctrl_InterfaceOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(Data_linkOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(Sta_ManagementOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(TestOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(TWDOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(TxnOFILES)
obj-$(CONFIG_SDIO_TI_WIFI_WGT7310) += $(utilsOFILES)

EXTRA_LDFLAGS += --strip-debug
