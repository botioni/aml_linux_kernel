#ifndef ETH_PINMUX_HEADER_
#define ETH_PINMUX_HEADER_

/*
"RMII_MDIOREG3[10]"
"RMII_MDCREG3[9]"
"RMII_TX_DATA0REG3[8]"
"RMII_TX_DATA1REG3[7]"
"RMII_TX_ENREG3[6]"
"RMII_RX_DATA0REG3[5]"
"RMII_RX_DATA1REG3[4]"
"RMII_RX_CRS_DVREG3[3]"
"RMII_RX_ERRREG3[2]"
Bank0_GPIOC3-C11
*/
#define ETH_BANK0_GPIOC3_C12	0
#define ETH_BANK0_REG1			3
#define ETH_BANK0_REG1_VAL		(0x1ff<<2)
/*
"RMII_TX_ENREG4[25]"
"RMII_RX_ERRREG4[24]"
"RMII_RX_DATA0REG4[23]"
"RMII_RX_DATA1REG4[22]"
"RMII_RX_CRS_DVREG4[21]"
"RMII_CLK50_OUTREG4[20]"
"RMII_MDCREG4[18]"
"RMII_MDIOREG4[17]"
"RMII_TX_DATA0REG4[16]"
"RMII_TX_DATA1REG4[15]"
Bank1_GPIOD2-D11
*/
#define ETH_BANK1_GPIOD2_D11	1
#define ETH_BANK1_REG1			4
#define ETH_BANK1_REG1_VAL		(0x1ff<<15)

/*
"RMII_CLK50_OUTREG7[13]"
"RMII_MDIOREG[10]"
"RMII_MDCREG5[9]"
"RMII_TX_DATA0REG5[8]"
"RMII_TX_DATA1REG5[7]"
"RMII_TX_ENREG5[6]"
"RMII_RX_DATA0REG5[5]"
"RMII_RX_DATA1REG5[4]"
"RMII_RX_CRS_DVREG5[3]"
"RMII_RX_ERRREG5[2]"
"RMII_CLK50_OUTREG5[1]"
*/
#define ETH_BANK2_GPIOD15_D23	2
#define ETH_BANK2_REG1			5
#define ETH_BANK2_REG1_VAL		(0x1ff<<2)


#define ETH_CLK_IN_GPIOC2_REG4_26		0
#define ETH_CLK_IN_GPIOC12_REG3_0		1
#define ETH_CLK_IN_GPIOD7_REG4_19		2
#define ETH_CLK_IN_GPIOD14_REG7_12	3
#define ETH_CLK_IN_GPIOD24_REG5_0		4

#define ETH_CLK_OUT_GPIOC2_REG4_27	5
#define ETH_CLK_OUT_GPIOC12_REG3_1	6
#define ETH_CLK_OUT_GPIOD7_REG4_20	7
#define ETH_CLK_OUT_GPIOD14_REG7_13	8
#define ETH_CLK_OUT_GPIOD24_REG5_1	9

/*
	select clk:
	5,6,7 sata
	4-extern pad
	3-other_pll_clk
	2-ddr_pll_clk
	1-APLL_CLK_OUT_400M
	0----sys_pll_div3 (333~400Mhz)

	clk_freq:50M=50000000
	output_clk:50000000;
	aways,maybe changed for others?
	
*/
#define ETH_CLKSRC_SYS_D3				(0)
#define ETH_CLKSRC_APLL_CLK			(1)
#define ETH_CLKSRC_DDR_CLK				(2)
#define ETH_CLKSRC_OTHER_CLK			(3)
#define ETH_CLKSRC_EXTERN_PAD_CLK		(4)
#define ETH_CLKSRC_SATA_5				(5)
#define ETH_CLKSRC_SATA_6				(6)
#define ETH_CLKSRC_SATA_7				(7)
#define CLK_1M							(1000000)
#define ETH_VALIDE_CLKSRC(clk,out_clk)			((clk%out_clk)==0)
int  eth_clk_set(int selectclk,unsigned long clk_freq,unsigned long out_clk);


#endif
