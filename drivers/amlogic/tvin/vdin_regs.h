/*
 * VDIN register bit-field definition
 * Sorted by the appearing order of registers in am_regs.h.
 *
 * Author: Lin Xu <lin.xu@amlogic.com>
 *
 * Copyright (C) 2010 Amlogic Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#ifndef __VDIN_REGS_H
#define __VDIN_REGS_H

//#define VDIN_SCALE_COEF_IDX                        0x1200
//#define VDIN_SCALE_COEF                            0x1201

//#define VDIN_COM_CTRL0                             0x1202
/* used by other modules,indicates that MPEG input.
0: mpeg source to NR directly,
1: mpeg source pass through here */
#define MPEG_VD_SEL_BIT                 31
#define MPEG_VD_SEL_WID                 1
/* indicates MPEG field ID,written by software.
0: EVEN FIELD 1: ODD FIELD */
#define MPEG_FE_BIT                     30
#define MPEG_FE_WID                     1
#define FORCE_FE_BIT                    29   // for test
#define FORCE_GO_FE_WID                 1    // pulse signal
#define FORCE_GO_LN_BIT                 28   // for test
#define FORCE_GO_LN_WID                 1    // pulse signal
#define MPEG_GO_FE_EN_BIT               27
#define MPEG_GO_FE_EN_WID               1
/* vdin read enable after hold lines counting from delayed Go-field (VS). */
#define HOLD_LN_BIT                     20
#define HOLD_LN_WID                     7
#define DLY_GF_EN_BIT                   19
#define DLY_GF_EN_WID                   1
#define DLY_GF_LNUM_BIT                 12
#define DLY_GF_LNUM_WID                 7    // delay go field lines
/* 00: component0_in 01: component1_in 10: component2_in */
#define C2_OUT_SW_BIT                   10
#define C2_OUT_SW_WID                   2
/* 00: component0_in 01: component1_in 10: component2_in */
#define C1_OUT_SW_BIT                   8
#define C1_OUT_SW_WID                   2
/* 00: component0_in 01: component1_in 10: component2_in */
#define C0_OUT_SW_BIT                   6
#define C0_OUT_SW_WID                   2
/* 0: no data input 1: common data input */
#define COMMON_IN_EN_BIT                4
#define COMMON_IN_EN_WID                1
/* 1: MPEG, 2: 656, 3: TVFE, 4: CDV2, 5: HDMI_Rx,6: DVIN otherwise: NULL */
#define SEL_BIT                         0
#define SEL_WID                         4


//#define VDIN_ACTIVE_MAX_PIX_CNT_STATUS             0x1203
/* ~field_hold & prehsc input active max pixel every line output of window */
#define AMAX_PCNT_BIT                   16
#define AMAX_PCNT_WID                   13
#define AMAX_PCNTS_BIT                  0    // latch by go_field
#define AMAX_PCNTS_WID                  13

//#define VDIN_LCNT_STATUS                           0x1204
/* line count by force_go_line |sel_go_line :output of decimate */
#define GO_LCNT_BIT                     16
#define GO_LCNT_WID                     13
/* line  count prehsc input active max pixel every active line output of window */
#define ALCNT_BIT                       0
#define ALCNT_WID                       13

//#define VDIN_COM_STATUS0                        0x1205
#define LFIFO_BCNT_BIT                  3
#define LFIFO_BCNT_WID                  10   //wren + read -
#define DIRECT_STS_BIT                  2
#define DIRECT_STS_WID                  1    // direct_done_clr_bit & reg_wpluse
#define NR_STS_BIT                      1
#define NR_STS_WID                      1    // nr_done_clr_bit & reg_wpluse
#define FIELD_BIT                       0
#define FIELD_WID                       1

//#define VDIN_COM_STATUS1                        0x1206
#define FIFO4_OVER_BIT                  31
#define FIFO4_OVER_WID                  1
#define ASFIFO4_CNT_BIT                 24
#define ASFIFO4_CNT_WID                 6
#define FIFO3_OVER_BIT                  23
#define FIFO3_OVER_WID                  1
#define ASFIFO3_CNT_BIT                 16
#define ASFIFO3_CNT_WID                 6
#define FIFO2_OVER_BIT                  15
#define FIFO2_OVER_WID                  1
#define ASFIFO2_CNT_BIT                 8
#define ASFIFO2_CNT_WID                 6
#define FIFO1_OVER_BIT                  7
#define FIFO1_OVER_WID                  1
#define ASFIFO1_CNT_BIT                 0
#define ASFIFO1_CNT_WID                 6

//#define VDIN_LCNT_SHADOW_STATUS                 0x1207
#define GO_LCNTS_BIT                    16
#define GO_LCNTS_WID                    13   // latch by go_field
#define ALCNTS_BIT                      0
#define ALCNTS_WID                      13   // latch by go_field

//#define VDIN_ASFIFO_CTRL0                       0x1208
#define ASF2_DE_EN_BIT                  23
#define ASF2_DE_EN_WID                  1
#define ASF2_GFE_EN_BIT                 22
#define ASF2_GFE_EN_WID                 1
#define ASF2_GLN_EN_BIT                 21
#define ASF2_GLN_EN_WID                 1
#define ASF2_NAIN_VS_BIT                20
#define ASF2_NAIN_VS_WID                1
#define ASF2_NAIN_HS_BIT                19
#define ASF2_NAIN_HS_WID                1
#define ASF2_VRST_F_BIT                 18
#define ASF2_VRST_F_WID                 1
#define ASF2_OSTS_CL_BIT                17
#define ASF2_OSTS_CL_WID                1
#define ASF2_RST_BIT                    16
#define ASF2_RST_WID                    1    // write 1 & then 0 to reset
#define ASF1_DE_EN_BIT                  7
#define ASF1_DE_EN_WID                  1
#define ASF1_GFE_EN_BIT                 6
#define ASF1_GFE_EN_WID                 1
#define ASF1_GLN_EN_BIT                 5
#define ASF1_GLN_EN_WID                 1
#define ASF1_NAIN_VS_BIT                4
#define ASF1_NAIN_VS_WID                1
#define ASF1_NAIN_HS_BIT                3
#define ASF1_NAIN_HS_WID                1
#define ASF1_VRST_F_BIT                 2
#define ASF1_VRST_F_WID                 1
#define ASF1_OSTS_CL_BIT                1
#define ASF1_OSTS_CL_WID                1
#define ASF1_RST_BIT                    0
#define ASF1_RST_WID                    1    // write 1 & then 0 to reset

//#define VDIN_ASFIFO_CTRL1                       0x1209
#define ASF4_DE_EN_BIT                  23
#define ASF4_DE_EN_WID                  1
#define ASF4_GFE_EN_BIT                 22
#define ASF4_GFE_EN_WID                 1
#define ASF4_GLN_EN_BIT                 21
#define ASF4_GLN_EN_WID                 1
#define ASF4_NAIN_VS_BIT                20
#define ASF4_NAIN_VS_WID                1
#define ASF4_NAIN_HS_BIT                19
#define ASF4_NAIN_HS_WID                1
#define ASF4_VRST_F_BIT                 18
#define ASF4_VRST_F_WID                 1
#define ASF4_OSTS_CL_BIT                17
#define ASF4_OSTS_CL_WID                1
#define ASF4_RST_BIT                    16
#define ASF4_RST_WID                    1    // write 1 & then 0 to reset
#define ASF3_DE_EN_BIT                  7
#define ASF3_DE_EN_WID                  1
#define ASF3_GFE_EN_BIT                 6
#define ASF3_GFE_EN_WID                 1
#define ASF3_GLN_EN_BIT                 5
#define ASF3_GLN_EN_WID                 1
#define ASF3_NAIN_VS_BIT                4
#define ASF3_NAIN_VS_WID                1
#define ASF3_NAIN_HS_BIT                3
#define ASF3_NAIN_HS_WID                1
#define ASF3_VRST_F_BIT                 2
#define ASF3_VRST_F_WID                 1
#define ASF3_OSTS_CL_BIT                1
#define ASF3_OSTS_CL_WID                1
#define ASF3_RST_BIT                    0
#define ASF3_RST_WID                    1    // write 1 & then 0 to reset

//#define VDIN_WIDTHM1I_WIDTHM1O                  0x120a
#define WIDTHM1I_BIT                    16
#define WIDTHM1I_WID                    13
#define WIDTHM1O_BIT                    0
#define WIDTHM1O_WID                    13

//#define VDIN_SC_MISC_CTRL                       0x120b
#define INIT_PIN_PT_BIT                 8
#define INIT_PIN_PT_WID                 7    // signed value for short line output
#define PRE_HS_EN_BIT                   7
#define PRE_HS_EN_WID                   1    // pre-hscaler: 1/2 coarse scale down
#define HSCALER_EN_BIT                  6
#define HSCALER_EN_WID                  1    // hscaler: fine scale down
#define SLN_OUT_EN_BIT                  5
#define SLN_OUT_EN_WID                  1
/*when decimation timing located in between 2 input pixels, decimate the nearest one*/
#define HSN_EN_BIT                      4
#define HSN_EN_WID                      1
#define PHASE0A_EN_BIT                  3
#define PHASE0A_EN_WID                  1
/* filter pixel buf len (depth), max is 4 in IP design */
#define HSB_LTH_BIT                     0
#define HSB_LTH_WID                     3

//#define VDIN_HSC_PHASE_STEP                     0x120c
#define HSP_STEP_I_BIT                  24
#define HSP_STEP_I_WID                  5
#define HSP_STEP_F_BIT                  0
#define HSP_STEP_F_WID                  24

//#define VDIN_HSC_INI_CTRL                          0x120d
/* repeatedly decimation of pixel #0 of each line? */
#define HSRPT_P0_N_BIT                  29
#define HSRPT_P0_N_WID                  2
/* if rev>rpt_p0+1, then start decimation upon ini_phase? */
#define HSINI_RCV_N_BIT                 24
#define HSINI_RCV_N_WID                 5
/* which one every some pixels is decimated */
#define HSINI_PHASE_BIT                 0
#define HSINI_PHASE_WID                 24


//#define VDIN_MATRIX_CTRL                        0x1210
#define MATRIX_EN_BIT                   0
#define MATRIX_EN_WID                   1    // post conversion matrix

//#define VDIN_MATRIX_COEF00_01                   0x1211
#define MATRIX_C00_BIT                  16
#define MATRIX_C00_WID                  13   // s2.10
#define MATRIX_C01_BIT                  0
#define MATRIX_C01_WID                  13   // s2.10

//#define VDIN_MATRIX_COEF02_10                   0x1212
#define MATRIX_C02_BIT                  16
#define MATRIX_C02_WID                  13   // s2.10
#define MATRIX_C10_BIT                  0
#define MATRIX_C10_WID                  13   // s2.10

//#define VDIN_MATRIX_COEF11_12                   0x1213
#define MATRIX_C11_BIT                  16
#define MATRIX_C11_WID                  13   // s2.10
#define MATRIX_C12_BIT                  0
#define MATRIX_C12_WID                  13   // s2.10

//#define VDIN_MATRIX_COEF20_21                   0x1214
#define MATRIX_C20_BIT                  16
#define MATRIX_C20_WID                  13   // s2.10
#define MATRIX_C21_BIT                  0
#define MATRIX_C21_WID                  13   // s2.10

//#define VDIN_MATRIX_COEF22                      0x1215
#define MATRIX_C22_BIT                  0
#define MATRIX_C22_WID                  13   // s2.10

//#define VDIN_MATRIX_OFFSET0_1                   0x1216
#define MATRIX_OF0_BIT                  16
#define MATRIX_OF0_WID                  11   // s8.2
#define MATRIX_OF1_BIT                  0
#define MATRIX_OF1_WID                  11   // s8.2

//#define VDIN_MATRIX_OFFSET2                     0x1217
#define MATRIX_OF2_BIT                  0
#define MATRIX_OF2_WID                  11   // s8.2

//#define VDIN_MATRIX_PRE_OFFSET0_1               0x1218
#define MATRIX_POF0_BIT                 16
#define MATRIX_POF0_WID                 11   // s8.2
#define MATRIX_POF1_BIT                 0
#define MATRIX_POF1_WID                 11   // s8.2

//#define VDIN_MATRIX_PRE_OFFSET2                 0x1219
#define MATRIX_POF2_BIT                 0
#define MATRIX_POF2_WID                 11   // s8.2

//#define VDIN_LFIFO_CTRL                         0x121a
#define LFIFOB_SIZE_BIT                 0
#define LFIFOB_SIZE_WID                 12

//#define VDIN_COM_GCLK_CTRL                      0x121b
#define CGCTRL_BB_BIT                   14
#define CGCTRL_BB_WID                   2    // 00: auto, 01: off, 1x: on
#define CGCTRL_HIST_BIT                 12
#define CGCTRL_HIST_WID                 2    // 00: auto, 01: off, 1x: on
#define CGCTRL_LF_BIT                   10
#define CGCTRL_LF_WID                   2    // 00: auto, 01: off, 1x: on
#define CGCTRL_M_BIT                    8
#define CGCTRL_M_WID                    2    // 00: auto, 01: off, 1x: on
#define CGCTRL_HS_BIT                   6
#define CGCTRL_HS_WID                   2    // 00: auto, 01: off, 1x: on
#define CGCTRL_PHS_BIT                  4
#define CGCTRL_PHS_WID                  2    // 00: auto, 01: off, 1x: on
#define CGCTRL_TOP_BIT                  2
#define CGCTRL_TOP_WID                  2    // 00: auto, 01: off, 1x: on
/* Caution !!! never turn it off, otherwise no way to wake up VDIN unless power reset  */
#define CGCTRL_REG_BIT                  0
#define CGCTRL_REG_WID                  1    //  0: auto,  1: off. Caution !!!

//#define VDIN_WR_CTRL                            0x1220
#define WR_OUT_CTRL_BIT                 24
#define WR_OUT_CTRL_WID                 8    //directly send out
#define FRA_RST_EN_BIT                  23
#define FRA_RST_EN_WID                  1
#define LF_RST_EN_BIT                   22   // reset LFIFO on VS (Go_field)
#define LF_RST_EN_WID                   1
#define DD_CLR_BIT_BIT                  21   // used by other modules
#define DD_CLR_BIT_WID                  1
#define NRD_CLR_BIT_BIT                 20   // used by other modules
#define NRD_CLR_BIT_WID                 1
#define WR_FORMAT_BIT                   12
#define WR_FORMAT_WID                   1    // 0: 422, 1: 444 directly send out
/* vdin_wr_canvas = vdin_wr_canvas_dbuf_en ? wr_canvas_shadow :wr_canvas;  */
#define WR_CBUF_EN_BIT                  11   //shadow is latch by go_field
#define WR_CBUF_EN_WID                  1
#define WR_REQ_UG_BIT                   9
#define WR_REQ_UG_WID                   1    // directly send out
#define WR_REQ_EN_BIT                   8
#define WR_REQ_EN_WID                   1    // directly send out
#define WR_CANVAS_BIT                   0
#define WR_CANVAS_WID                   8

//#define VDIN_WR_H_START_END                        0x1221
#define WR_HSTART_BIT                   16
#define WR_HSTART_WID                   12   // directly send out
#define WR_HEND_BIT                     0
#define WR_HEND_WID                     12   // directly send out

//#define VDIN_WR_V_START_END                        0x1222
#define WR_VSTART_BIT                   16
#define WR_VSTART_WID                   12   // directly send out
#define WR_VEND_BIT                     0
#define WR_VEND_WID                     12   // directly send out

//#define VDIN_HIST_CTRL                             0x1230
/* the total pixels = VDIN_HISTXX*(2^(VDIN_HIST_POW+3)) */
#define HIST_POW_BIT                    5
#define HIST_POW_WID                    2
/* Histgram source: 00: MAT_OUT, 01: HSC_OUT, 1X: PREHSC_IN */
#define HIST_MUX_BIT                    2
#define HIST_MUX_WID                    2
/* Histgram range: 0: full picture, 1: histgram window defined by VDIN_HIST_H_START_END & VDIN_HIST_V_START_END */
#define HIST_WIN_EN_BIT                 1
#define HIST_WIN_EN_WID                 1
/* Histgram readback: 0: disable, 1: enable */
#define HIST_RD_EN_BIT                  0
#define HIST_RD_EN_WID                  1

//#define VDIN_HIST_H_START_END                   0x1231
#define HIST_HSTART_BIT                 16
#define HIST_HSTART_WID                 13
#define HIST_HEND_BIT                   0
#define HIST_HEND_WID                   13

//#define VDIN_HIST_V_START_END                   0x1232
#define HIST_VSTART_BIT                 16
#define HIST_VSTART_WID                 13
#define HIST_VEND_BIT                   0
#define HIST_VEND_WID                   13

//#define VDIN_HIST_MAX_MIN                       0x1233
#define HIST_MAX_BIT                    8
#define HIST_MAX_WID                    8
#define HIST_MIN_BIT                    0
#define HIST_MIN_WID                    8

//#define VDIN_HIST_SPL_VAL                       0x1234
#define HIST_LSUM_BIT                   0
#define HIST_LSUM_WID                   32

//#define VDIN_HIST_SPL_PIX_CNT                   0x1235
#define HIST_PCNT_BIT                   0
#define HIST_PCNT_WID                   22   // the total calculated pixels

//#define VDIN_HIST_CHROMA_SUM                    0x1236
#define HIST_CSUM_BIT                   0
#define HIST_CSUM_WID                   32   // the total chroma value


//#define VDIN_DNLP_HIST00                        0x1237
#define HIST_OB_01_BIT                  16
#define HIST_OB_01_WID                  16
#define HIST_OB_00_BIT                  0
#define HIST_OB_00_WID                  16

//#define VDIN_DNLP_HIST01                        0x1238
#define HIST_OB_03_BIT                  16
#define HIST_OB_03_WID                  16
#define HIST_OB_02_BIT                  0
#define HIST_OB_02_WID                  16

//#define VDIN_DNLP_HIST02                        0x1239
#define HIST_OB_05_BIT                  16
#define HIST_OB_05_WID                  16
#define HIST_OB_04_BIT                  0
#define HIST_OB_04_WID                  16

//#define VDIN_DNLP_HIST03                        0x123a
#define HIST_OB_07_BIT                  16
#define HIST_OB_07_WID                  16
#define HIST_OB_06_BIT                  0
#define HIST_OB_06_WID                  16

//#define VDIN_DNLP_HIST04                        0x123b
#define HIST_OB_09_BIT                  16
#define HIST_OB_09_WID                  16
#define HIST_OB_08_BIT                  0
#define HIST_OB_08_WID                  16

//#define VDIN_DNLP_HIST05                        0x123c
#define HIST_OB_11_BIT                  16
#define HIST_OB_11_WID                  16
#define HIST_OB_10_BIT                  0
#define HIST_OB_10_WID                  16

//#define VDIN_DNLP_HIST06                        0x123d
#define HIST_OB_13_BIT                  16
#define HIST_OB_13_WID                  16
#define HIST_OB_12_BIT                  0
#define HIST_OB_12_WID                  16

//#define VDIN_DNLP_HIST07                        0x123e
#define HIST_OB_15_BIT                  16
#define HIST_OB_15_WID                  16
#define HIST_OB_14_BIT                  0
#define HIST_OB_14_WID                  16

//#define VDIN_DNLP_HIST08                        0x123f
#define HIST_OB_17_BIT                  16
#define HIST_OB_17_WID                  16
#define HIST_OB_16_BIT                  0
#define HIST_OB_16_WID                  16

//#define VDIN_DNLP_HIST09                        0x1240
#define HIST_OB_19_BIT                  16
#define HIST_OB_19_WID                  16
#define HIST_OB_18_BIT                  0
#define HIST_OB_18_WID                  16

//#define VDIN_DNLP_HIST10                        0x1241
#define HIST_OB_21_BIT                  16
#define HIST_OB_21_WID                  16
#define HIST_OB_20_BIT                  0
#define HIST_OB_20_WID                  16

//#define VDIN_DNLP_HIST11                        0x1242
#define HIST_OB_23_BIT                  16
#define HIST_OB_23_WID                  16
#define HIST_OB_22_BIT                  0
#define HIST_OB_22_WID                  16

//#define VDIN_DNLP_HIST12                        0x1243
#define HIST_OB_25_BIT                  16
#define HIST_OB_25_WID                  16
#define HIST_OB_24_BIT                  0
#define HIST_OB_24_WID                  16

//#define VDIN_DNLP_HIST13                        0x1244
#define HIST_OB_27_BIT                  16
#define HIST_OB_27_WID                  16
#define HIST_OB_26_BIT                  0
#define HIST_OB_26_WID                  16

//#define VDIN_DNLP_HIST14                        0x1245
#define HIST_OB_29_BIT                  16
#define HIST_OB_29_WID                  16
#define HIST_OB_28_BIT                  0
#define HIST_OB_28_WID                  16

//#define VDIN_DNLP_HIST15                        0x1246
#define HIST_OB_31_BIT                  16
#define HIST_OB_31_WID                  16
#define HIST_OB_30_BIT                  0
#define HIST_OB_30_WID                  16

//#define VDIN_DNLP_HIST16                        0x1247
#define HIST_OB_33_BIT                  16
#define HIST_OB_33_WID                  16
#define HIST_OB_32_BIT                  0
#define HIST_OB_32_WID                  16

//#define VDIN_DNLP_HIST17                        0x1248
#define HIST_OB_35_BIT                  16
#define HIST_OB_35_WID                  16
#define HIST_OB_34_BIT                  0
#define HIST_OB_34_WID                  16

//#define VDIN_DNLP_HIST18                        0x1249
#define HIST_OB_37_BIT                  16
#define HIST_OB_37_WID                  16
#define HIST_OB_36_BIT                  0
#define HIST_OB_36_WID                  16

//#define VDIN_DNLP_HIST19                        0x124a
#define HIST_OB_39_BIT                  16
#define HIST_OB_39_WID                  16
#define HIST_OB_38_BIT                  0
#define HIST_OB_38_WID                  16

//#define VDIN_DNLP_HIST20                        0x124b
#define HIST_OB_41_BIT                  16
#define HIST_OB_41_WID                  16
#define HIST_OB_40_BIT                  0
#define HIST_OB_40_WID                  16

//#define VDIN_DNLP_HIST21                        0x124c
#define HIST_OB_43_BIT                  16
#define HIST_OB_43_WID                  16
#define HIST_OB_42_BIT                  0
#define HIST_OB_42_WID                  16

//#define VDIN_DNLP_HIST22                        0x124d
#define HIST_OB_45_BIT                  16
#define HIST_OB_45_WID                  16
#define HIST_OB_44_BIT                  0
#define HIST_OB_44_WID                  16

//#define VDIN_DNLP_HIST23                        0x124e
#define HIST_OB_47_BIT                  16
#define HIST_OB_47_WID                  16
#define HIST_OB_46_BIT                  0
#define HIST_OB_46_WID                  16

//#define VDIN_DNLP_HIST24                        0x124f
#define HIST_OB_49_BIT                  16
#define HIST_OB_49_WID                  16
#define HIST_OB_48_BIT                  0
#define HIST_OB_48_WID                  16

//#define VDIN_DNLP_HIST25                        0x1250
#define HIST_OB_51_BIT                  16
#define HIST_OB_51_WID                  16
#define HIST_OB_50_BIT                  0
#define HIST_OB_50_WID                  16

//#define VDIN_DNLP_HIST26                        0x1251
#define HIST_OB_53_BIT                  16
#define HIST_OB_53_WID                  16
#define HIST_OB_52_BIT                  0
#define HIST_OB_52_WID                  16

//#define VDIN_DNLP_HIST27                        0x1252
#define HIST_OB_55_BIT                  16
#define HIST_OB_55_WID                  16
#define HIST_OB_54_BIT                  0
#define HIST_OB_54_WID                  16

//#define VDIN_DNLP_HIST28                        0x1253
#define HIST_OB_57_BIT                  16
#define HIST_OB_57_WID                  16
#define HIST_OB_56_BIT                  0
#define HIST_OB_56_WID                  16

//#define VDIN_DNLP_HIST29                        0x1254
#define HIST_OB_59_BIT                  16
#define HIST_OB_59_WID                  16
#define HIST_OB_58_BIT                  0
#define HIST_OB_58_WID                  16

//#define VDIN_DNLP_HIST30                        0x1255
#define HIST_OB_61_BIT                  16
#define HIST_OB_61_WID                  16
#define HIST_OB_60_BIT                  0
#define HIST_OB_60_WID                  16

//#define VDIN_DNLP_HIST31                        0x1256
#define HIST_OB_63_BIT                  16
#define HIST_OB_63_WID                  16
#define HIST_OB_62_BIT                  0
#define HIST_OB_62_WID                  16

//#define VDIN_BLKBAR_CTRL0                       0x1260
#define BB_BLEVEL_BIT                   22
#define BB_BLEVEL_WID                   10   // threshold to judge a black point
#define BB_HWID_BIT                     8
#define BB_HWID_WID                     13   // left and right region width
/* select yin or uin or vin to be the valid input */
#define BB_COMP_SEL_BIT                 5
#define BB_COMP_SEL_WID                 3
/* sw statistic of black pixels of each block,
1: search once, 0: search continuously till the exact edge */
#define BB_SW_ST_EN_BIT                 4
#define BB_SW_ST_EN_WID                 1
#define BB_DRST_N_BIT                   3
#define BB_DRST_N_WID                   1    // write 0 & then 1 to reset
/* 0: matrix_dout, 1: hscaler_dout,
2/3: pre-hscaler_din blkbar_din_srdy blkbar_din_rrdy  enable */
#define BB_DIN_SEL_BIT                  1
#define BB_DIN_SEL_WID                  2
/* blkbar_din_srdy blkbar_din_rrdy  enable */
#define BB_DTOP_EN_BIT                  0
#define BB_DTOP_EN_WID                  1

//#define VDIN_BLKBAR_H_START_END                    0x1261
#define BB_HSTART_BIT                   16
#define BB_HSTART_WID                   13   // Left region start
#define BB_HEND_BIT                     0
#define BB_HEND_WID                     13   // Right region end

//#define VDIN_BLKBAR_V_START_END                    0x1262
#define BB_VSTART_BIT                   16
#define BB_VSTART_WID                   13
#define BB_VEND_BIT                     0
#define BB_VEND_WID                     13

//#define VDIN_BLKBAR_CNT_THRESHOLD                  0x1263
/* black pixel number threshold to judge whether a block is totally black */
#define BB_CNT_TH_BIT                   0
#define BB_CNT_TH_WID                   20

//#define VDIN_BLKBAR_ROW_TH1_TH2                    0x1264
/* white pixel number threshold of black line on top */
#define BB_ROW_TH1_BIT                  16
#define BB_ROW_TH1_WID                  13
/* white pixel number threshold of black line on bottom */
#define BB_ROW_TH2_BIT                  0
#define BB_ROW_TH2_WID                  13

//#define VDIN_BLKBAR_IND_LEFT_START_END             0x1265
#define BB_LRG_HS_BIT                   16
#define BB_LRG_HS_WID                   13
#define BB_LRG_HE_BIT                   0
#define BB_LRG_HE_WID                   13

//#define VDIN_BLKBAR_IND_RIGHT_START_END            0x1266
#define BB_RRG_HS_BIT                   16
#define BB_RRG_HS_WID                   13
#define BB_RRG_HE_BIT                   0
#define BB_RRG_HE_WID                   13

//#define VDIN_BLKBAR_IND_LEFT1_CNT                  0x1267
/* Black pixels at left part of the left region */
#define BB_L1_CNT_BIT                   0
#define BB_L1_CNT_WID                   20

//#define VDIN_BLKBAR_IND_LEFT2_CNT                  0x1268
/* Black pixels at right part of the left region */
#define BB_L2_CNT_BIT                    0
#define BB_L2_CNT_WID                    20

//#define VDIN_BLKBAR_IND_RIGHT1_CNT                 0x1269
/* Black pixels at right part of the left region */
#define BB_R1_CNT_BIT                   0
#define BB_R1_CNT_WID                   20

//#define VDIN_BLKBAR_IND_RIGHT2_CNT                 0x126a
/* Black pixels at right part of the right region */
#define BB_R2_CNT_BIT                   0
#define BB_R2_CNT_WID                   20

//#define VDIN_BLKBAR_STATUS0                        0x126b
/* LEFT/RIGHT Black Bar detection done */
#define BB_DET_DONE_BIT                 29
#define BB_DET_DONE_WID                 1
#define BB_TPOS_BIT                     16
#define BB_TPOS_WID                     13
#define BB_BPOS_BIT                     0
#define BB_BPOS_WID                     13

//#define VDIN_BLKBAR_STATUS1                        0x126c
#define BB_LPOS_BIT                     16
#define BB_LPOS_WID                     13
#define BB_RPOS_BIT                     0
#define BB_RPOS_WID                     13

#endif

