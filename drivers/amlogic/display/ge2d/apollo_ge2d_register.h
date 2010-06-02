// synopsys translate_off
#ifdef GE2D_REGS_H
#else
#define GE2D_REGS_H
// synopsys translate_on


//===========================================================================
// GE2D Registers    0x8a0 - 0x8ff
//===========================================================================
#define GE2D_ADDR_START					0x8a0    
#define GE2D_ADDR_END					0x8ff

//Bit 31, destination bytemask only if destination bitmask is enable
//Bit 30, destination bitmask enable
//Bit 29, source2 key  enable
//Bit 28, source2 key  mode, 0: mask data when match, 1: mask data when unmatch
//Bit 27, source1 key  enable
//Bit 26, source1 key  mode, 0: mask data when match, 1: mask data when unmatch
//Bit 25:24, dst 8bit mode component selection, 
//            00: select Y(R), 01: Cb(G), 10: Cr(B), 11: Alpha
//Bit 23 dst clip mode, 0: write inside clip window, 1: write outside clip window
//Bit 21:20  dst format,  00: 8bit, 01:reserved, 10:24bit(RGB/YCbCr) 11: 32bit(RGBA/YCBCRA)
//Bit 18:17  src2 format, 00: 8bit, 01:reserved, 10:24bit(RGB/YCbCr) 11: 32bit(RGBA/YCBCRA)
//Bit 16:15, src2 8bit mode component selection, 
//            00: select Y(R), 01: Cb(G), 10: Cr(B), 11: Alpha
//Bit 14     src2 fill mode, 0: repeat data, 1: fill default color
//Bit 13:12  src2 picture struct, 00: frame, 10: even, 11: odd
//Bit 11     src1 x direction yc ration, 0: 1:1, 1: 2:1
//Bit 10     src1 y direction yc ration, 0: 1:1, 1: 2:1
//Bit 8:7    src1 format, 00: 8bit, 01:4:2:2 store togather, 10:24bit(RGB/YCbCr) 11: 32bit(RGBA/YCBCRA)
//Bit 6:5,   src1  8bit mode component selection, 
//            00: select Y(R), 01: Cb(G), 10: Cr(B), 11: Alpha
//Bit 4      src1 fill mode, 0: repeat data, 1: fill default color
//Bit 3      src1 lookup table enable
//Bit 2:1    src1 picture struct, 00: frame, 10: even, 11: odd
//Bit 0      src1 separate buffer enable
#define GE2D_GEN_CTRL0                  0x8a0  //32'h0  UPPER DISCRIPTION.

//Bit 31, soft rst
//bit 25:24, interrupt control, if bit[0] true, generate interrupt when one command done,
//                              if bit[1] true, generate interrupt when ge2d change from busy to not busy
//Bit 23:22 src2 burst size control
//Bit 21:16 src1 burst size control, 5:4, yfifo, 3:2, cbfifo, 1:0, crfifo
//          each 2bit, 00: 24 64bitword, 01: 32 64bitword, 10: 48 64bitwords, 11: 64 64bitwords  
//Bit 15:14, dst picture struct, 00: frame, 10:top, 11: bottom
//Bit 13:12, bit 13 if true, force read src1, bit 12 if true, force read src2
//Bit 10, src1 request urgent enable
//Bit 9,  src2 request urgent enable
//Bit 8,  dst request urgent enable
//Bit 7:0 src1 global alpha
#define GE2D_GEN_CTRL1                  0x8a1   //32'h00ff0000

//Bit 9     if true, all src2 data use default color
//Bit 8     if true, all src1 data use default color
//Bit 7     if true, dst x/y swap 
//Bit 6     if true, dst x direction reversely read
//Bit 5     if true, dst y direction reversely read
//Bit 4     if true, src2 x direction reversely read
//Bit 3     if true, src2 y direction reversely read
//Bit 2     if true, src1 x direction reversely read
//Bit 1     if true, src1 y direction reversely read
//Bit 0     cmd write
#define GE2D_CMD_CTRL                   0x8a3 //'h0

//Read only
//Bit 16:7  ge2d_dp status, for debug only
//Bit 6     read src2 cmd ready
//Bit 5     read src2 cmd ready
//Bit 4     pre dpcmd ready
//Bit 3     ge2d dpcmd ready
//Bit 2     ge2d buffer command valid
//Bit 1     ge2d current command valid
//Bit 0     ge2d busy
#define GE2D_STATUS0                    0x8a4 
//
//Read only
// Bit 29:16 ge2d_dst_status, for debug only
// Bit    15 ge2d_rd_src2 core.fifo_empty
// Bit    14 ge2d_rd_src2 core.fifo_overflow
// Bit 13:12 ge2d_rd_src2 core.req_st
// Bit    11 ge2d_rd_src2 cmd_if.cmd_err, true if cmd_format=1
// Bit    10 ge2d_rd_src2 cmd_if.cmd_st, 0=IDLE state, 1=BUSY state
// Bit     9 ge2d_rd_src1 luma_core(chroma_core).fifo_empty
// Bit     8 ge2d_rd_src1 luma_core(chroma_core).fifo_overflow
// Bit  7: 6 ge2d_rd_src1 chroma_core.req_st_cr
// Bit  5: 4 ge2d_rd_src1 chroma_core.req_st_cb
// Bit  3: 2 ge2d_rd_src1 luma_core.req_st_y
// Bit     1 ge2d_rd_src1 cmd_if.stat_read_window_err, 1=reading/clipping window setting exceed limit
// Bit     0 ge2d_rd_src1 cmd_if.cmd_st, 0=IDLE state, 1=BUSY state
#define GE2D_STATUS1                    0x8a5 

//SRC1 default clolor
//{Y,Cb,Cr,A}/{R,G,B,A}
#define GE2D_SRC1_DEF_COLOR             0x8a6   //32'h00808000 

//Bit 31, SRC1 clip x start extra, if true, one more data is read for chroma
//Bit 28:16, SRC1 clip x start
//Bit 15, SRC1 clip x end extra, if true, one more data is read for chroma
//Bit 12:0, SRC1 clip x end
#define GE2D_SRC1_CLIPX_START_END       0x8a7  //32'h0000_1fff 

//Bit 31, SRC1 clip y start extra, if true, one more data is read for chroma
//Bit 28:16, SRC1 clip y start
//Bit 15, SRC1 clip y end extra, if true, one more data is read for chroma
//Bit 12:0, SRC1 clip y end
#define GE2D_SRC1_CLIPY_START_END       0x8a8  //32'h0000_1fff 

//Bit 30:24, SRC1 canvas address0
//Bit 22:16, SRC1 canvas address1
//Bit 14:8, SRC1 canvas address2
#define GE2D_SRC1_CANVAS                0x8a9 //'h0 

//Bit 31, SRC1 x start extra bit1, if true, one more chroma data is read for x even start chroma data when y/c ratio = 2
//             or x even/odd start chroma extra data when y/c ratio = 1
//Bit 30, SRC1 x start extra bit0, if true, one more chroma data is read for x odd start chroma data when y/c ratio = 2
//Bit 29:16, SRC1 x start, signed data
//Bit 15, SRC1 x end extra bit1, if true, one more chroma data is read for x odd end chroma data when y/c ratio = 2
//             or x even/odd end chroma extra data when y/c ratio = 1
//Bit 14, SRC1 x end extra bit0, if true, one more chroma data is read for x even end chroma data when y/c ratio = 2
//Bit 13:0, SRC1 x end, signed data
#define GE2D_SRC1_X_START_END           0x8aa //'h0 

//Bit 31, SRC1 y start extra, if true, one more chroma data is read for y even start chroma data when y/c ratio = 2
//             or y even/odd start chroma extra data when y/c ratio = 1
//Bit 30, SRC1 y start extra, if true, one more chroma data is read for x odd start chroma data when y/c ratio = 2
//Bit 29:16, SRC1 y start
//Bit 15, SRC1 y end extra bit1, if true, one more chroma data is read for y odd end chroma data when y/c ratio = 2
//             or y even/odd end chroma extra data when y/c ratio = 1
//Bit 14, SRC1 y end extra bit0, if true, one more chroma data is read for y even end chroma data when y/c ratio = 2
//Bit 13:0, SRC1 y end
#define GE2D_SRC1_Y_START_END           0x8ab  //'h0 

// Bit 31: 9 Reserved
// Bit     8 RW, 0 = Write LUT, 1 = Read LUT
// Bit  7: 0 RW, lut_addr
#define GE2D_SRC1_LUT_ADDR              0x8ac

// Bit 31:24 RW, Y or R
// Bit 23:16 RW, Cb or G
// Bit 15: 8 RW, Cr or B
// Bit  7: 0 RW, Alpha
#define GE2D_SRC1_LUT_DAT               0x8ad

//Bit 19, if true, horizontal formatter using repeat to get the pixel, otherwise using interpolation
//Bit 18, horizontal formatter en
//Bit 17, if true, vertical formatter using repeat to get the pixel, otherwise using interpolation
//Bit 16, vertical formatter en
//Bit 15:8 X direction chroma phase,  
//          [7:4] for x direction even start/end chroma phase when y/c ratio = 2
//                or start/end even/odd chroma phase  when y/c ratio = 1 
//          [3:0] for x direction odd start/end chroma phase only when y/c ration = 2
//Bit 7:0  Y direction chroma phase, 
//          [7:4] for y direction even start/end chroma phase when y/c ratio = 2
//          or start/end even/odd chroma phase  when y/c ratio = 1 
//          [3:0] for y direction odd start/end chroma phase only when y/c ration = 2 
#define GE2D_SRC1_FMT_CTRL              0x8ae  //'h0

//SRC2 default clolor
//{Y,Cb,Cr,A}/{R,G,B,A}
#define GE2D_SRC2_DEF_COLOR             0x8af  //32'h00808000 

//Bit 28:16, SRC2 clip x start
//Bit 12:0, SRC2 clip x end
#define GE2D_SRC2_CLIPX_START_END       0x8b0  //32'h0000_1fff 

//Bit 28:16, SRC2 clip y start
//Bit 12:0, SRC2 clip y end
#define GE2D_SRC2_CLIPY_START_END       0x8b1  //32'h0000_1fff 

//Bit 28:16, SRC2 x start
//Bit 12:0, SRC2 x end
#define GE2D_SRC2_X_START_END           0x8b2  //'h0 

//Bit 28:16, SRC2 y start
//Bit 12:0, SRC2 y end
#define GE2D_SRC2_Y_START_END           0x8b3  //'h0 

//Bit 28:16, DST clip x start
//Bit 12:0, DST clip x end
#define GE2D_DST_CLIPX_START_END        0x8b4  //32'h0000_1fff 
//
//Bit 28:16, DST clip y start
//Bit 12:0, DST clip y end
#define GE2D_DST_CLIPY_START_END        0x8b5  //32'h0000_1fff

//Bit 28:16, DST x start
//Bit 12:0, DST x end
#define GE2D_DST_X_START_END            0x8b6 //'h0 
//
//Bit 28:16, DST y start
//Bit 12:0, DST y end
#define GE2D_DST_Y_START_END            0x8b7 //'h0 

//Bit 14:8 SRC2 canvas address
//Bit 6:0 DST canvas address
#define GE2D_SRC2_DST_CANVAS            0x8b8 //'h0

//vertical scaler phase step
//Bit 28:0,  5.24 format
#define GE2D_VSC_START_PHASE_STEP       0x8b9 //29'h01000000 

//phase slope 
//Bit 24:0, bit 24 signed bit
#define GE2D_VSC_PHASE_SLOPE            0x8ba //'h0 

//Bit 30:29, vertical repeat line0 number 
//Bit 23:0, vertical scaler initial phase
#define GE2D_VSC_INI_CTRL               0x8bb //'h0

//horizontal scaler phase step
//Bit 28:0,  5.24 format
#define GE2D_HSC_START_PHASE_STEP       0x8bc //29'h01000000 

//phase slope 
//Bit 24:0, bit 24 signed bit
#define GE2D_HSC_PHASE_SLOPE            0x8bd //'h0

//Bit 30:29, horizontal repeat line0 number 
//Bit 23:0, horizontal scaler initial phase
#define GE2D_HSC_INI_CTRL               0x8be //'h0

//Bit 31:24, advance number in this round, if horizontal scaler is working on dividing mode
//Bit 23:0, horizontal scaler advance phase in this round, if horizontal scaler is working on dividing mode 
#define GE2D_HSC_ADV_CTRL               0x8bf //'h0

//Bit 30, vertical nearest mode enable, must set vt_bank_length = 4
//Bit 29, horizontal nearest mode enable, must set hz_bank_length = 4 
//Bit 28, horizontal scaler dividing mode enable
//Bit 27:15, horizontal dividing length, if bit 25 is enable
//Bit 14, pre horizontal scaler enable 
//Bit 13, pre vertical scale enable
//Bit 12, vertical scale enable
//Bit 11, horizontal scaler enable
//Bit 9, if true, treat horizontal repeat line number(GE2D_HSC_INI_CTRL bit 30:29) as repeating line, 
//        otherwise using treat horizontal repeat line number as minus line number. 
//Bit 8, if true, treat vertical repeat line number(GE2D_VSC_INI_CTRL bit 30:29) as repeating line, 
//        otherwise using treat vertical repeat line number as minus line number. 
//Bit 7, if true, always use phase0 in vertical scaler
//Bit 6:4, vertical scaler bank length
//Bit 3, if true, always use phase0 in horizontal scaler
//Bit 2:0, horizontal scaler bank length
#define GE2D_SC_MISC_CTRL               0x8c0 //'h00000044

//Read only
//vertical scaler next round integer pixel pointer, signed data
//Bit 13:0
#define GE2D_VSC_NRND_POINT             0x8c1

//Read only
//vertical scaler next round phase
//bit 23:0
#define GE2D_VSC_NRND_PHASE             0x8c2 

//Read only
//horizontal scaler next round integer pixel pointer, signed data
//Bit 13:0
#define GE2D_HSC_NRND_POINT             0x8c3 

//Read only
//horizontal scaler next round phase
//bit 23:0
#define GE2D_HSC_NRND_PHASE             0x8c4 

//Bit 28:16 coef00
//Bit 12:0  coef01
#define GE2D_MATRIX_COEF00_01           0x8c6     //'h0 

//Bit 28:16 coef02
//Bit 12:0  coef10
#define GE2D_MATRIX_COEF02_10           0x8c7     //'h0 

//Bit 28:16 coef11
//Bit 12:0  coef12
#define GE2D_MATRIX_COEF11_12           0x8c8     //'h0

//Bit 28:16 coef20
//Bit 12:0  coef21
#define GE2D_MATRIX_COEF20_21           0x8c9     //'h0 

//Bit 28:16 coef22
//Bit 7    input y/cb/cr saturation enable
//Bit 6:4  minus 16 control for 3 components, bit 6 for Y, bit5 for Cb, bit4 for Cr
//Bit 3:1  sign control for 3 components, bit3 for Y, bit2 for Cb, bit1 for Cr    
//Bit 0    conversion matrix enable
#define GE2D_MATRIX_COEF22_CTRL         0x8ca     //'h0 


//Bit 28:20, offset0
//Bit 18:10, offset1 
//Bit 8:0,   offset2
#define GE2D_MATRIX_OFFSET              0x8cb     //'h0

//Bit 26:25, SRC1 color multiplier alpha selection
//           if 00, Cs = Csr
//           if 01, Cs = Csr * Asr * Ag (if source is not premultiplied)
//           if 10, Cs = Csr * Ag (if source is premultipied)
//Bit 24    SRC2 color multiplier alpha selection 
//          if 0, no multiplier, Cd = Cdr,  otherwise, Cd = Cdr * Ad.   
//Bit 22:12 ALU color operation
//          bit10:8 Blending Mode Parameter
//            3'b000: ADD               Cs*Fs + Cd*Fd
//            3'b001: SUBTRACT          Cs*Fs - Cd*Fd
//            3'b010: REVERSE SUBTRACT  Cd*Fd - Cs*Fs
//            3'b011: MIN               min(Cs*Fs, Cd*Fd)
//            3'b100: MAX               max(Cs*Fs, Cd*Fd)
//            3'b101: LOGIC OP          Cs op Cd
//          bit7:4 Source Color Blending Factor CFs
//            4'b0000: ZERO                        0
//            4'b0001: ONE                         1
//            4'b0010: SRC_COLOR                   Cs(RGBs)
//            4'b0011: ONE_MINUS_SRC_COLOR         1 - Cs(RGBs)
//            4'b0100: DST_COLOR                   Cd(RGBd)
//            4'b0101: ONE_MINUS_DST_COLOR         1 - Cd(RGBd)
//            4'b0110: SRC_ALPHA                   As
//            4'b0111: ONE_MINUS_SRC_ALPHA         1 - As
//            4'b1000: DST_ALPHA                   Ad
//            4'b1001: ONE_MINUS_DST_ALPHA         1 - Ad
//            4'b1010: CONST_COLOR                 Cc(RGBc)
//            4'b1011: ONE_MINUS_CONST_COLOR       1 - Cc(RGBc)
//            4'b1100: CONST_ALPHA                 Ac
//            4'b1101: ONE_MINUS_CONST_ALPHA       1 - Ac
//            4'b1110: SRC_ALPHA_SATURATE          min(As,1-Ad)
//          bit3:0 dest Color Blending Factor CFd, when bit10:8 != LOGIC OP
//            4'b0000: ZERO                        0
//            4'b0001: ONE                         1
//            4'b0010: SRC_COLOR                   Cs(RGBs)
//            4'b0011: ONE_MINUS_SRC_COLOR         1 - Cs(RGBs)
//            4'b0100: DST_COLOR                   Cd(RGBd)
//            4'b0101: ONE_MINUS_DST_COLOR         1 - Cd(RGBd)
//            4'b0110: SRC_ALPHA                   As
//            4'b0111: ONE_MINUS_SRC_ALPHA         1 - As
//            4'b1000: DST_ALPHA                   Ad
//            4'b1001: ONE_MINUS_DST_ALPHA         1 - Ad
//            4'b1010: CONST_COLOR                 Cc(RGBc)
//            4'b1011: ONE_MINUS_CONST_COLOR       1 - Cc(RGBc)
//            4'b1100: CONST_ALPHA                 Ac
//            4'b1101: ONE_MINUS_CONST_ALPHA       1 - Ac
//            4'b1110: SRC_ALPHA_SATURATE          min(As,1-Ad)
//          bit3:0 logic operations, when bit10:8 == LOGIC OP
//            4'b0000: CLEAR                       0
//            4'b0001: COPY                        s
//            4'b0010: NOOP                        d
//            4'b0011: SET                         1
//            4'b0100: COPY_INVERT                 ~s
//            4'b0101: INVERT                      ~d
//            4'b0110: AND_REVERSE                 s & ~d
//            4'b0111: OR_REVERSE                  s | ~d
//            4'b1000: AND                         s & d
//            4'b1001: OR                          s | d
//            4'b1010: NAND                        ~(s & d)
//            4'b1011: NOR                         ~(s | d)
//            4'b1100: XOR                         s ^ d
//            4'b1101: EQUIV                       ~(s ^ d)
//            4'b1110: AND_INVERTED                ~s & d
//            4'b1111: OR_INVERTED                 ~s | d
//Bit 10:0  ALU alpha operation
//            bit10:8 Blending Equation Math Operation
//              3'b000: ADD               As*Fs + Ad*Fd
//              3'b001: SUBTRACT          As*Fs - Ad*Fd
//              3'b010: REVERSE SUBTRACT  Ad*Fd - As*Fs
//              3'b011: MIN               min(As*Fs, Ad*Fd)
//              3'b100: MAX               max(As*Fs, Ad*Fd)
//              3'b101: LOGIC OP          As op Ad
//            bit7:4 Source alpha Blending Factor AFs
//              4'b0000                       0
//              4'b0001                       1
//              4'b0010                       As
//              4'b0011                       1 - As
//              4'b0100                       Ad
//              4'b0101                       1 - Ad
//              4'b0110                       Ac
//              4'b0111                       1 - Ac
//               ....                         reserved
//            bit3:0 Destination alpha Blending Factor AFd, when bit10:8 != LOGIC OP
//              4'b0000                       0
//              4'b0001                       1
//              4'b0010                       As
//              4'b0011                       1 - As
//              4'b0100                       Ad
//              4'b0101                       1 - Ad
//              4'b0110                       Ac
//              4'b0111                       1 - Ac
//               ....                         reserved
//            bit3:0 logic operations, when bit10:8 == LOGIC OP
//              4'b0000: CLEAR                       0
//              4'b0001: COPY                        s
//              4'b0010: NOOP                        d
//              4'b0011: SET                         1
//              4'b0100: COPY_INVERT                 ~s
//              4'b0101: INVERT                      ~d
//              4'b0110: AND_REVERSE                 s & ~d
//              4'b0111: OR_REVERSE                  s | ~d
//              4'b1000: AND                         s & d
//              4'b1001: OR                          s | d
//              4'b1010: NAND                        ~(s & d)
//              4'b1011: NOR                         ~(s | d)
//              4'b1100: XOR                         s ^ d
//              4'b1101: EQUIV                       ~(s ^ d)
//              4'b1110: AND_INVERTED                ~s & d
//              4'b1111: OR_INVERTED                 ~s | d
#define GE2D_ALU_OP_CTRL                0x8cc     //32'h00010010 

//bit 31:0 (RGBA,YCBCRA)
#define GE2D_ALU_CONST_COLOR            0x8cd     //32'h00808000 

//SRC1 Key
//31:0 
#define GE2D_SRC1_KEY                   0x8ce     //'h0 

//SRC1 Key Mask
//31:0 
#define GE2D_SRC1_KEY_MASK              0x8cf     //'h0 

//SRC2 Key
//31:0 
#define GE2D_SRC2_KEY                   0x8d0     //'h0

//SRC2 Key Mask
//31:0 
#define GE2D_SRC2_KEY_MASK              0x8d1     //'h0

//Destination Bit Mask
//31:0 
#define GE2D_DST_BITMASK                0x8d2     //'h0 

//Bit 31    DP onoff mode, 0: on_counter means how many pixels will output before ge2d turns off
//                         1: on_counter means how many clocks will ge2d turn on before ge2d turns off
//Bit 30:16     DP on counter
//Bit 15        0: vd_format doesnt have onoff mode, 1: vd format has onoff mode
//Bit 14:0      DP off counter
#define GE2D_DP_ONOFF_CTRL              0x8d3     //'h0


//Because there are many coefficients used in the vertical filter and horizontal filters,
//indirect access the coefficients of vertical filter and horizontal filter is used.
//For vertical filter, there are 33x4 coefficients 
//For horizontal filter, there are 33x4 coefficients
//Bit 15	index increment, if bit9 == 1  then (0: index increase 1, 1: index increase 2) else (index increase 2)	
//Bit 14	1: read coef through cbus enable, just for debug purpose in case when we wanna check the coef in ram in correct or not
//Bit 9     if true, use 9bit resolution coef, other use 8bit resolution coef
//Bit 8	    type of index, 0: vertical coef
//						   1: horizontal coef
//Bit 6:0 	coef index
#define GE2D_SCALE_COEF_IDX			0x8d4	 //16'h0

//coefficients for vertical filter and horizontal filter
#define GE2D_SCALE_COEF				0x8d5	 //32'h0

//Bit 24    src2 alpha fill mode, 0: repeat innner alpha, 1: fill src2 outside alpha
//Bit 24    src2 alpha fill mode: together with GE2D_GEN_CTRL0[4](fill_mode), define what alpha values are used
//                                for the area outside the clipping window. As below:
//                                fill_mode=0, alpha_fill_mode=0 : use inner alpha, (or default_alpha if src data have no alpha values);
//                                fill_mode=0, alpha_fill_mode=1 : use outside_alpha;
//                                fill_mode=1, alpha_fill_mode=0 : use default_alpha;
//                                fill_mode=1, alpha_fill_mode=1 : use outside_alpha.
//Bit 23:16 src2 outside alpha
//Bit 8     src1 alpha fill mode, refer to src2 alpha fill mode above. 
//Bit 7:0   src1 outside alpha
#define GE2D_SRC_OUTSIDE_ALPHA	    0x8d6	 //32'h0

// synopsys translate_off
#endif  // GE2D_REGS_H
// synopsys translate_on
