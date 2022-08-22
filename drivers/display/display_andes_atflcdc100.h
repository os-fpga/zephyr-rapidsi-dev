/*
 * Copyright (c) 2020 Andes Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef DISPLAY_ANDES_ATFLCD100_DRIVER_H__
#define DISPLAY_ANDES_ATFLCD100_DRIVER_H__

#define REG_LCD_TIMING_0        0x00
#define REG_LCD_TIMING_1        0x04
#define REG_LCD_TIMING_2        0x08
#define REG_LCD_FRAME_BASE      0x10
#define REG_LCD_INT_EN          0x18
#define REG_LCD_CTRL            0x1c
#define REG_LCD_INT_CLR         0x20
#define REG_LCD_INT             0x24
#define REG_OSD_CTRL_0          0x34
#define REG_OSD_CTRL_1          0x38
#define REG_OSD_CTRL_2          0x4c
#define REG_OSD_CTRL_3          0x40
#define REG_GPIO_CTRL           0x44
#define REG_PALETTE_WR_PORT     0x200
#define REG_OSD_FONT_WR_PORT    0x8000
#define REG_OSD_ATTR_WR_PORT    0xc000

#define LCD_DEV_CFG(dev) ((const struct display_atflcdc100_device_config * const) (dev)->config)

#define LCD_TIMING_0(dev)               (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_TIMING_0)
#define LCD_TIMING_1(dev)               (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_TIMING_1)
#define LCD_TIMING_2(dev)               (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_TIMING_2)
#define LCD_FRAME_BASE(dev) 	        (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_FRAME_BASE)
#define LCD_INT_EN(dev)                 (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_INT_EN)
#define LCD_CTRL(dev)                   (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_CTRL)
#define LCD_INT_CLR(dev)                (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_INT_CLR)
#define LCD_INT(dev)                    (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_LCD_INT)
#define LCD_OSD_CTRL_0(dev)             (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_CTRL_0)
#define LCD_OSD_CTRL_1(dev)             (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_CTRL_1)
#define LCD_OSD_CTRL_2(dev)             (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_CTRL_2)
#define LCD_OSD_CTRL_3(dev)	        (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_CTRL_3)
#define LCD_GPIO_CTRL(dev)              (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_GPIO_CTRL)
#define LCD_PALETTE_WR_PORT(dev)        (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_PALETTE_WR_PORT)
#define LCD_OSD_FONT_WR_PORT(dev)       (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_FONT_WR_PORT)
#define LCD_OSD_ATTR_WR_PORT(dev)       (LCD_DEV_CFG(dev)->lcdc_base_addr + REG_OSD_ATTR_WR_PORT)

#define REG_LCD_TIMING_0_HBP_OFFSET     (24)
#define REG_LCD_TIMING_0_HFP_OFFSET     (16)
#define REG_LCD_TIMING_0_HW_OFFSET      (8)
#define REG_LCD_TIMING_0_PL             (2)

#define REG_LCD_TIMING_1_VHP            (24)
#define REG_LCD_TIMING_1_VFP            (16)
#define REG_LCD_TIMING_1_VW             (10)
#define REG_LCD_TIMING_1_LF             (0)

#define REG_LCD_CTRL_LCD_ON             (11)
#define REG_LCD_CTRL_TFT                (5)

#define REG_LCD_CTRL_LCD_ON_MASK        (1 << REG_LCD_CTRL_LCD_ON)
#define REG_LCD_CTRL_TFT_MASK           (1 << REG_LCD_CTRL_TFT)
#define DISABLE_LCD_MASK                (REG_LCD_CTRL_LCD_ON_MASK | REG_LCD_CTRL_TFT_MASK)
/*
 * HBP : Horizontal Back Porch
 * HFP : Horizontal Front Porch
 * HSPW: Horizontal Sync. Pulse Width
 * PPL : Pixels-per-line = 16(PPL+1)
 */
#define ENC_PARAM_TIME0(HBP, HFP, HSPW, PPL)    \
        ((((HBP)         - 1) << 24) |          \
         (((HFP)         - 1) << 16) |          \
         (((HSPW)        - 1) << 8 ) |          \
         ((((PPL) >> 4) - 1) << 2 ))

/*
 * HBP : Vertical Back Porch
 * HFP : Vertical Front Porch
 * HSPW: Vertical Sync. Pulse Width
 * LPP : Lines-per-frame = LPF + 1
 */
#define ENC_PARAM_TIME1(VBP, VFP, VSPW, LPF)    \
        ((((VBP)     ) << 24) |                 \
         (((VFP)     ) << 16) |                 \
         (((VSPW) - 1) << 10) |                 \
         (((LPF)  - 1) ))

/*
 * PRA : Pixel Rate Adaptive
 * IOE : Invert Panel Output Enable
 * IPC : Invert Panel Clock (Test Chip Testing)
 * IHS : Invert Horisontal Sync.
 * IVS : Invert Versical Sync.
 * PCD : Panel Clock Divisor
 */
#define ENC_PARAM_TIME2(PRA, IOE, IPC, IHS, IVS, PCD)   \
        (((PRA)     << 15) |                            \
         ((IOE)     << 14) |                            \
         ((IPC)     << 13) |                            \
         ((IHS)     << 12) |                            \
         ((IVS)     << 11) |                            \
         (((PCD) - 1) ))

/*
 * Enable YCbCr
 * Enable YCbCr422
 * FIFO threadhold
 * Panel type, 0-6bit, 1-8bit
 * LcdVComp, when to generate interrupt, 1: start of back_porch
 * Power Enable
 * Big Endian Pixel/Byte Ordering
 * BGR
 * TFT
 * LCD bits per pixel
 * Controller Enable
 */
#define ENC_PARAM_CTRL(ENYUV, ENYUV422, FIFOTH, PTYPE, VCOMP, LCD_ON, ENDIAN, BGR, TFT, BPP, LCD_EN) \
        ((ENYUV        << 18) |                 \
         (ENYUV422     << 17) |                 \
         (FIFOTH       << 16) |                 \
         (PTYPE        << 15) |                 \
         (VCOMP        << 12) |                 \
         (LCD_ON       << 11) |                 \
         (ENDIAN       <<  9) |                 \
         (BGR          <<  8) |                 \
         (TFT          <<  5) |                 \
         (BPP          <<  1) |                 \
         (LCD_EN))


#ifdef CONFIG_ANDES_RGB_565
typedef uint16_t pixel_t;
#define PANEL_PIXEL_FORMAT      PIXEL_FORMAT_RGB_565
#define FRAMEBUF_SIZE           (X_RES * Y_RES << 1)
#define LCD_CTRL_VALUE          ENC_PARAM_CTRL(0, 0, 1, 1, 3, 1, 0, 1, 1, 4, 1 )
#else
typedef uint32_t pixel_t;
#define PANEL_PIXEL_FORMAT      PIXEL_FORMAT_RGB_888
#define FRAMEBUF_SIZE           (X_RES * Y_RES << 2)
#define LCD_CTRL_VALUE          ENC_PARAM_CTRL(0, 0, 1, 1, 3, 1, 0, 1, 1, 5, 1 )
#endif

#ifdef CONFIG_PANEL_AUA036QN01
#define X_RES                   320
#define Y_RES                   240
#define LCD_TIMING_0_VALUE      ENC_PARAM_TIME0(7, 6, 1, 320)
#define LCD_TIMING_1_VALUE      ENC_PARAM_TIME1(1, 1, 1, 240)
#define LCD_TIMING_2_VALUE      ENC_PARAM_TIME2(0, 0, 1, 1, 1, 7)
#endif

#ifdef CONFIG_PANEL_LW500AC9601
#define X_RES                   800
#define Y_RES                   480
#define LCD_TIMING_0_VALUE      ENC_PARAM_TIME0(88, 40, 128, 800)
#define LCD_TIMING_1_VALUE      ENC_PARAM_TIME1(21, 1, 3, 480)
#define LCD_TIMING_2_VALUE      ENC_PARAM_TIME2(0, 0, 1, 1, 1, 4)
#endif

#ifdef CONFIG_PANEL_AUA070VW04
#define X_RES                   800
#define Y_RES                   480
#define LCD_TIMING_0_VALUE      ENC_PARAM_TIME0(88, 44, 128, 800)
#define LCD_TIMING_1_VALUE      ENC_PARAM_TIME1(21, 1, 3, 480)
#define LCD_TIMING_2_VALUE      ENC_PARAM_TIME2(0, 1, 1, 1, 1, 4)
#endif

#define INWORD(x)     sys_read32(x)
#define OUTWORD(x, d) sys_write32(d, x)

#endif
