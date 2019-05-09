/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _DRV8323_H_
#define _DRV8323_H_

//! \file   drivers/drvic/drv8323/src/32b/f28x/f2806x/drv8323.h
//! \brief  Contains public interface to various functions related
//!         to the DRV8323 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "stdbool.h"
#include "stdint.h"

// drivers

#include "stm32f4xx_hal.h"

// Port
typedef SPI_HandleTypeDef* SPI_Handle;
typedef GPIO_TypeDef* GPIO_Handle;
typedef uint16_t GPIO_Number_e;


//!
//! \defgroup DRV8323

//!
//! \ingroup DRV8323
//@{


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines the address mask
//!
#define DRV8323_ADDR_MASK               (0x7800)


//! \brief Defines the data mask
//!
#define DRV8323_DATA_MASK               (0x07FF)


//! \brief Defines the R/W mask
//!
#define DRV8323_RW_MASK                 (0x8000)


//! \brief Defines the R/W mask
//!
#define DRV8323_FAULT_TYPE_MASK         (0x07FF)

#define DRV8323_STATUS1_VDS_LC          (1 << 0)
#define DRV8323_STATUS1_VDS_HC          (1 << 1)
#define DRV8323_STATUS1_VDS_LB          (1 << 2)
#define DRV8323_STATUS1_VDS_HB          (1 << 3)
#define DRV8323_STATUS1_VDS_LA          (1 << 4)
#define DRV8323_STATUS1_VDS_HA          (1 << 5)
#define DRV8323_STATUS1_OTSD            (1 << 6)
#define DRV8323_STATUS1_UVLO            (1 << 7)
#define DRV8323_STATUS1_GDF             (1 << 8)
#define DRV8323_STATUS1_VDS_OCP         (1 << 9)
#define DRV8323_STATUS1_FAULT           (1 << 10)

#define DRV8323_STATUS2_VGS_LC          (1 << 0)
#define DRV8323_STATUS2_VGS_HC          (1 << 1)
#define DRV8323_STATUS2_VGS_LB          (1 << 2)
#define DRV8323_STATUS2_VGS_HB          (1 << 3)
#define DRV8323_STATUS2_VGS_LA          (1 << 4)
#define DRV8323_STATUS2_VGS_HA          (1 << 5)
#define DRV8323_STATUS2_CPUV            (1 << 6)
#define DRV8323_STATUS2_OTW             (1 << 7)
#define DRV8323_STATUS2_SC_OC           (1 << 8)
#define DRV8323_STATUS2_SB_OC           (1 << 9)
#define DRV8323_STATUS2_SA_OC           (1 << 10)

#define DRV8323_DRVCTRL_CLR_FLT         (1 << 0)
#define DRV8323_DRVCTRL_BRAKE           (1 << 1)
#define DRV8323_DRVCTRL_COAST           (1 << 2)
#define DRV8323_DRVCTRL_1PWM_DIR        (1 << 3)
#define DRV8323_DRVCTRL_1PWM_COM        (1 << 4)
#define DRV8323_DRVCTRL_PWM_MODE        (3 << 5)
#define DRV8323_DRVCTRL_OTW_REP         (1 << 7)
#define DRV8323_DRVCTRL_DIS_GDF         (1 << 8)
#define DRV8323_DRVCTRL_DIS_CPUV        (1 << 9)

#define DRV8323_GATEDRVHS_IDRIVEN_HS    (0xf << 0)
#define DRV8323_GATEDRVHS_IDRIVEP_HS    (0xf << 4)
#define DRV8323_GATEDRVHS_LOCK          (3 << 8)

#define DRV8323_GATEDRVLS_IDRIVEN_LS    (0xf << 0)
#define DRV8323_GATEDRVLS_IDRIVEP_LS    (0xf << 4)
#define DRV8323_GATEDRVLS_TDRIVE        (3 << 8)
#define DRV8323_GATEDRVLS_CBC           (1 << 10)

#define DRV8323_OCPCTRL_VDS_LVL         (0xf << 0)
#define DRV8323_OCPCTRL_OCP_DEG         (3 << 4)
#define DRV8323_OCPCTRL_OCP_MODE        (3 << 6)
#define DRV8323_OCPCTRL_DEAD_TIME       (3 << 8)
#define DRV8323_OCPCTRL_TRETRY          (1 << 10)

#define DRV8323_CSACTRL_SEN_LVL         (3 << 0)
#define DRV8323_CSACTRL_CSA_CAL_C       (1 << 2)
#define DRV8323_CSACTRL_CSA_CAL_B       (1 << 3)
#define DRV8323_CSACTRL_CSA_CAL_A       (1 << 4)
#define DRV8323_CSACTRL_DIS_SEN         (1 << 5)
#define DRV8323_CSACTRL_CSA_GAIN        (3 << 6)
#define DRV8323_CSACTRL_LS_REF          (1 << 8)
#define DRV8323_CSACTRL_VREF_DIV        (1 << 9)
#define DRV8323_CSACTRL_CSA_FET         (1 << 10)


// **************************************************************************
// the typedefs

//! \brief Enumeration for the R/W modes
//!
typedef enum 
{
  DRV8323_CtrlMode_Read = 1 << 15,   //!< Read Mode
  DRV8323_CtrlMode_Write = 0 << 15   //!< Write Mode
} DRV8323_CtrlMode_e;


//! \brief Enumeration for the DC calibration modes
//!
typedef enum 
{
  DRV8323_DcCalMode_ChB_Load   = (0 << 3),   //!< Shunt amplifier 1 connected to load via input pins
  DRV8323_DcCalMode_ChB_NoLoad = (1 << 3),   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
  DRV8323_DcCalMode_ChC_Load   = (0 << 2),   //!< Shunt amplifier 2 connected to load via input pins
  DRV8323_DcCalMode_ChC_NoLoad = (1 << 2)    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
} DRV8323_DcCalMode_e;


//! \brief Enumeration for the fault types
//!
typedef enum 
{
  DRV8323_FaultType_NoFault  = (0 << 0),  //!< No fault
  DRV8323_FaultType_VDS_LC_OC = (1 << 0),  //!< FET Low side, Phase C Over Current fault
  DRV8323_FaultType_VDS_HC_OC = (1 << 1),  //!< FET High side, Phase C Over Current fault
  DRV8323_FaultType_VDS_LB_OC = (1 << 2),  //!< FET Low side, Phase B Over Current fault
  DRV8323_FaultType_VDS_HB_OC = (1 << 3),  //!< FET High side, Phase B Over Current fault
  DRV8323_FaultType_VDS_LA_OC = (1 << 4),  //!< FET Low side, Phase A Over Current fault
  DRV8323_FaultType_VDS_HA_OC = (1 << 5),  //!< FET High side, Phase A Over Current fault
  DRV8323_FaultType_OTSD      = (1 << 6),  //!< Over Temperature shutdown fault
  DRV8323_FaultType_UVLO      = (1 << 7),  //!< Undervoltage lockout fault
  DRV8323_FaultType_GDF       = (1 << 8),  //!< Gate drive fault
  DRV8323_FaultType_VCS_OCP   = (1 << 9),  //!< VDS overcurrent fault
  // DRV8323_FaultType_VGS_LC    = (1 << 10),
  // DRV8323_FaultType_VGS_HC    = (1 << 11),
  // DRV8323_FaultType_VGS_LB    = (1 << 12),
  // DRV8323_FaultType_VGS_HB    = (1 << 13),
  // DRV8323_FaultType_VGS_LA    = (1 << 14),
  // DRV8323_FaultType_VGS_HA    = (1 << 15),
  // DRV8323_FaultType_CPUV      = (1 << 16),
  // DRV8323_FaultType_OTW       = (1 << 17),
  // DRV8323_FaultType_SC_OC     = (1 << 18),
  // DRV8323_FaultType_SB_OC     = (1 << 19),
  // DRV8323_FaultType_SA_OC     = (1 << 20)
} DRV8323_FaultType_e;


//! \brief Enumeration for the Over Current modes
//!
typedef enum 
{
  DRV8323_OcMode_Latch        = 0 << 6,   //!< current limit when OC detected
  DRV8323_OcMode_Retry        = 1 << 6,   //!< latch shut down when OC detected
  DRV8323_OcMode_ReportOnly   = 2 << 6,   //!< report only when OC detected
  DRV8323_OcMode_Disabled     = 3 << 6    //!< OC protection disabled
} DRV8323_OcMode_e;


//! \brief Enumeration for the Over Current Off Time modes
//!
typedef enum 
{
  DRV8323_OcRetryTime_4ms     = 0 << 10,   //!< normal CBC operation
  DRV8323_OcRetryTime_50us    = 1 << 10    //!< off time control during OC
} DRV8323_OcRetryTime_e;


//! \brief Enumeration for the Over Current, Temperature Warning modes
//!
typedef enum 
{
  DRV8323_OcTwMode_Both    = 0 << 0,   //!< report both OT and OC at /OCTW pin
  DRV8323_OcTwMode_OT_Only = 1 << 0,   //!< report only OT at /OCTW pin
  DRV8323_OcTwMode_OC_Only = 2 << 0    //!< report only OC at /OCTW pin
} DRV8323_OcTwMode_e;


//! \brief Enumeration for the drv8323 peak current levels
//!
typedef enum 
{
  DRV8323_PeakCurrent_1p70_A  = 0 << 0,   //!< drv8323 driver peak current 1.70A
  DRV8323_PeakCurrent_0p70_A  = 1 << 0,   //!< drv8323 driver peak current 0.70A
  DRV8323_PeakCurrent_0p25_A  = 2 << 0    //!< drv8323 driver peak current 0.25A
} DRV8323_PeakCurrent_e;


//! \brief Enumeration for the PWM modes
//!
typedef enum 
{
  DRV8323_PwmMode_Six_Inputs   = 0 << 5,   //!< six independent inputs
  DRV8323_PwmMode_Three_Inputs = 1 << 5    //!< three independent nputs
} DRV8323_PwmMode_e;


//! \brief Enumeration for the register names
//!
typedef enum 
{
  DRV8323_RegName_Status_1      = 0 << 11,   //!< Status Register 1
  DRV8323_RegName_Status_2      = 1 << 11,   //!< Status Register 2
  DRV8323_RegName_Drive_control = 2 << 11,  //!< Control Register 1
  DRV8323_RegName_Gatedrive_HS  = 3 << 11,   //!< Control Register 2
  DRV8323_RegName_Gatedrive_LS  = 4 << 11,   //!< Control Register 2
  DRV8323_RegName_OCP_Control   = 5 << 11,   //!< Control Register 2
  DRV8323_RegName_CSA_Control   = 6 << 11   //!< Control Register 2
} DRV8323_RegName_e;



//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum 
{
  DRV8323_Reset_Normal = 0 << 2,   //!< normal
  DRV8323_Reset_All = 1 << 2       //!< reset all
} DRV8323_Reset_e;


//! \brief Enumeration for the shunt amplifier gains
//!
typedef enum 
{
  DRV8323_ShuntAmpGain_5VpV   = 0 << 6,   //!< 10 V per V
  DRV8323_ShuntAmpGain_10VpV  = 1 << 6,   //!< 20 V per V
  DRV8323_ShuntAmpGain_20VpV  = 2 << 6,   //!< 40 V per V
  DRV8323_ShuntAmpGain_40VpV  = 3 << 6    //!< 80 V per V
} DRV8323_ShuntAmpGain_e;


//! \brief Enumeration for the shunt amplifier number
//!
typedef enum 
{
  DRV8323_ShuntAmpNumber_1 = 1,      //!< Shunt amplifier number 1
  DRV8323_ShuntAmpNumber_2 = 2       //!< Shunt amplifier number 2
} DRV8323_ShuntAmpNumber_e;


//! \brief Enumeration for the Vds level for th over current adjustment
//!
typedef enum 
{
  DRV8323_VdsLevel_0p060_V =  0 << 6,      //!< Vds = 0.060 V
  DRV8323_VdsLevel_0p130_V =  1 << 6,      //!< Vds = 0.068 V
  DRV8323_VdsLevel_0p200_V =  2 << 6,      //!< Vds = 0.076 V
  DRV8323_VdsLevel_0p260_V =  3 << 6,      //!< Vds = 0.086 V
  DRV8323_VdsLevel_0p310_V =  4 << 6,      //!< Vds = 0.097 V
  DRV8323_VdsLevel_0p450_V =  5 << 6,      //!< Vds = 0.109 V
  DRV8323_VdsLevel_0p530_V =  6 << 6,      //!< Vds = 0.123 V
  DRV8323_VdsLevel_0p600_V =  7 << 6,      //!< Vds = 0.138 V
  DRV8323_VdsLevel_0p680_V =  8 << 6,      //!< Vds = 0.155 V
  DRV8323_VdsLevel_0p750_V =  9 << 6,      //!< Vds = 0.175 V
  DRV8323_VdsLevel_0p940_V = 10 << 6,      //!< Vds = 0.197 V
  DRV8323_VdsLevel_1p130_V = 11 << 6,      //!< Vds = 0.222 V
  DRV8323_VdsLevel_1p300_V = 12 << 6,      //!< Vds = 0.250 V
  DRV8323_VdsLevel_1p500_V = 13 << 6,      //!< Vds = 0.282 V
  DRV8323_VdsLevel_1p700_V = 14 << 6,      //!< Vds = 0.317 V
  DRV8323_VdsLevel_1p880_V = 15 << 6,      //!< Vds = 0.358 V
} DRV8323_VdsLevel_e;


typedef enum
{
  DRV8323_GETID=0
} Drv8323SpiOutputDataSelect_e;


typedef struct _DRV_SPI_8323_Stat1_t_
{
  bool                  FAULT;
  bool                  OVDS_OCP;
  bool                  GDF;
  bool                  UVLO;
  bool                  OTSD;
  bool                  VDS_HA_OC;
  bool                  VDS_LA_OC;
  bool                  VDS_HB_OC;
  bool                  VDS_LB_OC;
  bool                  VDS_HC_OC;
  bool                  VDS_LC_OC;
}DRV_SPI_8323_Stat1_t_;


typedef struct _DRV_SPI_8323_Stat2_t_
{
  bool                  SA_OC;
  bool                  SB_OC;
  bool                  SC_OC;
  bool                  OTW;
  bool                  CPUV;
  bool                  VGS_HA;
  bool                  VGS_LA;
  bool                  VGS_HB;
  bool                  VGS_LB;
  bool                  VGS_HC;
  bool                  VGS_LC;
}DRV_SPI_8323_Stat2_t_;

typedef struct _DRV_SPI_8323_DRVCTRL_t_
{
  DRV8323_PeakCurrent_e    DRV8323_CURRENT;
  DRV8323_Reset_e          DRV8323_RESET;
  DRV8323_PwmMode_e        PWM_MODE;
  DRV8323_OcMode_e         OC_MODE;
  DRV8323_VdsLevel_e       OC_ADJ_SET;
}DRV_SPI_8323_CTRL1_t_;



typedef struct _DRV_SPI_8323_Vars_t_
{
  // DRV_SPI_8323_Stat1_t_     Stat_Reg_1;
  // DRV_SPI_8323_Stat2_t_     Stat_Reg_2;
  // DRV_SPI_8323_CTRL1_t_     Ctrl_Reg_1;
  // DRV_SPI_8323_CTRL2_t_     Ctrl_Reg_2;
  // uint16_t                  Stat_Reg_1_Value;
  // uint16_t                  Stat_Reg_2_Value;
  // uint16_t                  Ctrl_Reg_1_Value;
  // uint16_t                  Ctrl_Reg_2_Value;
  // bool                  SndCmd;
  // bool                  RcvCmd;

}DRV_SPI_8323_Vars_t;


//! \brief Defines the DRV8323 object
//!
typedef struct _DRV8323_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      EngpioHandle;               //!< the gpio handle that is connected to the drv8323 enable pin
  GPIO_Number_e    EngpioNumber;               //!< the gpio number that is connected to the drv8323 enable pin
  GPIO_Handle      nCSgpioHandle;              //!< the gpio handle that is connected to the drv8323 nCS pin
  GPIO_Number_e    nCSgpioNumber;               //!< the gpio number that is connected to the drv8323 nCS pin
  bool             RxTimeOut;                  //!< the timeout flag for the RX fifo
  bool             enableTimeOut;              //!< the timeout flag for drv8323 enable
} DRV8323_Obj;


//! \brief Defines the DRV8323 handle
//!
typedef struct _DRV8323_Obj_ *DRV8323_Handle;


//! \brief Defines the DRV8323 Word type
//!
typedef  uint16_t    DRV8323_Word_t;


// **************************************************************************
// the globals



// **************************************************************************
// the function prototypes


//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DRV8323_Word_t DRV8323_buildCtrlWord(const DRV8323_CtrlMode_e ctrlMode,
                                                   const DRV8323_RegName_e regName,
                                                   const uint16_t data)
{
  DRV8323_Word_t ctrlWord = ctrlMode | regName | (data & DRV8323_DATA_MASK);

  return(ctrlWord);
} // end of DRV8323_buildCtrlWord() function


//! \brief     Gets the DC calibration mode
//! \param[in] handle     The DRV8323 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \return    The DC calibration mode
extern DRV8323_DcCalMode_e DRV8323_getDcCalMode(DRV8323_Handle handle,
                                                const DRV8323_ShuntAmpNumber_e ampNumber);

//! \brief     Enables the DRV8323
//! \param[in] handle     The DRV8323 handle
extern void DRV8323_enable(DRV8323_Handle handle);


//! \brief     Gets the fault type
//! \param[in] handle     The DRV8323 handle
//! \return    The fault type
extern DRV8323_FaultType_e DRV8323_getFaultType(DRV8323_Handle handle);


//! \brief     Gets the device ID
//! \param[in] handle     The DRV8323 handle
//! \return    The device ID
extern uint16_t DRV8323_getId(DRV8323_Handle handle);


//! \brief     Gets the over current level
//! \param[in] handle     The DRV8323 handle
//! \return    The over current level, V
extern DRV8323_VdsLevel_e DRV8323_getOcLevel(DRV8323_Handle handle);


//! \brief     Gets the over current mode
//! \param[in] handle     The DRV8323 handle
//! \return    The over current mode
extern DRV8323_OcMode_e DRV8323_getOcMode(DRV8323_Handle handle);


//! \brief     Gets the over current off time mode
//! \param[in] handle     The DRV8323 handle
//! \return    The over current off time mode
//extern DRV8323_OcOffTimeMode_e DRV8323_getOcOffTimeMode(DRV8323_Handle handle);


//! \brief     Gets the over current, temperature warning mode
//! \param[in] handle     The DRV8323 handle
//! \return    The over current, temperature warning mode
extern DRV8323_OcTwMode_e DRV8323_getOcTwMode(DRV8323_Handle handle);


//! \brief     Gets the peak current value
//! \param[in] handle     The DRV8323 handle
//! \return    The peak current value
extern DRV8323_PeakCurrent_e DRV8323_getPeakCurrent(DRV8323_Handle handle);


//! \brief     Gets the PWM mode
//! \param[in] handle     The DRV8323 handle
//! \return    The PWM mode
extern DRV8323_PwmMode_e DRV8323_getPwmMode(DRV8323_Handle handle);


//! \brief     Gets the shunt amplifier gain value
//! \param[in] handle     The DRV8323 handle
//! \return    The shunt amplifier gain value
extern DRV8323_ShuntAmpGain_e DRV8323_getShuntAmpGain(DRV8323_Handle handle);


//! \brief     Gets the status register 1 value
//! \param[in] handle     The DRV8323 handle
//! \return    The status register1 value
extern uint16_t DRV8323_getStatusRegister1(DRV8323_Handle handle);


//! \brief     Gets the status register 2 value
//! \param[in] handle     The DRV8323 handle
//! \return    The status register2 value
extern uint16_t DRV8323_getStatusRegister2(DRV8323_Handle handle);


//! \brief     Initializes the DRV8323 object
//! \param[in] pMemory   A pointer to the memory for the DRV8323 object
//! \param[in] numBytes  The number of bytes allocated for the DRV8323 object, bytes
//! \return    The DRV8323 object handle
extern DRV8323_Handle DRV8323_init(void *pMemory,const size_t numBytes);


//! \brief     Determines if DRV8323 fault has occurred
//! \param[in] handle     The DRV8323 handle
//! \return    A boolean value denoting if a fault has occurred (true) or not (false)
extern bool DRV8323_isFault(DRV8323_Handle handle);


//! \brief     Determines if DRV8323 is in reset
//! \param[in] handle     The DRV8323 handle
//! \return    A boolean value denoting if the DRV8323 is in reset (true) or not (false)
extern bool DRV8323_isReset(DRV8323_Handle handle);


//! \brief     Reads data from the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regName  The register name
//! \return    The data value
extern uint16_t DRV8323_readSpi(DRV8323_Handle handle,const DRV8323_RegName_e regName);


//! \brief     Resets the DRV8323
//! \param[in] handle   The DRV8323 handle
extern void DRV8323_reset(DRV8323_Handle handle);


//! \brief     Resets the enable timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetEnableTimeout(DRV8323_Handle handle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  obj->enableTimeOut = false;

  return;
}


//! \brief     Resets the RX fifo timeout flag
//! \param[in] handle   The DRV8323 handle
static inline void DRV8323_resetRxTimeout(DRV8323_Handle handle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  obj->RxTimeOut = false;

  return;
}


//! \brief     Sets the DC calibration mode
//! \param[in] handle     The DRV8323 handle
//! \param[in] ampNumber  The shunt amplifier number
//! \param[in] mode       The DC calibration mode
extern void DRV8323_setDcCalMode(DRV8323_Handle handle,
                                 const DRV8323_ShuntAmpNumber_e ampNumber,
                                 const DRV8323_DcCalMode_e mode);


//! \brief     Sets the GPIO handle in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] gpioHandle  The GPIO handle to use
void DRV8323_setGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] gpioHandle  The GPIO number to use
void DRV8323_setGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber);


//! \brief     Sets the over current level in terms of Vds
//! \param[in] handle    The DRV8323 handle
//! \param[in] VdsLevel  The over current level, V
extern void DRV8323_setOcLevel(DRV8323_Handle handle,const DRV8323_VdsLevel_e VdsLevel);


//! \brief     Sets the over current mode
//! \param[in] handle  The DRV8323 handle
//! \param[in] mode    The over current mode
extern void DRV8323_setOcMode(DRV8323_Handle handle,const DRV8323_OcMode_e mode);


//! \brief     Sets the over current off time mode
//! \param[in] handle   The DRV8323 handle
//! \param[in] mode     The over current off time mode
//extern void DRV8323_setOcOffTimeMode(DRV8323_Handle handle,const DRV8323_OcOffTimeMode_e mode);


//! \brief     Sets the over current, temperature warning mode
//! \param[in] handle  The DRV8323 handle
//! \param[in] mode    The over current, temperature warning mode
extern void DRV8323_setOcTwMode(DRV8323_Handle handle,const DRV8323_OcTwMode_e mode);


//! \brief     Sets the peak current value
//! \param[in] handle       The DRV8323 handle
//! \param[in] peakCurrent  The peak current value
extern void DRV8323_setPeakCurrent(DRV8323_Handle handle,const DRV8323_PeakCurrent_e peakCurrent);


//! \brief     Sets the PWM mode
//! \param[in] handle  The DRV8323 handle
//! \param[in] mode    The PWM mode
extern void DRV8323_setPwmMode(DRV8323_Handle handle,const DRV8323_PwmMode_e mode);


//! \brief     Sets the shunt amplifier gain value
//! \param[in] handle  The DRV8323 handle
//! \param[in] gain    The shunt amplifier gain value
extern void DRV8323_setShuntAmpGain(DRV8323_Handle handle,const DRV8323_ShuntAmpGain_e gain);


//! \brief     Sets the SPI handle in the DRV8323
//! \param[in] handle     The DRV8323 handle
//! \param[in] spiHandle  The SPI handle to use
void DRV8323_setSpiHandle(DRV8323_Handle handle,SPI_Handle spiHandle);


//! \brief     Writes data to the DRV8323 register
//! \param[in] handle   The DRV8323 handle
//! \param[in] regName  The register name
//! \param[in] data     The data value
extern void DRV8323_writeSpi(DRV8323_Handle handle,const DRV8323_RegName_e regName,const uint16_t data);


//! \brief     Interface to all 8323 SPI variables
//!
//! \details   Call this function periodically to be able to read the DRV8323 Status1, Status2,
//!            Control1, and Control2 registers and write the Control1 and Control2 registers.
//!            This function updates the members of the structure DRV_SPI_8323_Vars_t.
//!            <b>How to use in Setup</b>
//!            <b>Code</b>
//!            Add the structure declaration DRV_SPI_8323_Vars_t to your code
//!            Make sure the SPI and 8323 EN_Gate GPIO are setup for the 8323 by using HAL_init and HAL_setParams
//!            During code setup, call HAL_enableDrv and HAL_setupDrvSpi
//!            In background loop, call DRV8323_writeData and DRV8323_readData
//!            <b>How to use in Runtime</b>
//!            <b>Watch window</b>
//!            Add the structure, declared by DRV_SPI_8323_Vars_t above, to the watch window
//!            <b>Runtime</b>
//!            Pull down the menus from the DRV_SPI_8323_Vars_t strcuture to the desired setting
//!            Set SndCmd to send the settings to the DRV8323
//!            If a read of the DRV8323 registers is required, se RcvCmd
//!
//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8323_Vars  The (DRV_SPI_8323_Vars_t) structure that contains all DRV8323 Status/Control register options
//extern void DRV8323_writeData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


//! \param[in] handle  The DRV8323 handle
//! \param[in] Spi_8323_Vars  The (DRV_SPI_8323_Vars_t) structure that contains all DRV8323 Status/Control register options
//extern void DRV8323_readData(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


//! \brief     Initialize the interface to all 8323 SPI variables
//! \param[in] handle  The DRV8323 handle
//extern void DRV8323_setupSpi(DRV8323_Handle handle, DRV_SPI_8323_Vars_t *Spi_8323_Vars);


#ifdef __cplusplus
}
#endif // extern "C"

//@}  // ingroup

#endif // end of _DRV8323_H_ definition





