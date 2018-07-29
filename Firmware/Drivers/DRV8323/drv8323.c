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
//! \file   drivers/drvic/drv8323/src/32b/f28x/f2806x/drv8323.c
//! \brief  Contains the various functions related to the DRV8323 object
//!
//! (C) Copyright 2015, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "assert.h"
#include <math.h>
#include "cmsis_os.h"

// drivers
#include "drv8323.h"

#include "utils.h"


// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

void DRV8323_enable(DRV8323_Handle handle)
{

  //Enable driver
  HAL_GPIO_WritePin(handle->EngpioHandle, handle->EngpioNumber, GPIO_PIN_SET);

  //Wait for driver to come online
  osDelay(10);

  // Make sure the Fault bit is not set during startup
  while((DRV8323_readSpi(handle, DRV8323_RegName_Status_1) & DRV8323_STATUS1_FAULT) != 0);

  // Wait for the DRV8323 registers to update
  osDelay(1);

  return;
}

// DRV8323_DcCalMode_e DRV8323_getDcCalMode(DRV8323_Handle handle,const DRV8323_ShuntAmpNumber_e ampNumber)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // clear the bits
//   if(ampNumber == DRV8323_ShuntAmpNumber_1)
//     {
//       data &= (~DRV8323_CTRL2_DC_CAL_1_BITS);

//     }
//   else if(ampNumber == DRV8323_ShuntAmpNumber_2)
//     {
//       data &= (~DRV8323_CTRL2_DC_CAL_2_BITS);
//     }

//   return((DRV8323_DcCalMode_e)data);
// } // end of DRV8323_getDcCalMode() function


DRV8323_FaultType_e DRV8323_getFaultType(DRV8323_Handle handle)
{
  //DRV8323_Word_t      readWord;
  DRV8323_FaultType_e faultType = DRV8323_FaultType_NoFault;


  // read the data
  //readWord = DRV8323_readSpi(handle,DRV8323_RegName_Status_1);

  // if(readWord & DRV8323_STATUS1_FAULT_BITS)
  //   {
  //     faultType = (DRV8323_FaultType_e)(readWord & DRV8323_FAULT_TYPE_MASK);

  //     if(faultType == DRV8323_FaultType_NoFault)
  //       {
  //         // read the data
  //         readWord = DRV8323_readSpi(handle,DRV8323_RegName_Status_2);

  //         if(readWord & DRV8323_STATUS2_GVDD_OV_BITS)
  //           {
  //             faultType = DRV8323_FaultType_GVDD_OV;
  //           }
  //       }
  //   }

  return(faultType);
} // end of DRV8323_getFaultType() function


// uint16_t DRV8323_getId(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Status_2);

//   // mask bits
//   data &= DRV8323_STATUS2_ID_BITS;

//   return(data);
// } // end of DRV8323_getId() function


// DRV8323_VdsLevel_e DRV8323_getOcLevel(DRV8323_Handle handle)
// {
//   uint16_t data;

//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   // clear the bits
//   data &= (~DRV8323_CTRL1_OC_ADJ_SET_BITS);

//   return((DRV8323_VdsLevel_e)data);
// } // end of DRV8323_getOcLevel() function


// DRV8323_OcMode_e DRV8323_getOcMode(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   // clear the bits
//   data &= (~DRV8323_CTRL1_OC_MODE_BITS);

//   return((DRV8323_OcMode_e)data);
// } // end of DRV8323_getOcMode() function


// DRV8323_OcOffTimeMode_e DRV8323_getOcOffTimeMode(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // clear the bits
//   data &= (~DRV8323_CTRL2_OC_TOFF_BITS);

//   return((DRV8323_OcOffTimeMode_e)data);
// } // end of DRV8323_getOcOffTimeMode() function


// DRV8323_OcTwMode_e DRV8323_getOcTwMode(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // clear the bits
//   data &= (~DRV8323_CTRL2_OCTW_SET_BITS);

//   return((DRV8323_OcTwMode_e)data);
// } // end of DRV8323_getOcTwMode() function


// DRV8323_PeakCurrent_e DRV8323_getPeakCurrent(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   // clear the bits
//   data &= (~DRV8323_CTRL1_GATE_CURRENT_BITS);

//   return((DRV8323_PeakCurrent_e)data);
// } // end of DRV8323_getPeakCurrent() function


// DRV8323_PwmMode_e DRV8323_getPwmMode(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   // clear the bits
//   data &= (~DRV8323_CTRL1_PWM_MODE_BITS);

//   return((DRV8323_PwmMode_e)data);
// } // end of DRV8323_getPwmMode() function


// DRV8323_ShuntAmpGain_e DRV8323_getShuntAmpGain(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // clear the bits
//   data &= (~DRV8323_CTRL2_GAIN_BITS);

//   return((DRV8323_ShuntAmpGain_e)data);
// } // end of DRV8323_getShuntAmpGain() function


DRV8323_Handle DRV8323_init(void *pMemory,const size_t numBytes)
{
  DRV8323_Handle handle;


  if(numBytes < sizeof(DRV8323_Obj))
    return((DRV8323_Handle)NULL);


  // assign the handle
  handle = (DRV8323_Handle)pMemory;

  DRV8323_resetRxTimeout(handle);
  DRV8323_resetEnableTimeout(handle);


  return(handle);
} // end of DRV8323_init() function


void DRV8323_setEnGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioHandle = gpioHandle;

  return;
} // end of DRV8323_setGpioHandle() function


void DRV8323_setEnGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->EngpioNumber = gpioNumber;

  return;
} // end of DRV8323_setGpioNumber() function


void DRV8323_setnCSGpioHandle(DRV8323_Handle handle,GPIO_Handle gpioHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioHandle = gpioHandle;

  return;
} // end of DRV8323_setGpioHandle() function


void DRV8323_setnCSGpioNumber(DRV8323_Handle handle,GPIO_Number_e gpioNumber)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the gpio interface object
  obj->nCSgpioNumber = gpioNumber;

  return;
} // end of DRV8323_setGpioNumber() function


void DRV8323_setSpiHandle(DRV8323_Handle handle,SPI_Handle spiHandle)
{
  DRV8323_Obj *obj = (DRV8323_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of DRV8323_setSpiHandle() function


// bool DRV8323_isFault(DRV8323_Handle handle)
// {
//   DRV8323_Word_t readWord;
//   bool status=false;


//   // read the data
//   readWord = DRV8323_readSpi(handle,DRV8323_RegName_Status_1);

//   if(readWord & DRV8323_STATUS1_FAULT_BITS)
//     {
//       status = true;
//     }

//   return(status);
// } // end of DRV8323_isFault() function


// bool DRV8323_isReset(DRV8323_Handle handle)
// {
//   DRV8323_Word_t readWord;
//   bool status=false;


//   // read the data
//   readWord = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   if(readWord & DRV8323_CTRL1_GATE_RESET_BITS)
//     {
//       status = true;
//     }

//   return(status);
// } // end of DRV8323_isReset() function


uint16_t DRV8323_readSpi(DRV8323_Handle handle, const DRV8323_RegName_e regName)
{

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking read
  uint16_t zerobuff = 0;
  uint16_t controlword = (uint16_t)DRV8323_buildCtrlWord(DRV8323_CtrlMode_Read, regName, 0);
  uint16_t recbuff = 0xbeef;
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);

  // Datasheet says you don't have to pulse the nCS between transfers, (16 clocks should commit the transfer)
  // but for some reason you actually need to pulse it.
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  HAL_SPI_TransmitReceive(handle->spiHandle, (uint8_t*)(&zerobuff), (uint8_t*)(&recbuff), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);

  assert(recbuff != 0xbeef);

  // sj hack
  recbuff = 0;
  return(recbuff & DRV8323_DATA_MASK);
}  // end of DRV8323_readSpi() function


// void DRV8323_reset(DRV8323_Handle handle)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

//   // set the bits
//   data |= DRV8323_CTRL1_GATE_RESET_BITS;

//   // write the data
//   DRV8323_writeSpi(handle,DRV8323_RegName_Control_1,data);

//   return;
// }  // end of DRV8323_reset() function

  
// void DRV8323_setDcCalMode(DRV8323_Handle handle,const DRV8323_ShuntAmpNumber_e ampNumber,const DRV8323_DcCalMode_e mode)
// {
//   uint16_t data;


//   // read data
//   data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // clear the bits
//   if(ampNumber == DRV8323_ShuntAmpNumber_1)
//     {
//       data &= (~DRV8323_CTRL2_DC_CAL_1_BITS);

//     }
//   else if(ampNumber == DRV8323_ShuntAmpNumber_2)
//     {
//       data &= (~DRV8323_CTRL2_DC_CAL_2_BITS);
//     }

//   // set the bits
//   data |= mode;

//   // write the data
//   DRV8323_writeSpi(handle,DRV8323_RegName_Control_2,data);

//   return;
// } // end of DRV8323_setDcCalMode() function


void DRV8323_setOcLevel(DRV8323_Handle handle,const DRV8323_VdsLevel_e VdsLevel)
{
  // uint16_t data;


  // // read data
  // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

  // // clear the bits
  // data &= (~DRV8323_CTRL1_OC_ADJ_SET_BITS);

  // // set the bits
  // data |= VdsLevel;

  // // write the data
  // DRV8323_writeSpi(handle,DRV8323_RegName_Control_1,data);

  // return;
} // end of DRV8323_setOcLevel() function


void DRV8323_setOcMode(DRV8323_Handle handle,const DRV8323_OcMode_e mode)
{
  // uint16_t data;


  // // read data
  // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

  // // clear the bits
  // data &= (~DRV8323_CTRL1_OC_MODE_BITS);

  // // set the bits
  // data |= mode;

  // // write the data
  // DRV8323_writeSpi(handle,DRV8323_RegName_Control_1,data);

  // return;
} // end of DRV8323_setOcMode() function


// void DRV8323_setOcOffTimeMode(DRV8323_Handle handle,const DRV8323_OcOffTimeMode_e mode)
// {
//   // uint16_t data;


//   // // read data
//   // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

//   // // clear the bits
//   // data &= (~DRV8323_CTRL2_OC_TOFF_BITS);

//   // // set the bits
//   // data |= mode;

//   // // write the data
//   // DRV8323_writeSpi(handle,DRV8323_RegName_Control_2,data);

//   // return;
// } // end of DRV8323_setOcOffTimeMode() function


void DRV8323_setOcTwMode(DRV8323_Handle handle,const DRV8323_OcTwMode_e mode)
{
  // uint16_t data;


  // // read data
  // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_2);

  // // clear the bits
  // data &= (~DRV8323_CTRL2_OCTW_SET_BITS);

  // // set the bits
  // data |= mode;

  // // write the data
  // DRV8323_writeSpi(handle,DRV8323_RegName_Control_2,data);

  // return;
} // end of DRV8323_setOcTwMode() function


void DRV8323_setPeakCurrent(DRV8323_Handle handle,const DRV8323_PeakCurrent_e peakCurrent)
{
  // uint16_t data;


  // // read data
  // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

  // // clear the bits
  // data &= (~DRV8323_CTRL1_GATE_CURRENT_BITS);

  // // set the bits
  // data |= peakCurrent;

  // // write the data
  // DRV8323_writeSpi(handle,DRV8323_RegName_Control_1,data);

  // return;
} // end of DRV8323_setPeakCurrent() function


void DRV8323_setPwmMode(DRV8323_Handle handle,const DRV8323_PwmMode_e mode)
{
  // uint16_t data;


  // // read data
  // data = DRV8323_readSpi(handle,DRV8323_RegName_Control_1);

  // // clear the bits
  // data &= (~DRV8323_CTRL1_PWM_MODE_BITS);

  // // set the bits
  // data |= mode;

  // // write the data
  // DRV8323_writeSpi(handle,DRV8323_RegName_Control_1,data);

  // return;
} // end of DRV8323_setPwmMode() function


void DRV8323_setShuntAmpGain(DRV8323_Handle handle,const DRV8323_ShuntAmpGain_e gain)
{
  uint16_t data;


  // read data
  data = DRV8323_readSpi(handle,DRV8323_RegName_CSA_Control);

  // clear the bits
  data &= (~DRV8323_CSACTRL_CSA_GAIN);

  // set the bits
  data |= gain;

  // write the data
  DRV8323_writeSpi(handle, DRV8323_RegName_CSA_Control, data);

  return;
} // end of DRV8323_setShuntAmpGain() function


void DRV8323_writeSpi(DRV8323_Handle handle, const DRV8323_RegName_e regName,const uint16_t data)
{
  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_RESET);
  delay_us(1);

  // Do blocking write
  uint16_t controlword = (uint16_t)DRV8323_buildCtrlWord(DRV8323_CtrlMode_Write, regName, data);
  HAL_SPI_Transmit(handle->spiHandle, (uint8_t*)(&controlword), 1, 1000);
  delay_us(1);

  // Actuate chipselect
  HAL_GPIO_WritePin(handle->nCSgpioHandle, handle->nCSgpioNumber, GPIO_PIN_SET);
  delay_us(1);

  return;
}  // end of DRV8323_writeSpi() function





// end of file
