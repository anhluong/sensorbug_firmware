/******************************************************************************

  @file  bmxService.c

 @brief This file contains a simple I2C transaction with the bmx160
         Created on: Jul 11, 2017

 @author: Abhinand Sukumar

 ******************************************************************************/

#ifndef SERVICES_BMXSERVICE_C_
#define SERVICES_BMXSERVICE_C_

/*********************************************************************
 * INCLUDES
 */
#include <stdio.h>
#include <string.h>

#include <xdc/runtime/System.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include "io.h"


/* Board Header files */
#include "Config/Board_LoRaBUG.h"

//#include "loraCommon.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>

/*******************************************************************************
 * MACROS
 */

#define DELAY_MS(i)    Task_sleep(((i) * 1000) / Clock_tickPeriod)

/*******************************************************************************
 * CONSTANTS
 */
#define BMX_TASK_PRIORITY                     5

#define BMX_TASK_STACK_SIZE                   1200



/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */

// Task configuration
Task_Struct bmxTask;
Char bmxTaskStack[BMX_TASK_STACK_SIZE];


void getBmxData(uint32_t *data)
{

    I2C_Handle handle;
    I2C_Params params;
    I2C_Transaction normalSettings, normalCmd;
    I2C_Transaction suspendCmd;
    I2C_Transaction pmuNormalCheck, pmuSuspendCheck;
    I2C_Transaction dataRead, checkDataReady;

    I2C_Params_init(&params);     // sets custom to NULL
    params.transferMode = I2C_MODE_BLOCKING;
    I2CCC26XX_I2CPinCfg pinCfg;
    pinCfg.pinSDA = Board_I2C0_SDA0;
    pinCfg.pinSCL = Board_I2C0_SCL0;
    params.custom = &pinCfg;

    uint8_t pmu_status, pmu_status_reg_addr;
    pmu_status_reg_addr = 0x03;

    uint8_t data_reg_addr;
    data_reg_addr = 0x04;

    uint8_t cmdAccNorm[2];
    cmdAccNorm[0] = 0x7E;
    cmdAccNorm[1] = 0x11;

    uint8_t cmdGyrNorm[2];
    cmdGyrNorm[0] = 0x7E;
    cmdGyrNorm[1] = 0x15;

    uint8_t accNorm[2];
    accNorm[0] = 0x40;
    accNorm[1] = 0x28;//0b0_010_1000

    uint8_t gyrNorm[2];
    gyrNorm[0] = 0x42;
    gyrNorm[1] = 0x28;


    uint8_t cmdSuspendAll[2];
    cmdSuspendAll[0] = 0x7E;
    cmdSuspendAll[1] = 0xB6; //soft reset

    uint8_t dataReadyAddr;
    dataReadyAddr = 0x1B;

    uint8_t dataReadyStatus;

    //Magnetometer normal mode packets
    uint8_t magIfNormal[2];
    magIfNormal[0] = 0x7E;
    magIfNormal[1] = 0x19;

    //Wait 650uS after this^ conf

    uint8_t magManual[2];
    magManual[0] = 0x4C;
    magManual[1] = 0x80;

    //writes to put in sleep mode
    uint8_t magIf3Sleep[2], magIf2Sleep[2];
    magIf3Sleep[0] = 0x4F;
    magIf3Sleep[1] = 0x01;
    magIf2Sleep[0] = 0x4E;
    magIf2Sleep[1] = 0x4B;

    //Select power mode through indirect write

    uint8_t magIf2Normal[2], magIf3Normal[2];
    magIf3Normal[0] = 0x4F;
    magIf3Normal[1] = 0x04;
    magIf2Normal[0] = 0x4E;
    magIf2Normal[1] = 0x0E;

    //Prepare MAG_IF[1-3] for data mode

    uint8_t magIf3Data[2], magIf2Data[2], magIf1Data[2];

    magIf3Data[0] = 0x4F;
    magIf3Data[1] = 0x02;

    magIf2Data[0] = 0x4E;
    magIf2Data[1] = 0x4C;

    magIf1Data[0] = 0x4D;
    magIf1Data[0] = 0x42;

    uint8_t magConf[2];

    magConf[0] = 0x44;
    magConf[1] = 0x08;

    uint8_t magEn[2];

    magEn[0] = 0x4C;
    magEn[1] = 0x00;

    uint8_t cmdMagNorm[2];

    cmdMagNorm[0] = 0x7E;
    cmdMagNorm[1] = 0x19;


    handle = I2C_open(Board_I2C, &params);

    if(!handle)
    {
        setLed(Board_RLED, true);
        uartputs("Unable to open I2C\r\n");
    }

    //Configure settings for normal mode

    normalSettings.slaveAddress = 0x68;
    normalSettings.writeBuf = accNorm;
    normalSettings.writeCount = 2;
    normalSettings.readBuf = NULL;
    normalSettings.readCount = 0;

    bool status = false;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Acc normal settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Acc normal settings successful \r\n");
    }

    normalSettings.writeBuf = gyrNorm;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Gyr normal settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Gyr normal settings successful \r\n");
    }

    normalSettings.writeBuf = magIfNormal;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag normal settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag normal settings successful \r\n");
    }


    DELAY_MS(1);

    normalSettings.writeBuf = magManual;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag manual settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag manual settings successful \r\n");
    }

    normalSettings.writeBuf = magIf3Sleep;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 3 sleep settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 3 sleep settings successful \r\n");
    }

    normalSettings.writeBuf = magIf2Sleep;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 2 sleep settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 2 sleep settings successful \r\n");
    }


    normalSettings.writeBuf = magIf3Normal;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 3 normal settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 3 normal settings successful \r\n");
    }

    normalSettings.writeBuf = magIf2Normal;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 2 normal settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 2 normal settings successful \r\n");
    }

    normalSettings.writeBuf = magIf3Data;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 3 data settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 3 data settings successful \r\n");
    }

    normalSettings.writeBuf = magIf2Data;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 2 data settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 2 data settings successful \r\n");
    }


    normalSettings.writeBuf = magIf1Data;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag 1 data settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag 1 data settings successful \r\n");
    }


    normalSettings.writeBuf = magConf;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag conf settings unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag conf settings successful \r\n");
    }


    normalSettings.writeBuf = magEn;

    status = I2C_transfer(handle, &normalSettings);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag enable unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag enable successful \r\n");
    }


//
//        uint8_t magNormalCmd[2];
//
//        magNormalCmd[0] = 0x7E;
//        magNormalCmd[1] = 0x19;





    //Put sensors in normal mode
    normalCmd.slaveAddress = 0x68;
    normalCmd.writeBuf = cmdAccNorm;
    normalCmd.writeCount = 2;
    normalCmd.readBuf = NULL;
    normalCmd.readCount = 0;

    status = I2C_transfer(handle, &normalCmd);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Acc normal cmd to CMD reg failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Acc normal cmd to CMD reg successful \r\n");
    }


    normalCmd.writeBuf = cmdGyrNorm;

    status = I2C_transfer(handle, &normalCmd);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Gyr normal cmd to CMD reg failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Gyr normal cmd to CMD reg successful \r\n");
    }

    normalCmd.writeBuf = cmdMagNorm;

    status = I2C_transfer(handle, &normalCmd);
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Mag normal cmd to CMD reg unsuccessful\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Mag normal cmd to CMD reg successful \r\n");
    }


    //A large delay to accommodate startup time. (Can be shortened if you thoroughly read datasheet)

    DELAY_MS(500);

    //Read pmu_status to confirm normal mode

    pmuNormalCheck.slaveAddress = 0x68;
    pmuNormalCheck.writeBuf = &pmu_status_reg_addr;
    pmuNormalCheck.writeCount = 1;
    pmuNormalCheck.readBuf = &pmu_status;
    pmuNormalCheck.readCount = 1;


    status = I2C_transfer(handle, &pmuNormalCheck);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("pmu_status read failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("pmu_status read successful \r\n");
    }

    uint8_t accNormMask = 0x30;
    uint8_t gyrNormMask = 0x0C;
    uint8_t magNormMask = 0x03;
    if((pmu_status & accNormMask) == 0x10)
        uartputs("Accelerometer is now in normal mode \r\n");
    else
        uartputs("Failed to put accelerometer in normal mode \r\n");

    if((pmu_status & gyrNormMask) == 0x04)
        uartputs("Gyroscope is now in normal mode \r\n");
    else
        uartputs("Failed to put gyroscope in normal mode \r\n");

    if((pmu_status & magNormMask) == 0x01)
        uartputs("Magnetometer is now in normal mode \r\n");
    else
        uartputs("Failed to put magnetometer in normal mode \r\n");



    //check if data is ready


    checkDataReady.slaveAddress = 0x68;
    checkDataReady.writeBuf = &dataReadyAddr;
    checkDataReady.writeCount = 1;
    checkDataReady.readBuf = &dataReadyStatus;
    checkDataReady.readCount = 1;

    status = I2C_transfer(handle, &checkDataReady);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("data status read failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("data status read successful \r\n");
    }

    if((dataReadyStatus & 0x80) == 0x80)
        uartputs("Accel data is ready and valid \r\n");
    if((dataReadyStatus & 0x40) == 0x40)
        uartputs("Gyro data is ready and valid \r\n");
    if((dataReadyStatus & 0x20) == 0x20)
        uartputs("Mag data is ready and valid \r\n");
    //Read accel data

    dataRead.slaveAddress = 0x68;
    dataRead.writeBuf = &data_reg_addr;
    dataRead.writeCount = 1;
    dataRead.readBuf = &data;
    dataRead.readCount = 20;

    uint16_t accelZ;
    accelZ = data[19];
    accelZ = accelZ << 8;
    accelZ = accelZ | ((uint16_t)data[18]);

    uartprintf("Accelerometer Z data : %x \r\n", accelZ);

    status = I2C_transfer(handle, &dataRead);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("data read failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("data read successful\r\n");
    }

    //Code to process and print out accel data here

    //Put all sensors in suspend mode

   suspendCmd.slaveAddress = 0x68;
   suspendCmd.writeBuf = cmdSuspendAll;
   suspendCmd.writeCount = 2;
   suspendCmd.readBuf = NULL;
   suspendCmd.readCount = 0;

   uartputs("516\r\n");
   status = I2C_transfer(handle, &suspendCmd);
   uartputs("518\r\n");
    if(!status) {
        setLed(Board_RLED, true);
        uartputs("Suspend CMD to CMD register failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("Suspend cmd to CMD register was successful\r\n");
    }



    DELAY_MS(500);

    //Read pmu_status to confirm sleep mode

    pmuSuspendCheck.slaveAddress = 0x68;
    pmuSuspendCheck.writeBuf = &pmu_status_reg_addr;
    pmuSuspendCheck.writeCount = 1;
    pmuSuspendCheck.readBuf = &pmu_status;
    pmuSuspendCheck.readCount = 1;


    status = I2C_transfer(handle, &pmuSuspendCheck);

    if(!status) {
        setLed(Board_RLED, true);
        uartputs("pmu_status read failed\r\n");
    }
    else {
        setLed(Board_GLED, true);
        uartputs("pmu_status read successful \r\n");
    }

    if((pmu_status & accNormMask) == 0x00)
        uartputs("Accelerometer is now in suspend mode \r\n");
    else
        uartputs("Failed to put accelerometer in suspend mode \r\n");

    if((pmu_status & gyrNormMask) == 0x00)
        uartputs("Gyroscope is now in suspend mode \r\n");
    else
        uartprintf("Failed to put gyroscope in suspend mode %x \r\n", pmu_status);

    I2C_close(handle);
}


/*********************************************************************
 * @fn      bmx_taskFxn
 * @return  None.
 */
static void bmx_taskFxn (UArg a0, UArg a1)
{
    uint32_t data[20];
    while(1){
        getBmxData(data);
        DELAY_MS(5000);
    }


}


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      bmxService_createTask
 *
 * @brief   Task creation function for the bmx160 application.
 *
 * @param   None.
 *
 * @return  None.
 */
void bmxService_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = bmxTaskStack;
  taskParams.stackSize = BMX_TASK_STACK_SIZE;
  //taskParams.priority = BMX_TASK_PRIORITY;

  Task_construct(&bmxTask, bmx_taskFxn, &taskParams, NULL);
}



#endif /* SERVICES_BMXSERVICE_C_ */
