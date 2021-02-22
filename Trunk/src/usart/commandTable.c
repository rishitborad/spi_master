/** ***************************************************************************
 * @file   commandTable.c
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * Table of serial Debug console input Commands with callback function pointers
 * and help text
 ******************************************************************************/
#include "commandLine.h" // tCommand
#include "commands.h"

#define COMMAND_TABLE_END {"",0,0,""}

// SPI Master Command Table
const tCommand gCommands[] =
{ // Command,  callback, data, help text
  { "blink", &CmdBlinkLED, 0, "Usage: blink\r\nBlink LED4 <numBlinks, 3> <ms between blinks, 250>" },

  { "output", &CmdUserUsart, 0,
  "Usage: output which string\r\nPrint characters to the user uart <which> <character string>"},

  { "boardSpeedTest", &CmdUserSpi_SpeedTest, 0, "Usage: boardSpeedTest regAddr NumOfBytes" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 65 (0x41), NumOfBytes: 4>" },

  { "speed", &CmdSpeed, 0, "Usage: speed value" \
                           "\r\n       Select SPI clock rate as specified by 'speed'" \
                           "\r\n       <default -- speed: 2" \
                            "\r\n       1: reallySlow, 2: slow, 3: faster, 4: even faster, 5: fast" },

  // SPI communication commands
  { "spiCmd01", &CmdUserSpiTestSequence01, 0, "Usage: spiCmd01\r\nWrite a test sequence to the slave and display the response" },

  { "initSpi", &CmdInitSPIPeripheral, 0, "Usage: initSpi\r\nInitialize the SPI peripheral" },

  { "ver",    &CmdVersion, 0, "Usage: ver \r\nDisplay firmware version of SPI Master"},

  // Useful commands:
  { "pin", &CmdGpioPin, 0, "Usage: pin port pin to state"},

  // Finalized versions of SPI interface code:
  { "whoami", &CmdUserSpi_WhoAmI, 0, "Usage: whoami" \
                                    "\r\n       Read from register 0x54 to get the product ID" },

  { "atp", &CmdUserSpi_ATP_Process, 0, "Usage: atp" \
                                    "\r\n       Send a series of writes to configure the system followed by reads of the" \
                                    "\r\n         register to verify that the data was written.  This is echoed to the user" \
                                    "\r\n         to confirm the write was correct.  A 'Test Successful' statement is written" \
                                    "\r\n         if all writes were successful." },

  { "spitest", &CmdUserSpi_ATP_Process, 0, "Usage: spitest" \
                                    "\r\n       Send a series of writes to configure the system followed by reads of the" \
                                    "\r\n         register to verify that the data was written.  This is echoed to the user" \
                                    "\r\n         to confirm the write was correct.  A 'Test Successful' statement is written" \
                                    "\r\n         if all writes were successful." },

  { "jdInit", &CmdUserSpi_jdInitProcess, 0, "Usage: jdInit numOfTests numOfToggles_nRst numOfToggles_nSS" \
                                    "\r\n       Initiate the JD initialization sequence 'numOfTests' times in a row." \
                                    "\r\n       Note: the number of tests performed is actually 2x the number specified as" \
                                    "\r\n       each test consists of an nSS toggle sequence followed by nRst toggle." \
                                    "\r\n       <default -- numOfTests: 3, numOfToggles_nRst: 5, numOfToggles_nSS: 5>" },

  { "j", &CmdUserSpi_jdInitProcess, 0, "Usage: j numOfTests numOfToggles_nRst numOfToggles_nSS" \
                                    "\r\n       Initiate the JD initialization sequence 'numOfTests' times in a row." \
                                    "\r\n       Note: the number of tests performed is actually 2x the number specified as" \
                                    "\r\n       each test consists of an nSS toggle sequence followed by nRst toggle." \
                                    "\r\n       <default -- numOfTests: 3, numOfToggles_nRst: 5, numOfToggles_nSS: 5>" },

  { "indraTest", &CmdUserSpi_IndraTesting, 0, "Usage: indraTest numOfTests numOfToggles_nRst numOfToggles_nSS" \
                                    "\r\n       Initiate the JD initialization sequence 'numOfTests' times in a row." \
                                    "\r\n       Note: the number of tests performed is actually 2x the number specified as" \
                                    "\r\n       each test consists of an nSS toggle sequence followed by nRst toggle." \
                                    "\r\n       <default -- numOfTests: 3, numOfToggles_nRst: 5, numOfToggles_nSS: 5>" },

  { "i", &CmdUserSpi_IndraTesting, 0, "Usage: i numOfTests numOfToggles_nRst numOfToggles_nSS" \
                                    "\r\n       Initiate the JD initialization sequence 'numOfTests' times in a row." \
                                    "\r\n       Note: the number of tests performed is actually 2x the number specified as" \
                                    "\r\n       each test consists of an nSS toggle sequence followed by nRst toggle." \
                                    "\r\n       <default -- numOfTests: 3, numOfToggles_nRst: 5, numOfToggles_nSS: 5>" },

  { "burst2", &CmdUserSpi_BurstRead2, 0, "Usage: burst2 numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a burst-read to get sensor information (status/rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "burst_F2", &CmdUserSpi_BurstRead_F2, 0, "Usage: burst_f2 numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a burst-read to get sensor information (status/rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "burst_S0", &CmdUserSpi_BurstRead_S0, 0, "Usage: burst_s0 numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a burst-read to get sensor information (status/rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "ahrsMode", &CmdUserSpi_BurstForLowGainAHRS, 0, "Usage: ahrsMode" \
                                    "\r\n       Perform a burst-read to get AHRS mode information (HG/LG)" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "burstTest", &CmdUserSpi_burstReadSwitch, 0, "Usage: burstTest" \
                                    "\r\n       Perform a series of burst-reads to test switching between various" \
                                    "\r\n       burst commands (JD Burst, S0 Burst, S1 Burst, A1 Burst, F1 Burst)" \
                                    "\r\n       then repeats the test" \
                                    "\r\n       <default -- 5 tests, 2 MHz SPI clock" },

  { "magAlign", &CmdUserSpi_MagAlign, 0, "Usage: magAlign" \
                                    "\r\n       Perform a magnetic alignment" \
                                    "\r\n       <default -- 5 tests, 2 MHz SPI clock" },

  { "temp", &CmdUserSpi_GetBoardTemp, 0, "Usage: temp numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a read of temperature-sensor information" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "accels", &CmdUserSpi_GetAccels, 0, "Usage: accels numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a read of accelerometer information" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "sensors", &CmdUserSpi_GetSensorValues, 0, "Usage: sensors numberOfSamples delayBetweenSamples" \
                                    "\r\n       Perform a non-burst-mode read of sensor information (rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, delayBetweenSamples: 500 msec>" },

  { "read", &CmdUserSpi_ReadRegister, 0, "Usage: read regAddr NumOfBytes" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 65 (0x41), NumOfBytes: 4>" },

  { "r", &CmdUserSpi_ReadRegister, 0, "Usage: r regAddr NumOfBytes" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 65 (0x41), NumOfBytes: 4>" },

  { "rHex", &CmdUserSpi_ReadRegisterHex, 0, "Usage: rHex regAddr NumOfBytes" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 65 (0x41), NumOfBytes: 4>" },

  { "rHex2", &CmdUserSpi_ReadRegisterHex2, 0, "Usage: rHex2 regAddr NumOfBytes" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 65 (0x41), NumOfBytes: 4>" },



  // Updated functions and descriptions follow
  { "selfTest", &CmdUserSpi_SelfTest, 0, "Usage: selfTest numberOfTests" \
                                    "\r\n       Perform a self-test of the system" \
                                    "\r\n       <default -- numberOfTests: 10, delay between checks: 5 msec>" },

  { "burst", &CmdUserSpi_BurstRead, 0, "Usage: burst numberOfSamples OutputDataRate" \
                                    "\r\n       Perform a burst-read to get sensor information (status/rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, OutputDataRate: 2 Hz>" },

  { "regs", &CmdUserSpi_BurstRegs, 0, "Usage:   burst startAddress NumberOfBytes" \
                                    "\r\n       Perform a burst-read of registers)" \
                                    "\r\n       <default -- numberOfSamples: 10, OutputDataRate: 2 Hz>" },

  { "burst_A1", &CmdUserSpi_BurstRead_A1, 0, "Usage: burst_a1 numberOfSamples OutputDataRate" \
                                    "\r\n       Perform a burst-read of the A1 packet to get AHRS and sensor information" \
                                    "\r\n       <default -- numberOfSamples: 10, OutputDataRate: 2 Hz>" },

  { "burst_S1", &CmdUserSpi_BurstRead_S1, 0, "Usage: burst_s1 numberOfSamples OutputDataRate" \
                                    "\r\n       Perform a burst-read of the S1 packet to get sensor information (status/rates/accels/temp)" \
                                    "\r\n       <default -- numberOfSamples: 10, OutputDataRate: 2 Hz>" },

  { "rates", &CmdUserSpi_GetRates, 0, "Usage: rates numberOfSamples OutputDataRate" \
                                    "\r\n       Perform a read of rate-sensor registers" \
                                    "\r\n       <default -- numberOfSamples: 10, OutputDataRate: 2 Hz>" },

  { "readHex", &CmdUserSpi_ReadRegisterHex, 0, "Usage: readHex regAddr NumOfBytes NumberOfReads OutputDataRate" \
                                    "\r\n       Read from 'regAddr' the number of bytes specified by 'NumOfBytes'" \
                                    "\r\n       <default -- regAddr: 4 (0x04), NumOfWords: 1, NumOfReadsd: 10, ODR: 2 Hz>" },

  { "write", &CmdUserSpi_WriteRegister, 0, "Usage: write regAddr msg" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "justATest", &CmdUserSpi_WriteRegister, 0, "Usage: write regAddr msg" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "toggleCS", &CmdUserSpi_ToggleChipSelect, 0, "Usage: toggleCS" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "holdCS", &CmdUserSpi_HoldChipSelect, 0, "Usage: holdCS" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "displayConverted", &CmdUserSpi_DisplayConverted, 0, "Usage: displayConverted" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "displayHex", &CmdUserSpi_DisplayHex, 0, "Usage: displayHex" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "waitDRDY", &CmdUserSpi_WaitForDRDY, 0, "Usage: waitDRDY" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },

  { "bootLoad", &CmdUserSpi_BootLoad,    0, "Usage: bootLoad" \
                                     "\r\n       Load application image via SPI interface"},
                                     
  { "bootReset", &CmdUserSpi_BootReset,    0, "Usage: bootReset" \
                                     "\r\n       Reset device via spi interface in boot mode"},

  { "bootStatus", &CmdUserSpi_BootStatus,    0, "Usage: bootStatus" \
                                     "\r\n       Reads status register in boot mode"},
  
  { "dontWaitDRDY", &CmdUserSpi_DontWaitForDRDY, 0, "Usage: dontWaitDRDY" \
                                     "\r\n       Write a single byte, 'msg', to 'regAddr'" \
                                     "\r\n       <default -- regAddr: 0 (0x00), msg: 15 (0x0F)>" },
  { "syncOn",       &CmdUserSpi_SyncOn,          0,  "Usage: syncOn" \
                                     "\r\n       Start Toggling sync pin" \
                                     "\r\n       <default -- off>" },

  { "syncOff",      &CmdUserSpi_SyncOff,         0,  "Usage: syncOff" \
                                     "\r\n       Stop Toggling sync pin" \
                                     "\r\n       <default -- off>" },

  { "syncFreq",     &CmdUserSpi_FreqRatio,       0,  "Usage: syncFreq <freq>" \
                                     "\r\n       Set sync frequency" \
                                     "\r\n       <default -- 1000>" },
  { "cyclePower",   &CmdUserSpi_CyclePower,       0,  "Usage: cyclePower <numCycles>" \
                                     "\r\n       Set sync frequency" \
                                     "\r\n       <default -- 100>" },

  { "cycleTest",   &CmdUserSpi_CycleTest,        0,  "Usage: cycleTest <numCycles>" \
                                     "\r\n       <default -- 100>" },
  
  //{ "baddatatest1",   &SPI_baddata_test1,        0,  "Usage: baddatatest1 <numCycles>" \
                                     "\r\n       <default -- 100>" },
                                     
  //{ "baddatatest2",   &SPI_baddata_test2,        0,  "Usage: baddatatest2 <numCycles>" \
                                     "\r\n       <default -- 100>" },
                                     
  //{ "baddatatest3",   &SPI_baddata_test3,        0,  "Usage: baddatatest3 <numCycles>" \
                                     "\r\n       <default -- 100>" },
                                     
  //{ "baddatatest4",   &SPI_baddata_test4,        0,  "Usage: baddatatest4 <numCycles>" \
                                     "\r\n       <default -- 100>" },
                                     
  { "baddatatest5",   &SPI_baddata_test5,        0,  "Usage: baddatatest5 <numCycles>" \
                                     "\r\n       <default -- 100>" },
  { "oriRetention",   &oriRetention,             0,  "Usage: oriRetention" \
                                     "\r\n       <default -- 100>" },
  { "repeatSelfTest",   &repeatSelfTest,         0,  "Usage: repeatSelfTest" \
                                     "\r\n       <default -- 100>" },
                                     
  COMMAND_TABLE_END  //MUST BE LAST!!!
};

