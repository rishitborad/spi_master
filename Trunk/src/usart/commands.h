
#ifndef COMMANDS_H
#define _COMMANDS_H

void CmdVersion(uint32_t data);

void CmdBlinkLED( uint32_t data );

void CmdInitSPIPeripheral( uint32_t data );

void CmdUserSpi_ReadTest( uint32_t data );
void CmdUserSpi_SpeedTest( uint32_t data );

void CmdUserSpi_adiReadTest( uint32_t data );
void CmdUserSpi_jdBurstTest( uint32_t data );

void CmdUserSpi_WhoAmI( uint32_t data );

void CmdUserSpi_GetBoardTemp( uint32_t data );
void CmdUserSpi_GetRates( uint32_t data );
void CmdUserSpi_GetAccels( uint32_t data );
void CmdUserSpi_GetSensorValues( uint32_t data );

void CmdUserSpi_ReadRegister_TwoBytes( uint32_t data );
void CmdUserSpi_ReadRegisterHex( uint32_t data );
void CmdUserSpi_ReadRegisterHex2( uint32_t data );
void CmdUserSpi_ReadRegister( uint32_t data );
void CmdUserSpi_ReadRegister2( uint32_t data );
void CmdUserSpi_WriteRegister( uint32_t data );

void CmdUserSpi_SelfTest( uint32_t data );

void CmdUserSpi_MagAlign( uint32_t data );

void CmdUserSpi_ATP_Process( uint32_t data );
void CmdUserSpi_jdInitProcess( uint32_t data );

void CmdUserSpi_IndraTesting( uint32_t data );

void CmdUserSpi_BurstRead( uint32_t data );
void CmdUserSpi_BurstRegs( uint32_t data );

void CmdUserSpi_BurstRead2( uint32_t data );
void CmdUserSpi_BurstRead_F2( uint32_t data );

void CmdUserSpi_BurstRead_S0( uint32_t data );
void CmdUserSpi_BurstRead_S1( uint32_t data );

void CmdUserSpi_BurstRead_A1( uint32_t data );

void CmdUserSpi_BurstForLowGainAHRS( uint32_t data );

void CmdUserSpi_burstReadSwitch( uint32_t data );

void CmdUserSpi_BurstTest( uint32_t data );
void CmdUserSpi_BurstTest01( uint32_t data );

void CmdSpeed( uint32_t data );

void CmdUserSpiTestSequence01( uint32_t data );

void CmdSelfTest(uint32_t data);

void CmdUserUsart(uint32_t data);
void CmdGpioPin(uint32_t data);

void CmdUserSpi_HoldChipSelect( uint32_t data );
void CmdUserSpi_ToggleChipSelect( uint32_t data );

void CmdUserSpi_SyncOn( uint32_t data );
void CmdUserSpi_SyncOff( uint32_t data );
void CmdUserSpi_FreqRatio( uint32_t data );


void CmdUserSpi_DisplayConverted( uint32_t data );
void CmdUserSpi_DisplayHex( uint32_t data );

void CmdUserSpi_WaitForDRDY( uint32_t data );
void CmdUserSpi_DontWaitForDRDY( uint32_t data );
void CmdUserSpi_BootLoad( uint32_t data );
void CmdUserSpi_BootStatus( uint32_t data );
void CmdUserSpi_BootReset( uint32_t data );
void CmdUserSpi_CyclePower( uint32_t data );
void CmdUserSpi_CycleTest( uint32_t data );
//void SPI_baddata_test1();
//void SPI_baddata_test2();
//void SPI_baddata_test3();
//void SPI_baddata_test4();
void SPI_baddata_test5();
void oriRetention();
void repeatSelfTest();

#endif /* COMMANDS_H */