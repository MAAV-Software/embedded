/*
 * Sd.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include <stdlib.h>
#include "SdCard.hpp"
#include "EEPROM.h"
#include "Apeshit.hpp"

SdCard::SdCard()
{
//	for (int i = 0; i < SD_BUFFER_SIZE; i++) buffer[i] = 0;
	// THESE ARE FOR 2015 AND 2016 SIGNAL BOARDS (EXCEPT FOR FSS)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	ROM_GPIOPinConfigure(GPIO_PD0_SSI1CLK);
//	ROM_GPIOPinConfigure(GPIO_PD1_SSI1FSS); // THIS IS THE FSS. COMMENT OUT THIS LINE FOR 2015 SIGNAL BOARD
	ROM_GPIOPinConfigure(GPIO_PD2_SSI1RX);
	ROM_GPIOPinConfigure(GPIO_PD3_SSI1TX);
	SDMounted = false;
	FileOpened = false;
	FResult = FR_OK;
	if(!mount())
	{
	    goApeshit();
	}
	FileCounter = Read_LOG_EEPROM();
}

bool SdCard::mount()
{
	if (SDMounted) return SDMounted;
	FResult = f_mount(&FatFs, "", 1);
	switch(FResult)
	{
		case FR_OK:
//			UARTprintf("SD Card mounted successfully\n");
			SDMounted = true;
			break;
		case FR_INVALID_DRIVE:
//			fatalError("ERROR: Invalid drive number\n");
//			break;
		case FR_DISK_ERR:
//			fatalError("ERROR: DiskIO error - Check hardware!\n");
//			break;
		case FR_NOT_READY:
//			ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED);
//			fatalError("ERROR: Medium removal or disk_initialize\n");
//			break;
		case FR_NO_FILESYSTEM:
//			fatalError("ERROR: No valid FAT volume on drive\n");
//			break;
		default:
			SDMounted = false;
			break;
	}
	return SDMounted;

}

void SdCard::unmount()
{
	FResult = f_mount(0, "", 1);
}

bool SdCard::open(char* Filename)
{
	if (SDMounted)
	{
		FResult = f_open(&FileObject, Filename, FA_WRITE | FA_OPEN_EXISTING);
		switch(FResult)
		{
			case FR_OK:
				FileOpened = true;
				break;
			default:
				FileOpened = false;
				break;
		}
	}
	return FileOpened;
}

bool SdCard::createWithName(char* Filename)
{
	if (SDMounted)
	{
		FResult = f_open(&FileObject, Filename, FA_WRITE | FA_CREATE_ALWAYS);
		switch(FResult)
		{
			case FR_OK:
				FileOpened = true;
				break;
			default:
				FileOpened = false;
				break;
		}
	}
	return FileOpened;
}

bool SdCard::createFile()
{
	FileCounter = Read_LOG_EEPROM();
	if (FileCounter >= 	MAX_FILE_COUNT)
	{
		FileCounter = 0;
	}
	snprintf(fileName, sizeof(fileName), "log%u.txt", FileCounter++);
	Write_LOG_EEPROM(FileCounter);

	return createWithName(fileName);
}

void SdCard::closeFile()
{
	f_close(&FileObject);
}

uint32_t SdCard::read(char* BufferRead, uint32_t Readlen)
{
	uint32_t ActualReadLen = 0;
	if (FileOpened)
	{
		FResult = f_read(&FileObject, BufferRead, Readlen, &ActualReadLen);
	}
	return ActualReadLen;
}

uint32_t SdCard::write(char* BufferWrite, uint32_t WriteLen)
{
	uint32_t ActualWriteLen = 0;
	if (FileOpened)
	{
		f_write(&FileObject, BufferWrite, WriteLen, &ActualWriteLen);
	}
	return ActualWriteLen;
}

void SdCard::sync()
{
	f_sync(&FileObject);
}
