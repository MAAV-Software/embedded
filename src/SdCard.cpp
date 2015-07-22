/*
 * Sd.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#include "SdCard.hpp"

SdCard::SdCard()
{
//	for (int i = 0; i < SD_BUFFER_SIZE; i++) buffer[i] = 0;
	SDMounted = false;
	FileOpened = false;
	FResult = FR_OK;
	mount();
}

bool SdCard::mount()
{
	FResult = f_mount(&FatFs, "", 1);
	switch(FResult)
	{
		case FR_OK:
//			UARTprintf("SD Card mounted successfully\n");
			SDMounted = true;
			break;
//		case FR_INVALID_DRIVE:
//			fatalError("ERROR: Invalid drive number\n");
//			break;
//		case FR_DISK_ERR:
//			fatalError("ERROR: DiskIO error - Check hardware!\n");
//			break;
//		case FR_NOT_READY:
//			ROM_GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE, LED_RED);
//			fatalError("ERROR: Medium removal or disk_initialize\n");
//			break;
//		case FR_NO_FILESYSTEM:
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

bool SdCard::createFile(char* Filename)
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
