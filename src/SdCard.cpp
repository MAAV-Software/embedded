/*
 * 	SdCard.cpp
 *
 *  Created on: Jul 10, 2015
 *
 */

#include <stdlib.h>
#include "SdCard.hpp"
#include "EEPROM.h"
#include "Apeshit.hpp"

/**
 * 	@brief Default Constructor
 */
SdCard::SdCard()
{
//	for (int i = 0; i < SD_BUFFER_SIZE; i++) buffer[i] = 0;
	// THESE ARE FOR 2015 AND 2016 SIGNAL BOARDS (EXCEPT FOR FSS)
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
	ROM_GPIOPinConfigure(GPIO_PD0_SSI1CLK);
	ROM_GPIOPinConfigure(GPIO_PD1_SSI1FSS); // THIS IS THE FSS. COMMENT OUT THIS LINE FOR 2015 SIGNAL BOARD
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
/**
 * 	@brief mounts the SD card.
 *	@returns true on success
 */
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
/**
 * @brief Unmounts the sd card
 */
void SdCard::unmount()
{
	FResult = f_mount(0, "", 1);
}
/**
 * @brief Opens the file at the supplied file location
 * @param Filename A char array of the filename to open
 * @returns true on success
 */
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
/**
 * 	@brief creates a new file with the given name
 *	@param Filename the name of the file to create
 *	@returns true on success
 */
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
/**
 * @brief creates a log file following the last log file made
 * @details For example, if the last log is log88.txt, this one will
 * be named log89.txt
 * @returns true on success
 */
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
/**
 * @brief closes the currently open file
 */
void SdCard::closeFile()
{
	f_close(&FileObject);
}
/**
 * @brief reads the given number of characters to the passed buffer
 * @param BufferRead the buffer to read to
 * @param Readlen the length of the passed buffer
 * @returns the actual length of characters read, as the file may
 * not contain as many characters as Readlen requested
 */
uint32_t SdCard::read(char* BufferRead, uint32_t Readlen)
{
	uint32_t ActualReadLen = 0;
	if (FileOpened)
	{
		FResult = f_read(&FileObject, BufferRead, Readlen, &ActualReadLen);
	}
	return ActualReadLen;
}
/**
 * @brief writes the passed buffer to the currently open file
 * @param BufferWrite a pointer to the buffer to write to the file
 * @param WriteLen the number of characters in the buffer
 * @returns the actual length of characters written.
 */
uint32_t SdCard::write(char* BufferWrite, uint32_t WriteLen)
{
	uint32_t ActualWriteLen = 0;
	if (FileOpened)
	{
		f_write(&FileObject, BufferWrite, WriteLen, &ActualWriteLen);
	}
	return ActualWriteLen;
}
/**
 * @brief flushes the buffer to the file. Call this before shutting down
 */
void SdCard::sync()
{
	f_sync(&FileObject);
}
