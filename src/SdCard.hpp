/*
 * Sd.hpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#ifndef SD_HPP_
#define SD_HPP_

#include "SdCardHw.hpp"

#include "diskio.h"
#include "ff.h"
#include "integer.h"
#include "ffconf.h"

#include <string.h>

class SdCard
{
public:
	SdCard();
	bool mount();
	bool open(char* Filename);
	bool createFile();
	uint32_t read(char* BufferRead, uint32_t Readlen);
	uint32_t write(char* BufferWrite, uint32_t Writelen);
	void closeFile();
	void unmount();
	void sync();
	uint32_t getCounter();

private:
	FATFS FatFs;			// FatFs work area needed for each volume
	FIL FileObject;		// File object needed for each open file
	FRESULT FResult;
	bool SDMounted;
	bool FileOpened;
	char fileName[15];
	uint32_t FileCounter;

	bool createWithName(char* Filename);
};

#endif /* SD_HPP_ */
