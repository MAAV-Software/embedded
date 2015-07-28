/*
 * Sd.hpp
 *
 *  Created on: Jul 10, 2015
 *      Author: Zhengjie
 */

#ifndef SD_HPP_
#define SD_HPP_

#include "SdHw.hpp"

#include "sdlib/diskio.h"
#include "sdlib/ff.h"
#include "sdlib/integer.h"
#include "sdlib/ffconf.h"

class Sd
{
public:
	Sd();
	bool sdMount();
	bool sdOpen(char* Filename);
	bool sdCreate(char* Filename);
	uint32_t sdRead(char* BufferRead, uint32_t Readlen);
	uint32_t sdWrite(char* BufferWrite, uint32_t Writelen);
	void sdSync();
	void sdClose();
	void sdUnmount();

private:
	FATFS FatFs;			// FatFs work area needed for each volume
	FIL FileObject;		// File object needed for each open file
	FRESULT FResult;
	bool SDMounted;
	bool FileOpened;
};

#endif /* SD_HPP_ */
