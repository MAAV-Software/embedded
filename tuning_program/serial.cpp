#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "serial.h"
#include <iostream>

SerialTTY::SerialTTY(char* ttydevice)
{
        unsigned char c='D';

        memset(&tio,0,sizeof(tio));
        tio.c_iflag=0;
        tio.c_oflag=0;
        tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
        tio.c_lflag=0;
        tio.c_cc[VMIN]=1;
        tio.c_cc[VTIME]=5;
 
        serialtty=open(ttydevice, O_RDWR | O_NONBLOCK);      
        cfsetospeed(&tio,B115200);            // 115200 baud
        cfsetispeed(&tio,B115200);            // 115200 baud
 
        tcsetattr(serialtty,TCSANOW,&tio);
	frame = data_frame_create(256);
	last = (char*)malloc(256);
	lastsz = 0;
}

void SerialTTY::send(char *buffer, int length)
{
	char dlbuf[256];
	int len = data_link_assemble_packet((uint8_t*)buffer, (uint8_t*)dlbuf, length);
	write(serialtty,dlbuf,len);
}

bool SerialTTY::recv(char &to)
{
	return read(serialtty,&to,1) > 0;
}

char SerialTTY::blockrecv()
{
	char c;
	while(read(serialtty,&c,1));
	return c;
}

bool SerialTTY::receive_and_process()
{
	char c;
	// While there is data to read in
	while(recv(c))
	{
		data_frame_push_byte(frame, (uint8_t)c);
		if(frame->state == DONE)
		{
			memcpy(last, frame->buffer, frame->size);
			lastsz = frame->size;
			data_frame_clear(frame);
			return true;
		}
		if(frame->state == START_ERR or frame->state == DATA_ERR)
		{
			data_frame_clear(frame);
		}
	}
	return false;
}

int SerialTTY::last_packet(char *to)
{
	memcpy(to, last, lastsz);
	return lastsz;
}
