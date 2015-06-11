#include <iostream>
#include <libusb.h>
#include <unistd.h>
#include <cstdio>
#include <math.h>

using namespace std;

void Serial_Init();
void Connect(uint16_t vendor_id, uint16_t product_id);
void Check_Kernel_Driver();
void Claim_Interface();
void Send_Data(unsigned char, unsigned char *);
int Receive_Data(unsigned char);
void Parse_Data(unsigned char *);
float Bytes_2_Float(unsigned char *, unsigned int);
int Bytes_2_Int(unsigned char*, unsigned int);

//pointer to pointer of device, used to retrieve a list of devices
libusb_device **devs; 

//a device handle
libusb_device_handle *dev_handle;
 
//a libusb session
libusb_context *ctx = NULL;

//holding number of devices in list
ssize_t cnt;
	
//The actual length should be 79.
unsigned char outputBytes[128];

int main() {
	
	Serial_Init();
	
	//These are the vendor_id and product_id on my laptop, checking using "lsusb"
	Connect(0x199b, 0x3065);
		
	//free the list, unref the devices in it	
	libusb_free_device_list(devs, 1); 
	
	Check_Kernel_Driver();
	
	Claim_Interface();
	
	//Address we care about: from the microstrain datasheet
	unsigned char address = 0xCC; 
	
	//Run 10 secs
	unsigned int running_time = 60*2;
	//Sleep time in useconds- 0.01sec = 0.01*1000*1000 = 10000 usec;
	unsigned int sleep_time = 10000;
	unsigned int count = 0;
	cout<<"Running for "<<running_time<<" sec(s)"<<endl;
	
	//Create and Open log.txt
	freopen("log.txt", "w", stdout);
	
	while(count<int(running_time/0.01))
	{		
		//The endpoint could be got by using "lsusb -v"
		Send_Data(0x03, &address);		
		int data_length = Receive_Data(0x81);
		
		//Should use Checksum to check the data. here is just for simplicity
		if (data_length == 79) Parse_Data(outputBytes);
		
		//Delay sometime for the next communication
		usleep(sleep_time);
		count++;
		
	}
	
	//close the log file
	fclose (stdout);
	
	return 0;
}

void Serial_Init(void)
{
	//initialize the library for the session we just declared
	int r = libusb_init(&ctx); 
	
	if(r < 0) {
		cout<<"Init Error "<<r<<endl;
	}
	
	//set verbosity level,suggested in the documentation
	libusb_set_debug(ctx, LIBUSB_LOG_LEVEL_WARNING); 
	
	//get the list of devices
	cnt = libusb_get_device_list(ctx, &devs); 
	
	if(cnt < 0) {
		cout<<"Get Device Error"<<endl;
	}
	
	cout<<cnt<<": Devices in list."<<endl;
}

void Connect(uint16_t vendor_id, uint16_t product_id)
{
	//vendorID and productID
	dev_handle = libusb_open_device_with_vid_pid(ctx, vendor_id, product_id);
	
	if(dev_handle == NULL)
		cout<<"Cannot open device"<<endl;
	else
		cout<<"Device Opened"<<endl;
}

void Check_Kernel_Driver(void)
{
	//find out if kernel driver is attached
	if(libusb_kernel_driver_active(dev_handle, 0) == 1) { 
		
		cout<<"Kernel Driver Active"<<endl;
		
		//detach it
		if(libusb_detach_kernel_driver(dev_handle, 0) == 0) 
			cout<<"Kernel Driver Detached!"<<endl;
	}
}

void Claim_Interface(void)
{	
	//claim interface 0 (the first) of device
	int r = libusb_claim_interface(dev_handle, 0); 
	
	if(r < 0) {
		cout<<"Cannot Claim Interface"<<endl;
	}
	cout<<"Claimed Interface"<<endl;
	
}

void Send_Data(unsigned char endpoint, unsigned char *address)
{
	//used to find out how many bytes were written
	int actual;
	
	int r = libusb_bulk_transfer(dev_handle, endpoint, address, 1, &actual, 0);
	
	//successfully 
	if(r == 0) {		
		//cout<<"Sending Successfully!"<<endl;
	}
	else
	{
		cout<<"Send Error"<<endl;

		//release the claimed interface
		r = libusb_release_interface(dev_handle, 0); 
	
		if(r!=0) {
			cout<<"Cannot Release Interface"<<endl;
		}
		
		cout<<"Released Interface"<<endl;
		
		//close the device we opened
		libusb_close(dev_handle); 
		//needs to be called to end the
		libusb_exit(ctx); 
	} 
}

int Receive_Data(unsigned char endpoint)
{	
	//used to find out how many bytes were received
	int actual;	
	
	int r = libusb_bulk_transfer(dev_handle, endpoint, outputBytes, sizeof(outputBytes), &actual, 0); 
	
	if (r == 0) {
		//cout<<"Receiving Successfully!"<<endl<<"What i get is :\n"<<outputBytes<<endl;
		//cout<<"Received Length: "<<actual<<endl;	
		return actual;
	}
	else {
		cout<<"Receive Error"<<endl;

		r = libusb_release_interface(dev_handle, 0); //release the claimed interface
	
		if(r!=0) {
			cout<<"Cannot Release Interface"<<endl;
		}
		
		cout<<"Released Interface"<<endl;
		
		//close the device we opened
		libusb_close(dev_handle); 
		//needs to be called to end the
		libusb_exit(ctx); 
		return 0;
	}
}

void Parse_Data(unsigned char * data)
{ 
		
	//Big-Endian?
	char FirstByte = data[0];
	float AccX = Bytes_2_Float(data, 1);
	float AccY = Bytes_2_Float(data, 5);
	float AccZ = Bytes_2_Float(data, 9);
	float AngRateX = Bytes_2_Float(data, 13);
	float AngRateY = Bytes_2_Float(data, 17);
	float AngRateZ = Bytes_2_Float(data, 21);
	float MagX = Bytes_2_Float(data, 25);
	float MagY = Bytes_2_Float(data, 29);
	float MagZ = Bytes_2_Float(data, 33);
	float M11 = Bytes_2_Float(data, 37);
	float M12 = Bytes_2_Float(data, 41);
	float M13 = Bytes_2_Float(data, 45);
	float M21 = Bytes_2_Float(data, 49);
	float M22 = Bytes_2_Float(data, 53);
	float M23 = Bytes_2_Float(data, 57);
	float M31 = Bytes_2_Float(data, 61);
	float M32 = Bytes_2_Float(data, 65);
	float M33 = Bytes_2_Float(data, 69);
	int32_t Timer = Bytes_2_Int(data, 73);

	float Roll = asin(-M13);
	float Pitch = atan2(M23, M33);
	float Yaw = atan2(M12, M11);
	
	//cout<<"AccX = "<<AccX<<endl;
	//cout<<"AccY = "<<AccY<<endl;
	//cout<<"AccZ = "<<AccZ<<endl;
	//cout<<"AngRateX = "<<AngRateX<<endl;
	//cout<<"AngRateY = "<<AngRateY<<endl;
	//cout<<"AngRateZ = "<<AngRateZ<<endl;
	//cout<<"Timer = "<<Timer/62500<<endl;
	cout<<AccX<<" "<<AccY<<" "<<AccZ<<" "<<AngRateX<<" "<<AngRateY<<" "<<AngRateZ<<" "<<Roll<<" "<<Pitch<<" "<<Yaw<<" "<<Timer<<endl;
	
}

float Bytes_2_Float(unsigned char *raw, unsigned int i)
{
	union B2F{
	   unsigned char buf[4];
	   float number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];
	
	return data.number;
}

int Bytes_2_Int(unsigned char *raw, unsigned int i)
{
	union B2I{
	   unsigned char buf[4];
	   int number;
	}data;

	data.buf[0] = raw[i+3];
	data.buf[1] = raw[i+2];
	data.buf[2] = raw[i+1];
	data.buf[3] = raw[i+0];
	
	return data.number;
}

// sudo apt-get install libusb-dev
// g++ -I/usr/include/libusb-1.0/ -c micro_strain_test.cpp 
// g++ -o micro_strain_test micro_strain_test.o -lusb-1.0 
