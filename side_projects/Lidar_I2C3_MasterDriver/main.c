// June 2015, Zhengjie Cui

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"
#include "sensorlib/i2cm_drv.h"

// Define LIDAR I2C Address.
#define LIDAR_I2C_ADDRESS      	0x62

// The states of the LIDAR state machine.
#define LIDAR_STATE_IDLE      	0           // State machine is idle
#define LIDAR_STATE_INIT      	1           // Waiting for initialization
#define LIDAR_STATE_READ      	2           // Waiting for read
#define LIDAR_STATE_WRITE     	3           // Waiting for write
#define LIDAR_STATE_RMW       	4           // Waiting for read-modify-write
#define LIDAR_STATE_REQ_DIS 	5           // Requested Distance
#define LIDAR_STATE_WAIT_DIS 	6           // Waiting for Distance ready
#define LIDAR_STATE_READ_DIS 	7           // Reading Distance value

//*****************************************************************************
#define LIDAR_START_MEA     	0x00        // Soft Start
#define LIDAR_START_CRT 		0x04        // Request soft start

// The following are defines for the LIDAR register addresses.
#define LIDAR_FULL_READ 		0x8F        // FULL read register

typedef struct
{
    // The pointer to the I2C master interface instance used to communicate with the LIDAR.
    tI2CMInstance *psI2CInst;

    // The I2C address of the LIDAR.
    uint8_t ui8Addr;

    // The state of the state machine used while accessing the LIDAR.
    uint8_t ui8State;

    // The sampling mode to be used by the LIDAR.
    uint8_t ui8Mode;

    // The new sampling mode, which is used when a register write succeeds.
    uint8_t ui8NewMode;

    // The data buffer used for sending/receiving data to/from the LIDAR.
    uint8_t pui8Data[2];

    // The function that is called when the current request has completed processing.
    tSensorCallback *pfnCallback;

    // The pointer provided to the callback function.
    void *pvCallbackData;

    // A union of structures that are used for read, write and
    // read-modify-write operations.  Since only one operation can be active at
    // a time, it is safe to re-use the memory in this manner.
    union
    {
        // A buffer used to store the write portion of a register read.  This
        // is also used to read back the calibration data from the device.
        uint8_t pui8Buffer[3];

        // The write state used to write register values.
        tI2CMWrite8 sWriteState;

        // The read-modify-write state used to modify register values.
        tI2CMReadModifyWrite8 sReadModifyWriteState;
    }
    uCommand;
}
tLIDAR;

// Global instance structure for the I2C master driver.
tI2CMInstance g_sI2CInst;

// Global instance structure for the LIDAR sensor driver.
tLIDAR g_sLIDARInst;

// Global new data flag to alert main that LIDAR data is ready.
volatile uint_fast8_t g_vui8DataFlag;

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

// LIDAR Sensor callback function.  Called at the end of LIDAR sensor driver transactions. This is called from I2C interrupt context.
// Therefore, we just set a flag and let main do the bulk of the computations and display.
//*****************************************************************************
void LIDARAppCallback(void* pvCallbackData, uint_fast8_t ui8Status)
{
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8DataFlag = 1;
    }
}

// Called by the NVIC as a result of I2C3 Interrupt. I2C3 is the I2C connection to the LIDAR.
//*****************************************************************************
void
LIDARI2CIntHandler(void)
{
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    I2CMIntHandler(&g_sI2CInst);
}

// Configure the UART and its pins.  This must be called before UARTprintf().
void
ConfigureUART(void)
{
    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 115200, 16000000);

}
// The callback function that is called when I2C transations to/from the LIDAR have completed.
static void
LIDARCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    tLIDAR *psInst;

    // Convert the instance data into a pointer to a tLIDAR structure.
    psInst = pvCallbackData;

    // If the I2C master driver encountered a failure, force the state machine
    // to the idle state (which will also result in a callback to propagate the error).
    if(ui8Status != I2CM_STATUS_SUCCESS)
    {
        psInst->ui8State = LIDAR_STATE_IDLE;
    }

    // Determine the current state of the LIDAR state machine.
    switch(psInst->ui8State)
    {
        // All states that trivially transition to IDLE, and all unknown states.
        case LIDAR_STATE_READ_DIS:
        default:
        {
            // The state machine is now idle.
            psInst->ui8State = LIDAR_STATE_IDLE;
            break;
        }

        // The first step of initialization has just completed.
        case LIDAR_STATE_INIT:
        {

            break;
        }

        // The distance has been requested.
        case LIDAR_STATE_REQ_DIS:
        {
            // Read the control register to see if the distance reading is available.
            I2CMRead(psInst->psI2CInst, psInst->ui8Addr,
                     psInst->uCommand.pui8Buffer + 2, 1,
                     psInst->pui8Data, 2, LIDARCallback,
                     psInst);

            // Move to the wait for distance state.
            psInst->ui8State = LIDAR_STATE_READ_DIS;

            break;
        }

    }

    // See if the state machine is now idle and there is a callback function.
    if((psInst->ui8State == LIDAR_STATE_IDLE) && psInst->pfnCallback)
    {
        // Call the application-supplied callback function.
        psInst->pfnCallback(psInst->pvCallbackData, ui8Status);
    }
}

uint_fast8_t
LIDARInit(tLIDAR *psInst, tI2CMInstance *psI2CInst, uint_fast8_t ui8I2CAddr,
           tSensorCallback *pfnCallback, void *pvCallbackData)
{
    // Initialize the LIDAR instance structure.
    psInst->psI2CInst = psI2CInst;
    psInst->ui8Addr = ui8I2CAddr;
    psInst->ui8State = LIDAR_STATE_INIT;
    psInst->ui8Mode = 0;
    psInst->ui8NewMode = 0;

    // Save the callback information.
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    // Perform a initial request to the LIDAR.
    psInst->uCommand.pui8Buffer[0] = LIDAR_START_MEA;
    psInst->uCommand.pui8Buffer[1] = LIDAR_START_CRT;

    if(I2CMWrite(psI2CInst, ui8I2CAddr, psInst->uCommand.pui8Buffer, 2, LIDARCallback, psInst) == 0)
    {
        // The I2C write failed, so move to the idle state and return a failure.
        psInst->ui8State = LIDAR_STATE_IDLE;
        return(0);
    }

    // Success.
    return(1);
}

uint_fast8_t
LIDARDataRead(tLIDAR *psInst, tSensorCallback *pfnCallback, void *pvCallbackData)
{
    // Return a failure if the LIDAR driver is not idle (in other words, there
    // is already an outstanding request to the LIDAR).
    if(psInst->ui8State != LIDAR_STATE_IDLE)
    {
        return(0);
    }

    // Save the callback information.
    psInst->pfnCallback = pfnCallback;
    psInst->pvCallbackData = pvCallbackData;

    // Move the state machine to the Distance reading request state.
    psInst->ui8State = LIDAR_STATE_REQ_DIS;

    // Request the Distance reading from the LIDAR.
    psInst->uCommand.pui8Buffer[0] = LIDAR_START_MEA;
    psInst->uCommand.pui8Buffer[1] = LIDAR_START_CRT;
    psInst->uCommand.pui8Buffer[2] = LIDAR_FULL_READ;

    if(I2CMWrite(psInst->psI2CInst, psInst->ui8Addr, psInst->uCommand.pui8Buffer, 2, LIDARCallback, psInst) == 0)
    {
        // The I2C write failed, so move to the idle state and return a failure.
        psInst->ui8State = LIDAR_STATE_IDLE;
        return(0);
    }

    // Success.
    return(1);
}

void
LIDARDataDistanceGetRaw(tLIDAR *psInst, uint_fast16_t *pui16Distance)
{
    // Return the raw Distance value.
    *pui16Distance = (psInst->pui8Data[0] << 8) | psInst->pui8Data[1];
}

int main(void)
{
    uint32_t uDistance;

    // Setup the system clock to run at 40 MHz from PLL with crystal reference
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Initialize the UART.
    ConfigureUART();

    // Print the welcome message to the terminal.
    UARTprintf("LIDAR Start Testing\n");

    // The I2C3 peripheral must be enabled before use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    // Configure the pin muxing for I2C3 functions on port D0 and D1.
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    // Initialize the GPIO for the LED.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);

    I2CIntRegister(I2C3_BASE, LIDARI2CIntHandler);

    // Enable interrupts to the processor.
    IntMasterEnable();

    // Initialize the I2C3 peripheral. Fill Tx and Rx with 0xff
    I2CMInit(&g_sI2CInst, I2C3_BASE, INT_I2C3, 0xff, 0xff, SysCtlClockGet());

    // Initialize the LIDAR.
    LIDARInit(&g_sLIDARInst, &g_sI2CInst, LIDAR_I2C_ADDRESS, LIDARAppCallback, &g_sLIDARInst);

    // Wait for initialization callback to indicate reset request is complete.
    while(g_vui8DataFlag == 0)
    {
        // Wait for I2C Transactions to complete.
    }

    // Reset the data ready flag
    g_vui8DataFlag = 0;

    // Begin the data collection and printing.  Loop Forever.
    while(1)
    {
        // Read the data from the LIDAR over I2C.  This command starts a
        // Distance measurement. When measurement if complete and in the local
        // buffer then the application callback is called from the I2C
        // interrupt context.  Polling is done on I2C interrupts allowing
        // processor to continue doing other tasks as needed.
        LIDARDataRead(&g_sLIDARInst, LIDARAppCallback, &g_sLIDARInst);

        // Wait for the new data set to be available.
        while(g_vui8DataFlag == 0);

        // Reset the data ready flag.
        g_vui8DataFlag = 0;

        // Get a local copy of the latest Distance data in float format.
        LIDARDataDistanceGetRaw(&g_sLIDARInst, &uDistance);

        // Print Distance with three digits of decimal precision.
        UARTprintf("Distance %u\t\t", uDistance);

        // Print new line.
        UARTprintf("\n");

        // Delay to keep printing speed reasonable. About 50 milliseconds.
        SysCtlDelay(SysCtlClockGet() / (1000 * 3) * 50);

    }//while end
	
//	return 0;
}
