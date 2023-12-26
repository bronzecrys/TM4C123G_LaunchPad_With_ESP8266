#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include <string.h>
#include "utils/uartstdio.h"
#include "inc/hw_ints.h"
#include "driverlib/i2c.h"
#include "driverlib/timer.h"
#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
// ***** DEFINES  ********************* //
#define TARGET_IP_ADDRESS "192.168.0.2"
#define TARGET_PORT 12345
#define BUFFER_SIZE 5000
#define TSL2591_ADDRESS        0x29
#define COMMAND_BIT            0xA0
#define ENABLE_REGISTER        0x00
#define ENABLE_POWERON         0x01
#define ENABLE_AEN             0x02
#define ID_REGISTER            0x12
#define TSL2591_VISIBLE 2 //channel 0 - channel 1
#define TSL2591_INFRARED 1
#define TSL2591_FULLSPECTRUM 0
#define TSL2591_ADDR 0x29
#define TSL2591_READBIT 0x01
#define TSL2591_COMMAND_BIT 0xA0
#define TSL2591_CLEAR_INT 0xE7
#define TSL2591_TEST_INT 0xE4
#define TSL2591_WORD_BIT 0x20
#define TSL2591_BLOCK_BIT 0x10
#define TSL2591_ENABLE_POWEROFF 0x00
#define TSL2591_ENABLE_POWERON 0x01
#define TSL2591_ENABLE_AEN 0x02
#define TSL2591_ENABLE_AIEN 0x10
#define TSL2591_ENABLE_NPIEN 0x80
#define TSL2591_LUX_DF 408.0F
#define TSL2591_LUX_COEFB 1.64F
#define TSL2591_LUX_COEFC 0.59F
#define TSL2591_LUX_COEFD 0.86F

uint8_t
TSL2591_REGISTER_ENABLE = 0x00,
TSL2591_REGISTER_CONTROL = 0x01,
TSL2591_REGISTER_THRESHOLD_AILTL = 0x04,
TSL2591_REGISTER_THRESHOLD_AILTH = 0x05,
TSL2591_REGISTER_THRESHOLD_AIHTL = 0x06,
TSL2591_REGISTER_THRESHOLD_AIHTH = 0x07,
TSL2591_REGISTER_THRESHOLD_NPAILTL = 0x08,
TSL2591_REGISTER_THRESHOLD_NPAILTH = 0x09,
TSL2591_REGISTER_THRESHOLD_NPAIHTL = 0x0A,
TSL2591_REGISTER_THRESHOLD_NPAIHTH = 0x0B,
TSL2591_REGISTER_PERSIST_FILTER = 0x0C,
TSL2591_REGISTER_PACKAGE_PID = 0x11,
TSL2591_REGISTER_DEVICE_ID = 0x12,
TSL2591_REGISTER_DEVICE_STATUS = 0x13,
TSL2591_REGISTER_CHAN0_LOW = 0x14,
TSL2591_REGISTER_CHAN0_HIGH = 0x15,
TSL2591_REGISTER_CHAN1_LOW = 0x16,
TSL2591_REGISTER_CHAN1_HIGH = 0x17,
TSL2591_INTEGRATIONTIME_100MS = 0x00,
TSL2591_INTEGRATIONTIME_200MS = 0x01,
TSL2591_INTEGRATIONTIME_300MS = 0x02,
TSL2591_INTEGRATIONTIME_400MS = 0x03,
TSL2591_INTEGRATIONTIME_500MS = 0x04,
TSL2591_INTEGRATIONTIME_600MS = 0x05,
// bit 7:4:0
TSL2591_PERSIST_EVERY = 0x00,
TSL2591_PERSIST_ANY = 0x01,
TSL2591_PERSIST_2 = 0x02,
TSL2591_PERSIST_3 = 0x03,
TSL2591_PERSIST_5 = 0x04,
TSL2591_PERSIST_10 = 0x05,
TSL2591_PERSIST_15 = 0x06,
TSL2591_PERSIST_20 = 0x07,
TSL2591_PERSIST_25 = 0x08,
TSL2591_PERSIST_30 = 0x09,
TSL2591_PERSIST_35 = 0x0A,
TSL2591_PERSIST_40 = 0x0B,
TSL2591_PERSIST_45 = 0x0C,
TSL2591_PERSIST_50 = 0x0D,
TSL2591_PERSIST_55 = 0x0E,
TSL2591_PERSIST_60 = 0x0F,
TSL2591_GAIN_LOW = 0x00,
TSL2591_GAIN_MED = 0x10,
TSL2591_GAIN_HIGH = 0x20,
TSL2591_GAIN_MAX = 0x30;
uint8_t DevID;
// ***** VARS  ************************** //
volatile char uartBuffer[BUFFER_SIZE];
volatile uint32_t bufferIndex = 0;
volatile bool dataReceived = false;
// ***** INIT  ********************* //
void InitializeUART1(void);
void InitializeUART0(void);
void init(void);
void InitializeSystemClock();
// ***** INTERRUPTS  ********************* //
void UART1IntHandler(void);
void Timer0AIntHandler(void);
// ***** ESP8266  ********************* //
void exit_wifi_passthrough_mode(void *ptr);
void setUpWifi(char *ssid, char *password);
void set_up_server(void *ptr);
void send_AT_Command(char *command);
void send_data_to_ip(char *data);
// ***** UART  ********************* //
void UARTStringSend(uint32_t ui32Base, const char *pui8Buffer);
// ***** I2C  ********************* //
void TSL2591_enable(void);
// ***** Timers  ********************* //
void Delay(uint32_t ui32Count);
void SetupTimer(void);

// *****  Initial ESP8266 Setup ***************** //
void SetupTimer(void) {
		uint32_t ui32Period;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    ui32Period = SysCtlClockGet() * 2;
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
    TimerEnable(TIMER0_BASE, TIMER_A);
}

void UARTStringSend(uint32_t ui32Base, const char *pui8Buffer) {
    while(*pui8Buffer) {
        UARTCharPut(ui32Base, *pui8Buffer++);
    }
}
void UART1IntHandler(void) {
    uint32_t ui32Status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, ui32Status);
    while(UARTCharsAvail(UART1_BASE) && bufferIndex < BUFFER_SIZE - 1) {
        uartBuffer[bufferIndex] = (char)UARTCharGetNonBlocking(UART1_BASE);
        if (uartBuffer[bufferIndex] == '\n') {
            dataReceived = true;
            uartBuffer[++bufferIndex] = '\0';
            return;
        }
        bufferIndex++;
    }
}

void InitializeSystemClock() {
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}

void InitializeUART0(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}

void InitializeUART1(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)) {}
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)) {}
    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTIntRegister(UART1_BASE, UART1IntHandler);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART1);
    IntPrioritySet(INT_UART1, 0x20);
}
void Delay(uint32_t ui32Count) {
    SysCtlDelay(ui32Count * SysCtlClockGet());
}
void set_up_server(void *ptr)
{
		char command[1000];  // Define a buffer for the command
    char nonVolatileBuffer[BUFFER_SIZE];
    uint32_t port = 12345;
    sprintf(command, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", TARGET_IP_ADDRESS, port);
		send_AT_Command(command);
}
void exit_wifi_passthrough_mode(void *ptr)
{
	// Exit Wifi Passthrough Mode
	SysCtlDelay(SysCtlClockGet() / 3);
	UARTStringSend(UART1_BASE, "+++");
	SysCtlDelay(SysCtlClockGet() / 3);
}
void send_data_to_ip(char *data) {
		send_AT_Command("AT+CIPMODE=1\r\n");
		send_AT_Command("AT+CIPSEND\r\n");
		UARTStringSend(UART1_BASE, data);
		exit_wifi_passthrough_mode(0);
		send_AT_Command("AT+CIPMODE=0\r\n");
		
}
void setUpWifi(char *ssid, char *password)
{
	  uint32_t numCommands;
		bool commandSent;
		uint32_t currentCommand;
		char nonVolatileBuffer[BUFFER_SIZE];
		const char *commands[] = {
    "AT\r\n",
    "AT+CWMODE=1\r\n",
    "AT+CWJAP=\"Network Name\",\"Password\"\r\n", // Replace with your SSID and Password
		"AT+CIFSR\r\n", // Get IP information
		};
		numCommands = sizeof(commands) / sizeof(commands[0]);
		currentCommand = 0;
    commandSent = false;
		while(1) {
        if (!dataReceived && !commandSent) {
            if (currentCommand < numCommands) {
                UARTStringSend(UART0_BASE, "Sending Command: ");
                UARTStringSend(UART0_BASE, commands[currentCommand]);
                UARTStringSend(UART0_BASE, "\r\n");
                UARTStringSend(UART1_BASE, commands[currentCommand]);
                commandSent = true;
								SysCtlDelay(SysCtlClockGet() * 2);
            }
        }
        if (dataReceived) {
						memcpy(nonVolatileBuffer, (char *) uartBuffer, bufferIndex); // Copy data to nonVolatileBuffer
						UARTStringSend(UART0_BASE, "Received Response: ");
						UARTStringSend(UART0_BASE, nonVolatileBuffer);
            UARTStringSend(UART0_BASE, "\r\n");
            memset((void *)uartBuffer, 0, BUFFER_SIZE);
            bufferIndex = 0;
            dataReceived = false;
            commandSent = false;
            currentCommand++;
            SysCtlDelay(SysCtlClockGet() * 2);
        }
    }
}
void send_AT_Command(char *command) {
    char nonVolatileBuffer[BUFFER_SIZE];
    bool commandSent = false;

    // Send the command to ESP8266
    UARTStringSend(UART0_BASE, "Sending Command: ");
    UARTStringSend(UART0_BASE, command);
    UARTStringSend(UART0_BASE, "\r\n");

    UARTStringSend(UART1_BASE, command); // Send the command to ESP8266
    commandSent = true;
		SysCtlDelay(SysCtlClockGet() /3);
    // Wait for the response
    while (!dataReceived) {
        // Implement a timeout mechanism if needed
        // Check for a timeout condition and break out of the loop
    }

    // Copy the response to a buffer
    memcpy(nonVolatileBuffer, (char *) uartBuffer, bufferIndex);

    // Print the received response
    UARTStringSend(UART0_BASE, "Received Response: ");
    UARTStringSend(UART0_BASE, nonVolatileBuffer);
    UARTStringSend(UART0_BASE, "\r\n");

    // Clear the buffer and flags
    memset((void *) uartBuffer, 0, BUFFER_SIZE);
    bufferIndex = 0;
    dataReceived = false;
}
// *****  Light Sensor Setup ***************** //
void UARTPrint(const char *str) {
    while(*str != '\0') {
        UARTCharPut(UART0_BASE, *str++);
    }
}

void I2CWrite(uint8_t reg, uint8_t value) {
		 UARTPrint("i2cWriteFunction\r\n");
    while (I2CMasterBusy(I2C1_BASE));
    I2CMasterSlaveAddrSet(I2C1_BASE, TSL2591_ADDRESS, false);
    I2CMasterDataPut(I2C1_BASE, COMMAND_BIT | reg);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while (I2CMasterBusy(I2C1_BASE));
    I2CMasterDataPut(I2C1_BASE, value);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
    while (I2CMasterBusy(I2C1_BASE));
}
void I2C1_read(uint8_t slave_addr, uint8_t *RxData, uint8_t N)
{
  uint8_t i; 
  I2CMasterSlaveAddrSet(I2C1_BASE, slave_addr, true);
  if (N==1)
  {
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C1_BASE));
    RxData[0]=I2CMasterDataGet(I2C1_BASE);
    while (I2CMasterBusy(I2C1_BASE));
  }
  else
  {
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while (I2CMasterBusy(I2C1_BASE));
    RxData[0]=I2CMasterDataGet(I2C1_BASE);
    while (I2CMasterBusy(I2C1_BASE));
    for (i=1;i<(N-1);i++)
    {
      I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
      while (I2CMasterBusy(I2C1_BASE));
      RxData[i]=I2CMasterDataGet(I2C1_BASE);
      while (I2CMasterBusy(I2C1_BASE));
    }

  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
  while (I2CMasterBusy(I2C1_BASE));
  RxData[N-1]=I2CMasterDataGet(I2C1_BASE);
  while (I2CMasterBusy(I2C1_BASE));
  }
}
uint8_t I2CReadSingleByte(uint8_t reg) {
		UARTPrint("ReadSingle Byte\r\n");
    while (I2CMasterBusy(I2C1_BASE));
    I2CMasterSlaveAddrSet(I2C1_BASE, TSL2591_ADDRESS, false);
    I2CMasterDataPut(I2C1_BASE, COMMAND_BIT | reg);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND);
    while (I2CMasterBusy(I2C1_BASE));
    I2CMasterSlaveAddrSet(I2C1_BASE, TSL2591_ADDRESS, true);
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
    while (I2CMasterBusy(I2C1_BASE));

    return I2CMasterDataGet(I2C1_BASE);
}
void InitI2C1(void) {
		//Enable I2C module 1
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
  SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
  GPIOPinConfigure(GPIO_PA6_I2C1SCL);
  GPIOPinConfigure(GPIO_PA7_I2C1SDA);
  GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
  GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);
  I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
}
void TSL2591_Init(void) {

  uint8_t temp_data[1];
	char message[50];
  I2CWrite(TSL2591_ADDR, TSL2591_REGISTER_DEVICE_ID);
  I2C1_read(TSL2591_ADDR, temp_data, 1);
  DevID = temp_data[0];
	snprintf(message, sizeof(message), "Device ID: %d\r\n",DevID);
	UARTPrint(message);
  if (DevID == 0x12)
  {
    UARTPrint("Device Found...\r\n");
  }
  else
  {
    UARTPrint("\n Device Not Found\r\n");
  }
  TSL2591_enable();
}
void TSL2591_enable(void)
{
  I2CMasterSlaveAddrSet(I2C1_BASE, TSL2591_ADDR, false);
  I2CMasterDataPut(I2C1_BASE, TSL2591_COMMAND_BIT|TSL2591_INTEGRATIONTIME_100MS);
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while (I2CMasterBusy(I2C1_BASE));
  I2CMasterDataPut(I2C1_BASE, TSL2591_GAIN_MED);
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while (I2CMasterBusy(I2C1_BASE));
  I2CMasterSlaveAddrSet(I2C1_BASE, TSL2591_ADDR, false);
  I2CMasterDataPut(I2C1_BASE, TSL2591_COMMAND_BIT|TSL2591_REGISTER_ENABLE);
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
  while (I2CMasterBusy(I2C1_BASE));
  I2CMasterDataPut(I2C1_BASE,TSL2591_ENABLE_POWERON|TSL2591_ENABLE_AEN| TSL2591_ENABLE_AIEN|TSL2591_ENABLE_NPIEN);
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
  while (I2CMasterBusy(I2C1_BASE));
}
uint16_t ReadLightSensorData(uint8_t lowReg, uint8_t highReg) {
    uint8_t lowByte = I2CReadSingleByte(lowReg);
    uint8_t highByte = I2CReadSingleByte(highReg);
    return (highByte << 8) | lowByte;
}

void ReadSensor(void) {
    uint16_t visibleLight = ReadLightSensorData(TSL2591_REGISTER_CHAN0_LOW, TSL2591_REGISTER_CHAN0_HIGH);
    uint16_t infraredLight = ReadLightSensorData(TSL2591_REGISTER_CHAN1_LOW, TSL2591_REGISTER_CHAN1_HIGH);
    char message[100];
    snprintf(message, sizeof(message), "Visible Light: %u, Infrared Light: %u\r\n", visibleLight, infraredLight);
    UARTPrint(message);
		send_data_to_ip(message);
}
// *****  Temperature Sensor ***************** //
void InitializeADC0(void) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_TS | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);
}

uint32_t ReadADC0(void) {
		uint32_t adcValue;
    ADCProcessorTrigger(ADC0_BASE, 3);
    while (!ADCIntStatus(ADC0_BASE, 3, false)) {
    }
    ADCSequenceDataGet(ADC0_BASE, 3, &adcValue);

    return adcValue;
}
float ConvertADCToTemperature(uint32_t adcValue) {

    return (147.5 - ((75.0 * 3.3 * adcValue) / 4096.0));
}
void ConvertTemperatureToString(float temperature, char *buffer, int bufferSize) {
    snprintf(buffer, bufferSize, "Temperature: %.2f C\r\n", temperature);
}
void ReadTemps(void)
{
		uint32_t adcValue = ReadADC0();
		float temperature = ConvertADCToTemperature(adcValue);
		char message[50];
		ConvertTemperatureToString(temperature, message, sizeof(message));
    send_data_to_ip(message);
    SysCtlDelay(SysCtlClockGet() / 3);
		InitializeADC0();
}
int main(void) {
		
		
    InitializeSystemClock();
		InitializeUART0();
    InitializeUART1();
    IntMasterEnable();
		send_AT_Command("AT+RST\r\n");
		SysCtlDelay(SysCtlClockGet() * 2);
		//setUpWifi(" ", " ");
		set_up_server(0);
		send_data_to_ip("Hello world!");
		send_data_to_ip("A good method of sending data");
		send_data_to_ip("Even Lighting and Temperature Data");
    InitI2C1();
		UARTPrint("Start Init for TSL2591\r\n");
    TSL2591_Init();
		InitializeADC0();
		ReadSensor();

    while(1) {
				ReadSensor();
				SysCtlDelay(SysCtlClockGet() / 3);
				ReadTemps();
				SysCtlDelay(SysCtlClockGet() / 3);
        }
}
