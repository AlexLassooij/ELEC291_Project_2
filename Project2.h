//============================================================================================================
//Display for LCD using PIC32
//Code source : Dr. Jesus Calvino Fraga (gathered from ELEC291 Winter Term 2 2020-2021 Piazza posts
//Date : April 7, 2021
//===========================================================================================================

#ifndef P__Project2_H
#define P__Project2_H
/**************************************************************************************************************/
//Pin configurations
/**************************************************************************************************************/
// Indices
#define SMALL 0x0033f8
#define LARGE 0x00789e
#define FERROUS 0x00c038
#define NON 0x00fe01
#define OBJECT 0x013efd
#define DETECTED 0x016ebc
#define DETERMINING 0x0187ce
#define REFERENCE 0x01d928
#define FREQUENCY 0x021c6f
#define IS 0x026cca
#define DEVICE 0x02ab30
#define READY 0x02f367
#define TO 0x0329cf
#define MEASURE 0x03580f
#define MEASUREMENT 0x038fe6
#define FAILED 0x03d39b
#define TRYING 0x04159b
#define AGAIN 0x0454e6

// Lengths

#define SMALL_LEN 0x0044a6
#define LARGE_LEN 0x00479a
#define FERROUS_LEN 0x003dc9
#define NON_LEN 0x0040fc
#define OBJECT_LEN 0x002fbf
#define DETECTED_LEN 0x001912
#define DETERMINING_LEN 0x00515a
#define REFERENCE_LEN 0x004347
#define FREQUENCY_LEN 0x00505b
#define IS_LEN 0x003e66
#define DEVICE_LEN 0x004837
#define READY_LEN 0x003668
#define TO_LEN 0x002e40
#define MEASURE_LEN 0x0037d7
#define MEASUREMENT_LEN 0x0043b5
#define FAILED_LEN 0x004200
#define TRYING_LEN 0x003f4b
#define AGAIN_LEN 0x003f82

/**************************************************************************************************************/

/**************************************************************************************************************/
void Init_pwm (void);
void Set_pwm (unsigned char val);
void UART2Configure(int baud_rate);
void uart_putc (unsigned char c);
void uart_puts (char * buff);
unsigned char uart_getc (void);
void config_SPI(void);
unsigned char SPIWrite(unsigned char a);
void SetupTimer1 (void);
// void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void);
void Start_Playback (unsigned long int address, unsigned long int numb);
void Enable_Write (void);
void Check_WIP (void);
void Init_LCD_Pins (void);
int _mon_getc(int canblock);
void wait_1ms(void);
long int GetPeriod (int n);
float Get_Ref_Freq(void);
float live_frequency(void);













/**************************************************************************************************************/
#endif
