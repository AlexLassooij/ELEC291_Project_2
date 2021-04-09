/#ifndef P__Project2_H
#define P__Project2_H
/**************************************************************************************************************/
//Pin configurations
/**************************************************************************************************************/
// Indices
#define SMALL 0x0000e8
#define LARGE 0x0033f8
#define FERROUS 0x00789e
#define NON 0x00c038
#define OBJECT 0x00fe01
#define DETECTED 0x013efd
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

#define SMALL_LEN 0x00479a
#define LARGE_LEN 0x00479a
#define FERROUS_LEN 0x00479a
#define NON_LEN 0x003dc9
#define OBJECT_LEN 0x0040fc
#define DETECTED_LEN 0x004912
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

// Defines
#define SYSCLK 40000000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define MEASURING_COUNT 50
#define TOLERANCE 20
#define SMALL_OBJECT_TOLERANCE 400
#define DEF_FREQ 22050L

/**************************************************************************************************************/

/**************************************************************************************************************/
void Init_pwm(void);
void Set_pwm(unsigned char val);
void UART2Configure(int baud_rate);
void uart_putc(unsigned char c);
void uart_puts(char* buff);
unsigned char uart_getc(void);
void config_SPI(void);
unsigned char SPIWrite(unsigned char a);
void SetupTimer1(void);
// void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void);
void Start_Playback(unsigned long int address, unsigned long int numb);
void Enable_Write(void);
void Check_WIP(void);
void Init_LCD_Pins(void);
int _mon_getc(int canblock);
void wait_1ms(void);
long int GetPeriod(int n);
float Get_Ref_Freq(void);
float live_frequency(void);

#define LCD_RS LATBbits.LATB2
#define LCD_RW LATBbits.LATB15
#define LCD_E  LATBbits.LATB13
#define LCD_D4 LATBbits.LATB12
#define LCD_D5 LATBbits.LATB0		
#define LCD_D6 LATBbits.LATB3		
#define LCD_D7 LATBbits.LATB6		
#define CHARS_PER_LINE 16



/**************************************************************************************************************/
//Initialize LCD pins
/**************************************************************************************************************/
void Init_LCD_Pins(void);
/**************************************************************************************************************/
//Delay t microseconds using timer 4
/**************************************************************************************************************/
void Timer4us(unsigned char t);
/**************************************************************************************************************/
//Wait ms miliseconds
/**************************************************************************************************************/
void waitms(unsigned int ms);
/**************************************************************************************************************/
//
/**************************************************************************************************************/
void LCD_pulse(void);
/**************************************************************************************************************/
//
/**************************************************************************************************************/
void LCD_byte(unsigned char x);
/**************************************************************************************************************/
//
/**************************************************************************************************************/
void WriteData(unsigned char x);
/**************************************************************************************************************/
//
/**************************************************************************************************************/
void WriteCommand(unsigned char x);
/**************************************************************************************************************/
//Initialize LCD into 4-bit mode
/**************************************************************************************************************/
void LCD_4BIT(void);
/**************************************************************************************************************/
//Print string at line "line" of the LCD
/**************************************************************************************************************/
void LCDprint(char* string, unsigned char line, int clear);
/**************************************************************************************************************/
// void printStringLCD(const char* string, int line_num);
/**************************************************************************************************************/

/**************************************************************************************************************/
#endif
