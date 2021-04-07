#include <XC.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "LCD.h"
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Secondary Oscillator Enable (Disabled)

// Defines
#define SYSCLK 40000000L
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define WAIT_INTERVAL 5000
#define TOLERANCE 10
#define LCD_RS LATBbits.LATB2
#define LCD_RW LATBbits.LATB15
#define LCD_E  LATBbits.LATB13
#define LCD_D4 LATBbits.LATB12
#define LCD_D5 LATBbits.LATB0
#define LCD_D6 LATBbits.LATB3
#define LCD_D7 LATBbits.LATB6
#define CHARS_PER_LINE 16

#define WRITE_ENABLE     0x06  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define WRITE_DISABLE    0x04  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define READ_STATUS      0x05  // Address:0 Dummy:0 Num:1 to infinite fMax: 32MHz
#define READ_BYTES       0x03  // Address:3 Dummy:0 Num:1 to infinite fMax: 20MHz
#define READ_SILICON_ID  0xab  // Address:0 Dummy:3 Num:1 to infinite fMax: 32MHz
#define FAST_READ        0x0b  // Address:3 Dummy:1 Num:1 to infinite fMax: 40MHz
#define WRITE_STATUS     0x01  // Address:0 Dummy:0 Num:1 fMax: 25MHz
#define WRITE_BYTES      0x02  // Address:3 Dummy:0 Num:1 to 256 fMax: 25MHz
#define ERASE_ALL        0xc7  // Address:0 Dummy:0 Num:0 fMax: 25MHz
#define ERASE_BLOCK      0xd8  // Address:3 Dummy:0 Num:0 fMax: 25MHz
#define READ_DEVICE_ID   0x9f  // Address:0 Dummy:2 Num:1 to infinite fMax: 25MHz

volatile unsigned long int playcnt=0;
volatile unsigned char play_flag=0;

void Init_pwm (void)
{
    // OC1 can be assigned to PA0, PB3, PB4, PB15, and PB7(in use).
    // Check TABLE 11-2: OUTPUT PIN SELECTION in datasheet.
    // Set OC1 to pin PA0 (pin 2 of DIP 28) with peripheral pin select
    RPA0Rbits.RPA0R = 0x0005;
 
    // Configure standard PWM mode for output compare module 1
    OC1CON = 0x0006; 
 
    // A write to PRy configures the PWM frequency
    // PR = [FPB / (PWM Frequency * TMR Prescale Value)] � 1
    PR2 = (SYSCLK / (PWM_FREQ*1)) - 1;
 
    // A write to OCxRS configures the duty cycle
    // : OCxRS / PRy = duty cycle
    OC1RS = (PR2 + 1) * ((float)DUTY_CYCLE / 100);

 	T2CON = 0x0;
    T2CONSET = 0x8000;      // Enable Timer2, prescaler 1:1
	T2CONbits.TCKPS=0x0; // Set pre-scaler to 1
    OC1CONSET = 0x8000;     // Enable Output Compare Module 1
}

void Set_pwm (unsigned char val)
{
	OC1RS = (PR2 + 1) * ((float)val / 256.0);
}


void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void uart_putc (unsigned char c)
{
    while( U2STAbits.UTXBF); // wait while TX buffer full
    U2TXREG = c; // send single character to transmit buffer
}

void uart_puts (char * buff)
{
	while (*buff)
	{
		uart_putc(*buff);
		buff++;
	}
}

unsigned char uart_getc (void)
{
	unsigned char c;
	
	while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	c=U2RXREG;
	return c;
}

void config_SPI(void)
{
	int rData;

	// SDI1 can be assigned to any of these pins (table TABLE 11-1: INPUT PIN SELECTION):
	//0000 = RPA1; 0001 = RPB5; 0010 = RPB1; 0011 = RPB11; 0100 = RPB8
	SDI1Rbits.SDI1R=0b0010; //SET SDI1 to RB1, pin 5 of DIP28
    ANSELB &= ~(1<<1); // Set RB1 as a digital I/O
    TRISB |= (1<<1);   // configure pin RB1 as output
	
	// SDO1 can be configured to any of these pins by writting 0b0011 to the corresponding register.
	// Check TABLE 11-2: OUTPUT PIN SELECTION (assuming the pin exists in the dip28 package): 
	// RPA1, RPB5, RPB1, RPB11, RPB8, RPA8, RPC8, RPA9, RPA2, RPB6, RPA4
	// RPB13, RPB2, RPC6, RPC1, RPC3
	RPA1Rbits.RPA1R=0b0011; // config RA1 (pin 3) for SD01
	
	// SCK1 is assigned to pin 25 and can not be changed, but it MUST be configured as digital I/O
	// because it is configured as analog input by default.
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB |= (1<<14);   // configure RB14 as output
    
    // CSn is assigned to RB0, pin 4.  Also onfigure as digital output pin.
    ANSELB &= ~(1<<0); // Set RB0 as a digital I/O
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = 1;	

	SPI1CON = 0; // Stops and resets the SPI1.
	rData=SPI1BUF; // clears the receive buffer
	SPI1STATCLR=0x40; // clear the Overflow
	SPI1CON=0x10008120; // SPI ON, 8 bits transfer, SMP=1, Master,  SPI mode unknown (looks like 0,0)
	SPI1BRG=8; // About 2.4MHz clock frequency
}

unsigned char SPIWrite(unsigned char a)
{
	SPI1BUF = a; // write to buffer for TX
	while(SPI1STATbits.SPIRBF==0); // wait for transfer complete
	return SPI1BUF; // read the received value
}

void SetupTimer1 (void)
{
	// Explanation here:
	// https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/DEF_FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // Pre-scaler: 1
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	unsigned char c;
	
	LATBbits.LATB6 = !LATBbits.LATB6; // Toggle pin RB6 (used to check the right frequency)
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
	
	if(play_flag!=0)
	{  
		if(playcnt==0)
		{
			SET_CS; // Done playing: Disable 25Q32 SPI flash memory
			play_flag=0;
		}
		else
		{
			c=SPIWrite(0x00);
			//SPIWrite transmits passed value, receives value and returns it
			Set_pwm(c); // Output value to PWM (used as DAC)
			playcnt--;
		}
	}
}

void Start_Playback (unsigned long int address, unsigned long int numb)
{
    CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(READ_BYTES);
    SPIWrite((unsigned char)((address>>16)&0xff));
    SPIWrite((unsigned char)((address>>8)&0xff));
    SPIWrite((unsigned char)(address&0xff));
    playcnt=numb;
    play_flag=1;
}

void Enable_Write (void)
{
    CLR_CS; // Enable 25Q32 SPI flash memory.
    SPIWrite(WRITE_ENABLE);
	SET_CS; // Disable 25Q32 SPI flash memory
}

void Check_WIP (void)
{
	unsigned char c;
	do
	{
    	CLR_CS; // Enable 25Q32 SPI flash memory.
	    SPIWrite(READ_STATUS);
	    c=SPIWrite(0x55);
		SET_CS; // Disable 25Q32 SPI flash memory
	} while (c&0x01);
}

//Initiate LCD pins
void Init_LCD_Pins (void)
{
    TRISBbits.TRISB2=0;
    TRISBbits.TRISB15=0;
    TRISBbits.TRISB13=0;
    TRISBbits.TRISB12=0;
    TRISAbits.TRISA4=0;
    TRISBbits.TRISB4=0;
    TRISAbits.TRISA3=0;
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

// void waitms(int len)
// {
// 	while(len--) wait_1ms();
// }

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}

float Get_Ref_Freq(void){
	long int count;
	int  t_elapsed = 0;
	float T, new_freq, f, freq_diff, ref_freq;


	count = GetPeriod(100);
	if (count>0){
		T = (count * 2.0) / (SYSCLK * 100.0);
		ref_freq = 1.0 / T;
	}
	
    while(t_elapsed<WAIT_INTERVAL)
	{
		count = GetPeriod(100);
		if (count > 0)
		{
			T = (count * 2.0) / (SYSCLK * 100.0);
			new_freq = 1 / T;

			freq_diff = abs(ref_freq);
			if(freq_diff < TOLERANCE){
				print("Invalid reference frequency!: %f Hz \r", ref_freq);
			}
			else{
				print("Reference frequency valid!: %f Hz\r", ref_freq);
				waitms(1);
			}
		}
	}
	return ref_freq;
}

float live_frequency(void){
	long int count;
	int  t_elapsed = 0;
	float live_freq;

		count = GetPeriod(100);
		if (count>0){
			T = (count * 2.0) / (SYSCLK * 100.0);
			live_freq = 1.0 / T;
		}
	return live_freq;
}




// Information here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/1-basic-digital-io-220
void main(void)
{
    long int count;
    int  t_elapsed = 0;
    float T, live_freq, f, freq_diff, ref_freq;

    CFGCON = 0;

    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    
    Init_LCD_Pins();			//Initialize LCD pins
    LCD_4BIT();				//Set LCD as 4-bit mode

    ANSELB &= ~(1<<5); // Set RB5 as a digital I/O
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
 
	waitms(500);
	printf("Period measurement using the core timer free running counter.\r\n"
	      "Connect signal to RB5 (pin 14).\r\n");
	// bool isInit = false;
	ref_freq = Get_Ref_Freq();
	
	while (1){
		if(/*button_press*/){
			ref_freq = Get_Ref_Freq();
		}
		live_freq = live_frequency();

		freq_diff = ref_freq - live_freq;

		if(abs(freq_diff) < SMALL_OBJECT_TOLERANCE){
			if(freq_diff<0){
				//decrease in freq --> means small ferrous
				Start_Playback(SMALL, SMALL_LEN);
				Start_Playback(FERROUS, FERROUS_LEN);
				Start_Playback(OBJECT, OBJECT_LEN);
				Start_Playback(DETECTED, DETECTED_LEN);
				
			}
			else{
				//increase in freq  -->means small non ferrous
				Start_Playback(SMALL, SMALL_LEN);
				Start_Playback(NON, NON_LEN);
				Start_Playback(FERROUS, FERROUS_LEN);
				Start_Playback(OBJECT, OBJECT_LEN);
				Start_Playback(DETECTED, DETECTED_LEN);
			}
			// determine sign of difference
			//announce small object detected
	
		}
		else if (freq_diff<0){
			//announce large ferrous object 
			Start_Playback(LARGE, LARGE_LEN);
			Start_Playback(FERROUS, FERROUS_LEN);
			Start_Playback(OBJECT, OBJECT_LEN);
			Start_Playback(DETECTED, DETECTED_LEN);
		}
		else{
			//announce large non ferrous object
			Start_Playback(LARGE, LARGE_LEN);
			Start_Playback(NON, NON_LEN);
			Start_Playback(FERROUS, FERROUS_LEN);
			Start_Playback(OBJECT, OBJECT_LEN);
			Start_Playback(DETECTED, DETECTED_LEN);
		}

		waitms(2000);


	}

	
	//After getting reference frequency, measure live frequency forever
	

}
