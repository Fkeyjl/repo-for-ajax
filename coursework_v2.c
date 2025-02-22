/*******************************************************
This program was created by the CodeWizardAVR V3.50 
Automatic Program Generator
© Copyright 1998-2023 Pavel Haiduc, HP InfoTech S.R.L.
http://www.hpinfotech.ro

Project : 
Version : 
Date    : 30.04.2023
Author  : 
Company : 
Comments: 


Chip type               : ATmega8
Program type            : Application
AVR Core Clock frequency: 1,000000 MHz
Memory model            : Small
External RAM size       : 0
Data Stack size         : 256
*******************************************************/

// I/O Registers definitions
#include <mega8.h>

#include <delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Voltage Reference: AREF pin
#define ADC_VREF_TYPE ((0<<REFS1) | (0<<REFS0) | (0<<ADLAR))
#define LCD PORTD
#define E 0
#define RW 1
#define RS 2
#define MAX_ADC_READING 1023 //розрядність аналого-цифрового перетворювача (2 в 10 степені - 1)
#define ADC_REF_VOLTAGE 5.0  //опорна напруга
#define REF_RESISTANCE 10000 //опір на статичному резисторі

// Read the AD conversion result
unsigned int read_adc(unsigned char adc_input)
{
ADMUX=adc_input | ADC_VREF_TYPE;
// Delay needed for the stabilization of the ADC input voltage
delay_us(10);
// Start the AD conversion
ADCSRA|=(1<<ADSC);
// Wait for the AD conversion to complete
while ((ADCSRA & (1<<ADIF))==0);
ADCSRA|=(1<<ADIF);
return ADCW;
}

// Declare your global variables here

void lcdcmd(unsigned char command);
void lcddata(unsigned char data);
void lcd_initialize();
void lcd_print(char* string);

void main(void)
{
// Declare your local variables here
char LDRSHOW[7];
float clbr_coeff[] = {2215.45, 2382.83, 2435.82, 2368.94};

lcd_initialize();
lcd_print("Lux meter");
lcdcmd(0xC0);
lcd_print("LDR lux:");
lcdcmd(0xC8);


// Input/Output Ports initialization
// Port B initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRB=(1<<DDB7) | (1<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (1<<DDB0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

// Port C initialization
// Function: Bit6=In Bit5=In Bit4=In Bit3=In Bit2=In Bit1=In Bit0=In 
DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
// State: Bit6=T Bit5=T Bit4=T Bit3=T Bit2=T Bit1=T Bit0=T 
PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

// Port D initialization
// Function: Bit7=Out Bit6=Out Bit5=Out Bit4=Out Bit3=Out Bit2=Out Bit1=Out Bit0=Out 
DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (1<<DDD4) | (1<<DDD3) | (1<<DDD2) | (1<<DDD1) | (1<<DDD0);
// State: Bit7=0 Bit6=0 Bit5=0 Bit4=0 Bit3=0 Bit2=0 Bit1=0 Bit0=0 
PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: Timer 0 Stopped
TCCR0=(0<<CS02) | (0<<CS01) | (0<<CS00);
TCNT0=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: Timer1 Stopped
// Mode: Normal top=0xFFFF
// OC1A output: Disconnected
// OC1B output: Disconnected
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(0<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (0<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (0<<WGM12) | (0<<CS12) | (0<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;

// Timer/Counter 2 initialization
// Clock source: System Clock
// Clock value: Timer2 Stopped
// Mode: Normal top=0xFF
// OC2 output: Disconnected
ASSR=0<<AS2;
TCCR2=(0<<PWM2) | (0<<COM21) | (0<<COM20) | (0<<CTC2) | (0<<CS22) | (0<<CS21) | (0<<CS20);
TCNT2=0x00;
OCR2=0x00;

// Timer(s)/Counter(s) Interrupt(s) initialization
TIMSK=(0<<OCIE2) | (0<<TOIE2) | (0<<TICIE1) | (0<<OCIE1A) | (0<<OCIE1B) | (0<<TOIE1) | (0<<TOIE0);

// External Interrupt(s) initialization
// INT0: Off
// INT1: Off
MCUCR=(0<<ISC11) | (0<<ISC10) | (0<<ISC01) | (0<<ISC00);

// USART initialization
// USART disabled
UCSRB=(0<<RXCIE) | (0<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (0<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);

// Analog Comparator initialization
// Analog Comparator: Off
// The Analog Comparator's positive input is
// connected to the AIN0 pin
// The Analog Comparator's negative input is
// connected to the AIN1 pin
ACSR=(1<<ACD) | (0<<ACBG) | (0<<ACO) | (0<<ACI) | (0<<ACIE) | (0<<ACIC) | (0<<ACIS1) | (0<<ACIS0);

// ADC initialization
// ADC Clock frequency: 125,000 kHz
// ADC Voltage Reference: Int., cap. on AREF
ADMUX=ADC_VREF_TYPE | (0 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
ADCSRA=(1<<ADEN) | (0<<ADSC) | (1<<ADFR) | (0<<ADIF) | (0<<ADIE) | (0<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
SFIOR=(0<<ACME);

// SPI initialization
// SPI disabled
SPCR=(0<<SPIE) | (0<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (0<<SPR1) | (0<<SPR0);

// TWI initialization
// TWI disabled
TWCR=(0<<TWEA) | (0<<TWSTA) | (0<<TWSTO) | (0<<TWEN) | (0<<TWIE);



while (1)
      {
        int ldrRawData = read_adc(0);
        
        float ldrResistance;
        float ldrLux;
        
        float outVoltage = (ADC_REF_VOLTAGE * ldrRawData) / MAX_ADC_READING;
        ldrResistance = REF_RESISTANCE * ((ADC_REF_VOLTAGE / outVoltage) - 1);

        //обчислюємо освітленість, використовуючи постійні
        //постійні розраховані грунтуючись на взаємозв'язку між освітленістю та опором
        if (outVoltage <= 4.41) {
            ldrLux = 255.84 * pow(ldrResistance, -1.111) * clbr_coeff[0]; 
        } else if (outVoltage > 4.41 && outVoltage <= 4.76) {
            ldrLux = 255.84 * pow(ldrResistance, -1.111) * clbr_coeff[1];
        } else if (outVoltage > 4.76 && outVoltage <= 4.85) {
            ldrLux = 255.84 * pow(ldrResistance, -1.111) * clbr_coeff[2];
        } else {
            ldrLux = 255.84 * pow(ldrResistance, -1.111) * clbr_coeff[3];  
        }
        
        //виводимо результат на lcd
        ftoa(ldrLux, 2, LDRSHOW);
        lcd_print(LDRSHOW);
        delay_ms(2000);
        lcdcmd(0xC8);        
      }
}

void lcdcmd(unsigned char command) {
    PORTB &= ~(1 << RS);
    PORTB &= ~(1 << RW);
    LCD = command;
    PORTB |= (1 << E);
    delay_ms(50);
    PORTB &= ~(1 << E);
}

void lcddata(unsigned char data) {
    PORTB |= (1 << RS);
    PORTB &= ~(1 << RW);
    LCD = data;             //LCD: PORTD
    PORTB |= (1 << E);
    delay_ms(50);
    PORTB &= ~(1 << E);
}

void lcd_initialize() {
    DDRD = 0xFF;
    DDRB = 0xFF;
    PORTD &= ~(1 << E);
    
    lcdcmd(0x01);
    lcdcmd(0x38);
    lcdcmd(0xF);
    delay_ms(10);
}

void lcd_print(char* string) {
    while(*string > 0) {
        lcddata(*string++);
    }    
}
