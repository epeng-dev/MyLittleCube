/*
 * SmartCube.c
 *
 * Created: 2017-12-04 오후 2:40:25
 * Author : dsm2016
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#define DHT11_PIN 1
uint8_t c=0,I_RH,D_RH,I_Temp,D_Temp,CheckSum;

#define lcd_data PORTA
#define lcd_rs_on()   PORTD|=0x10
#define lcd_rw_on()   PORTD|=0x20
#define lcd_e_on()    PORTD|=0x40
#define lcd_rs_off()   PORTD&=~(0x10)
#define lcd_rw_off()  PORTD&=~(0x20)
#define lcd_e_off()    PORTD&=~(0x40)

#define RLOAD 10.0
#define RZERO 35.63
#define PARA 116.6020682
#define PARB 2.769034857

void command(unsigned char CMD);
void lcd_char(unsigned char CMD);
void lcd_str(char* string);
void lcd_init();
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_clear();
void lcd_Data(unsigned char data);
void Request();						/* Microcontroller send start pulse/request */
void Response();					/* receive response from DHT11 */
uint8_t Receive_data();				/* receive data */
void USART0_init(unsigned int UBRR0);
char rx_char(void);
void rx_str(char* buf);
void tx_char(char tx_data);
void tx_str(char* tx_data);
void RGB_LED();
void DHT11_sensor();
void I_sensor();

int analogRead();

int PPM = 0;
int TEMPERATURE = 0;
int HUMIDITY = 0;
int main(void)
{
	char buf[20];
	DDRD = 0xff;
	DDRA = 0xff;
	DDRF = 0x00;
	
	DDRB = 0xff;
	TCCR0 = 0x63;         // 0b01100011  Phase Correct PWM Mode 비반전 32분주

	TCCR1A = 0xA1;        // 0b10100001
	TCCR1B = 0x03;        // 64분주
	TCCR1C = 0x00;
	
	USART0_init(103);
	ADCSRA = 0x87;
	ADMUX = 0x00;
	
	lcd_init();			/* Initialize LCD */
	lcd_clear(0x01);			/* Clear LCD */
	
	while(1)
	{	
		RGB_LED();
		_delay_ms(3);
		
		DHT11_sensor();
		_delay_ms(5000);
		
		I_sensor();

		sprintf(buf, "%d %d %d\0", TEMPERATURE, HUMIDITY, PPM);
		tx_str(buf);
	}
}

void command(unsigned char CMD){
	lcd_data = CMD;
	lcd_e_on();
	_delay_ms(3);
	lcd_e_off();
}

void lcd_char(unsigned char CMD){
	lcd_rs_on();
	lcd_rw_off();
	command(CMD);
	lcd_rs_off();
}

void lcd_str(char* string){
	int i = 0;
	while(string[i]!='\0')
	lcd_char(string[i++]);
}

void lcd_init(){
	lcd_rs_off();
	lcd_rw_off();

	command(0x38);
	command(0x38);
	command(0x0C);
	command(0x06);
	command(0x01);
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstcharadd[]={0x80, 0xC0};
	command(firstcharadd[y] + x);
}

void lcd_clear()
{
	command(0x01);
	_delay_ms(2);
}

void lcd_Data(unsigned char data)
{
	lcd_rs_on();
	lcd_rw_off();
	lcd_e_on();
	//LCD_CTRL |=  (1 << LCD_RS);    //RS=1, R/W=0 으로 데이터 쓰기 싸이클
	//LCD_CTRL &= ~(1 << LCD_RW);
	//LCD_CTRL |=  (1 << LCD_EN);    //LCD 사용
	_delay_us(50);
	
	lcd_data = data;                //데이터 출력
	_delay_us(50);
	lcd_e_off();
	//LCD_CTRL &= ~(1 << LCD_EN);    //LCD 사용안함
}

void Request()				/* Microcontroller send start pulse/request */
{
	DDRF |= (1<<DHT11_PIN);
	PORTF &= ~(1<<DHT11_PIN);	/* set to low pin */
	_delay_ms(20);			/* wait for 20ms */
	PORTF |= (1<<DHT11_PIN);	/* set to high pin */
}

void Response()				/* receive response from DHT11 */
{
	DDRF &= ~(1<<DHT11_PIN);
	while(PINF & (1<<DHT11_PIN));
	while((PINF & (1<<DHT11_PIN))==0);
	while(PINF & (1<<DHT11_PIN));
}

uint8_t Receive_data()			/* receive data */
{
	for (int q=0; q<8; q++)
	{
		while((PINF & (1<<DHT11_PIN)) == 0);  /* check received bit 0 or 1 */
		_delay_us(30);
		if(PINF & (1<<DHT11_PIN))/* if high pulse is greater than 30ms */
		c = (c<<1)|(0x01);	/* then its logic HIGH */
		else			/* otherwise its logic LOW */
		c = (c<<1);
		while(PINF & (1<<DHT11_PIN));
	}
	return c;
}

int analogRead()
{
	int value = 0;
	ADCSRA = ADCSRA | 0x40;
	while((ADCSRA & 0x10) == 0);
	value = (int)ADCL + ((int)ADCH << 8);
	return value;
}

void USART0_init(unsigned int UBRR0){
	UBRR0H=(unsigned char)(UBRR0 >> 8);
	UBRR0L=(unsigned char)UBRR0;
	UCSR0B=(1<<(TXEN0));
}

char rx_char(void) {
	while((UCSR0A&0x80) == 0);
	return UDR0;
}

void rx_str(char* buf) {
	int i = 0;
	char ch = 0;
	memset(buf, 0, sizeof(buf));

	while((ch = rx_char()) != '\0'){
		buf[i] = ch;
		i+=1;
	}
	buf[i] = '\0';
}

void tx_char(char tx_data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0 = tx_data;
}

void tx_str(char* tx_data){
	int i = 0;
	while (tx_data[i] != '\0')
	{
		tx_char(tx_data[i]);
		i+=1;
	}
}

void RGB_LED()
{
	unsigned char i = 0;
	unsigned char j = 0;
	
	i = 0; j = 254;
	for(; i<254; )
	{
		OCR0 = j;
		OCR1A = i;
		_delay_ms(5);
		i++; j--;
	}
	OCR1A = 254;
	OCR0 = 0;
	_delay_ms(250);
	
	i = 254; j = 0;
	for(; i>0; )
	{
		OCR1A = i;
		OCR1B = j;
		_delay_ms(5);
		i--; j++;
	}
	OCR1A = 0;
	OCR1B = 254;
	_delay_ms(250);
	
	i = 0; j = 254;
	for(; i<254; )
	{
		OCR1B = j;
		OCR0 = i;
		_delay_ms(5);
		i++; j--;
	}
	OCR0 = 254;
	OCR1B = 0;
	_delay_ms(250);
}

void DHT11_sensor()
{	
	lcd_init();			/* Initialize LCD */
	lcd_clear(0x01);			/* Clear LCD */
	lcd_gotoxy(0,0);		/* Enter column and row position */
	lcd_str("Humidity =");
	lcd_gotoxy(0,1);
	lcd_str("Temp = ");
	
	char data[5];
	
	Request();		/* send start pulse */
	Response();		/* receive response */
	I_RH=Receive_data();	/* store first eight bit in I_RH */
	D_RH=Receive_data();	/* store next eight bit in D_RH */
	I_Temp=Receive_data();	/* store next eight bit in I_Temp */
	D_Temp=Receive_data();	/* store next eight bit in D_Temp */
	CheckSum=Receive_data();/* store next eight bit in CheckSum */
	
	if ((I_RH + D_RH + I_Temp + D_Temp) != CheckSum)
	{
		lcd_gotoxy(0,0);
		lcd_str("Error");
	}
	
	else
	{
		HUMIDITY = I_RH;
		itoa(I_RH,data,10);
		lcd_gotoxy(11,0);
		lcd_str(data);
		lcd_str(".");
	
		itoa(D_RH,data,10);
		lcd_str(data);
		lcd_str("%");

		TEMPERATURE = I_Temp;
		itoa(I_Temp,data,10);
		lcd_gotoxy(6,1);
		lcd_str(data);
		lcd_str(".");
		
		itoa(D_Temp,data,10);
		lcd_str(data);
		lcd_Data(0xDF);
		lcd_str("C ");
		
		itoa(CheckSum,data,10);
		lcd_str(data);
		lcd_str(" ");
	}	
}

void I_sensor()
{	
	lcd_init();			/* Initialize LCD */
	lcd_clear(0x01);			/* Clear LCD */
	
	char strvalue[20];
	int value = 0;
	double res = 0;
	double ppm = 0;
	command(0x01);

	value = analogRead();
	res = (1023.0/(double)value) * 5.0 - 1.0*RLOAD;

	ppm = PARA * pow((res/RZERO), -PARB);
	
	sprintf(strvalue, "%d", (int)ppm);
	command(0x01);
	lcd_gotoxy(0,0);		/* Enter column and row position */
	lcd_str(" density of CO2 ");
	lcd_gotoxy(0,1);
	lcd_str("      ");
	lcd_str(strvalue);
	//lcd_str(strvalue);
	_delay_ms(1000);
	
	PPM = (int)ppm;
}


