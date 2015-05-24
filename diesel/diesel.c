/*
 * diesel.c
 *
 * Created: 26.03.2015 18:15:58
 *  Author: macboy
  PIN CONFIGURATION
  PB0 - PB7 - LED SEGMENTS
  PC2, PC3, PC4, PC5 - common cathodes
  
  
  

 */ 


// useful macros;
#define BIT_1(port, x) port |= 1<<x
#define BIT_0(port, x) port &= ~(1<<x)
#define IS_BIT(a,b) ((a) & (1<<(b)))
#define low(x)   ((x) & 0xFF)
#define high(x)   (((x)>>8) & 0xFF)


#define F_CPU 8000000UL


#include <avr/interrupt.h>

#include <util/delay.h>
#include <avr/pgmspace.h>
								//  -	
								// | |	
#include <avr/io.h>				//  -	
								// | |	
								//  -	
const unsigned char symbols [] = {0b11111100, 0b01100000, 0b11011010, 0b11110010, 0b01100110, 0b10110110, 0b10111110, 0b11100000, 0b11111110, 0b11110110, 0, 0b000000010};

unsigned char display[4] = {10, 11 ,0, 5};
	
uint16_t angle = 0;
uint16_t angle_a = 0;
uint16_t angle_filtered = 0;
char acc = 0;

uint16_t taho = 0;	
uint16_t diff = 0;
uint16_t period = 0;
char minus = 0;
char error = 0;
char cycle_start = 0; // flag for measuring cycle start;
char cycle_end = 0;
#define FILTER 2


// PID routine

typedef struct
{
	char IsReset;                                  // Команда на сброс ПИД-регулятора
	float InputValue;                             // Значение технологического датчика (y)
	float OutputValue;                           // Выходное значение ПИД-регулятора (u)
	float PreviousDeltaValue;                // Предыдущие значение ошибки
	float Integral;                                 // Результат интегрирования
	float TiNorm;                                  // Коэффициент интегрирования приведенный к реальному времени
	float TdNorm;                                 // Коэффициент дифференцирования приведенный к реальному времени
	float TargetValue;                           // Задание технологического параметра (y0)
	float K;                                          // Заданный пропорциональный коэффициент
	float Ti;                                         // Заданный коэффициент интегрирования
	float Td;                                        // Заданный коэффициент дифференцирования
	float MinOutputValue;                     // Задание минимального выходного значения
	float MaxOutputValue;                    // Задание максимального выходного значения
	float MaxAbsIntegral;                     // Задание максимального значения интегрирования (по модулю)
	float StartRate;                              // Задание максимального темпа нарастания
	float StopRate;                              // Задание максимального темпа убывания
} regulator_t;                                  // Структура переменной ПИД-регулятора

regulator_t* RegParam;                   // Объявление переменной содержащей параметры ПИД-регулятора
float Tc;                                          // Период основной частоты выполнение программы для учета дискретности

/* Подпрограмма сброса ПИД-регулятора в начальное состояние и вычисление коэффициентов ПИД-регулятора с учетом времени квантования */

void ResetRegulator(regulator_t * r)
{
	if (r->IsReset)                              // Если пришла команда сброса регулятора, то
	{
		r->Integral=0;                          // Обнуление результата интегрирования
		r->PreviousDeltaValue = 0;        // Обнуление предыдущего значения ошибки
		if (r->Ti==0)                             // Приведение значения коэффициента интегрирования к реальному времени
		r->TiNorm=0;
		else
		r->TiNorm = Tc/r->Ti;
		r->TdNorm = r->Td*Tc;             // Приведение значения коэффициента дифференцирования к реальному времени
		r->IsReset = 0;                         // Очищение команды сброса регулятора
	}
}

float Kp = 4;
float Kpp = 1;
float Ki = 0.0;
float Kd = 0;
uint16_t INTERVAL = 10000;

signed int pid_control(signed char setpoint, signed char currentpoint)
{
	static signed int last_error;
	signed int P;
	signed int D;
	static signed int I;
	signed int PID;
	P = (setpoint - currentpoint);
	I = (I + (setpoint - currentpoint) * INTERVAL);
	D = (((setpoint - currentpoint) - last_error) / INTERVAL);
	last_error = setpoint - currentpoint;
	if (P>=0) 
	PID = (Kp * P) + (Ki * I) + (Kd * D);
	else
	PID = (Kpp * P) + (Ki * I) + (Kd * D);
	
	return (PID);
}




ISR(INT0_vect)
{ // mean that INT0 - basic (injector), INT1 - slave signal
	
    BIT_1(PORTD, 7);  //illumination
    BIT_0(PORTD, 6);
	
	if (cycle_start == 0 && cycle_end == 1) // seems that it is second pulse, can calc period and angle 
	{
		period = TCNT1;
		TCNT1 = 0; // clear counter
		cycle_start = 1; 
		cycle_end = 0;
		error = 0;
	} else
	// first pulse
	{
		period = 0;
		TCNT1 = 0;
		cycle_start = 1;
		cycle_end = 0;
	}
   
}

ISR(INT1_vect)
{
	  BIT_1(PORTD, 6);
	if (cycle_start == 1 && cycle_end != 1) //injector  pulse was some time ago, and no pulses from crankshaft
	  { 
		diff = TCNT1;
		cycle_start = 0;//forget about this
		cycle_end = 1;  //first pulse frome crankshaft we got
      }
	  else
	  {
		  
	  }
	  
	    if(period!=0 && period>100 && period <100000) //rpm from 100 to 6000
	    taho=10000000/8/period*6*2;
	  	
	    BIT_0(PORTD, 6);
	  	BIT_0(PORTD, 7);
	
}

ISR(TIMER1_OVF_vect)
{
	//обнуляем значения
	if (error <20) error +=10;
	angle = 0 ;
		BIT_0(PORTD, 6);
		BIT_0(PORTD, 7);
		

}

ISR(TIMER2_OVF_vect)
{
   // PWM right here
   BIT_1(PORTC, PC1);
   
}
	
ISR(TIMER2_COMP_vect)
{
	//if (IS_BIT(PINC, PC1))
	 BIT_0(PORTC, PC1); //else BIT_1(PORTC, PC1);
	
}
	
char output = 0;
char counter  = 0;
char need = 8; //8 deg at freerun;
int16_t PWM= 0;
int main(void)
{
	 
	 PORTB = 0x00;
	 DDRB = 0xff;
	 DDRC = 0xff;
	 PORTC = 0x00;
	 DDRD = 0;
	 PORTD = 0xff;
	 //_BV(ISC10) 
	 	MCUCR |= _BV(ISC00) | _BV(ISC01) |  _BV(ISC10) |  _BV(ISC11); // all INT0 INT1 setup for rising edge
		//MCUCR |=  _BV(ISC01) |  _BV(ISC11) ; // all INT0 INT1 setup for falling edge
		  	
	 	GICR |= _BV(INT1) | _BV(INT0); // enable them;

	 	// Timer(s)/Counter(s) Interrupt(s) initialization
	 	TCCR1A=0x00;
		 
	 	TCCR2 = _BV(CS20) | _BV(CS21) | _BV(CS22); //timer 2 - 1/1024
	//	TCCR2 = _BV(WGM20);							//timer 2 - PWM - phase correct
		
		OCR2 = 10; //50% fill
		 
	 	TCCR1B=   _BV(CS11) | _BV(CS10);//clk/64
	 	TIMSK |= _BV(TOIE2) | _BV(OCIE2) | _BV(TOIE1); //timer 1 - ovf, timer 2- PWM correct
		sei();

DDRD = 0b011100000;
PORTD = 0b00000000;

    while(1)
    {
	// calculations of angle and period
			  if (period != 0)
			  {
				  error = 0; //flush error flag;
				  angle = (uint32_t)(period-diff)*3600/period;
				  angle=(angle+5)/10;
				  if (angle>180) {angle = 360-angle; minus=1;} else {minus=0;};
				  //angle -=20;
				  angle_a +=angle; acc++;
				  if (acc == FILTER) {angle_filtered = angle_a/FILTER; angle_a = 0; acc = 0;}
			  }
	
		
    output = 0;
	//if (counter == 0) {output++; if (output >1) output = 0;}
	
	if (error>2) display[3] = 11; else 
	switch (output)
	{
		case 0: display[3] = angle_filtered      % 10;
		display[2] = angle_filtered /10  % 10;
		display[1] = angle_filtered /100 % 10;
		display[0] = minus;
		break;
		case 1:
		display[3] = taho      % 10;
		display[2] = taho /10  % 10;
		display[1] = taho /100 % 10;
		display[0] = taho /1000 % 10;
		break;
	}	
	
	// regulator;
	need = taho/800+8;
	
	if (error<5) //got angle
	{
	if (minus)	// normal operation
	{
		PWM -= pid_control(need, angle_filtered);
		
	}
	// bounds
	if (PWM<0) PWM=0;
	if (PWM>2550) PWM = 2550;
	OCR2 = PWM/10;
	}
	//OCR2 = 10;

	  for (unsigned char i=0; i<4; i++) //dynamic display
		{
			PORTC |= 0b00111100;
			switch (i)
				{
				 case 0: BIT_0(PORTC, 2); if (display[0] == 1) PORTB = 0b01000000; else PORTB = 0b00000000; break; 
				 case 1: BIT_0(PORTC, 3); PORTB = symbols[display[i]]; break;
				 case 2: BIT_0(PORTC, 4); PORTB = symbols[display[i]]; break;
				 case 3: BIT_0(PORTC, 5); PORTB = symbols[display[i]]; break;
				 break;
				}
			_delay_ms(4);	
		}
    }
	
}