/*
 * Dancer_sensor2.cpp
 *
 * Created: 2018-06-18 오후 2:41:49
 *  Author: soondo
 */ 


/*
;문서번호 : EL-TENSION-PGM-03
;연결문서번호 : EL-TENSION-CIR-01~(구성회로)/EL-TENSION-BOX-ARRA-01(제어기 외함과 단자대 정의도면)
;참조문서번호 : 관련문건 전체
;참조문서번호 : '오차수정의 최적점을 찾는 구조의 재 설계'(2018. 4. 20/금요일 완성)
;프로그램 시작일 : 2018. 4. 20(금)~
;
;
;
;+++++++++++++++++++++++************        파일명 : Dancer_sensor(ATmega16A ;16MHz)        ***********+++++++++++++++++++++
;
;
;                   ==============================     프로그램의 구조    ================================
;
;1. 이 프로그램파일은 제품 'Dancer_con'의 시험장치에서 사용하고자 하는 위치센서 제어기의 gcc 프로그램이다.
;
;2. 위치센서는 상호인덕턴스의 유도형으로 제작되었다. 그러므로 1차측에는 약 5.5V의 직류전압으로 부터 120Hz의 교류를 생성, 공급한다. 이는 Simulation에 의한 결정값이다.
;
;3. Inverting의 프로그램구조는 120Hz의 반주기인 4.166ms 의 생성을 Timer1, 2를 사용하여 만들고 4166us를 마무리하는 Timer2의 Interrupt Precedure를 갔다올 때 현재주기를 바꾸는 구조로 한다.
;
*/



#include <avr/io.h>
#include <util/delay_basic.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <stdio.h>



#define		_UPPER_TURN_ON				PORTD |=(1<<PD2);
#define		_LOWER_TURN_ON				PORTD |=(1<<PD3);
#define		_UPPER_CTRL_ON				PORTD |=(1<<PD0);
#define		_LOWER_CTRL_ON				PORTD |=(1<<PD1);

#define		_UPPER_TURN_OFF				PORTD &=~(1<<PD2);
#define		_LOWER_TURN_OFF				PORTD &=~(1<<PD3);
#define		_UPPER_CTRL_OFF				PORTD &=~(1<<PD0);
#define		_LOWER_CTRL_OFF				PORTD &=~(1<<PD1);

#define		_TIMER0_STOP				TCCR0 &=~((1<<CS02) | (1<<CS01) | (1<<CS00));
#define		_TIMER0_64_START			TCCR0 |=(TCCR0 &=~(1<<CS02)) | (1<<CS01) | (1<<CS00);
#define		_TIMER1_64_START			TCCR1B |=(TCCR1B &=~(1<<CS12)) | (1<<CS11) | (1<<CS10);

#define		_LED1_ON					PORTD |=(1<<PD6);
#define		_LED2_ON					PORTB |=(1<<PB0);

#define		_SLEEP_ENABLE				MCUCR |=(1<<SE);			// Build-in의 초기화에서는 'Idle Mode'로 된다.
#define		_SLEEP_DISABLE				MCUCR &=~(1<<SE);

#define		_EEPROM_MST_WRITE			EECR |=(1<<EEMWE);
#define		_EEPROM_WRITE				EECR |=(1<<EEWE);
/*
#define		_ADC0_P_ADC1_N_DIFFERENTIAL	ADMUX |=(1<<MUX4) | (ADMUX &=~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0))); // Step_down 출력전압 변환용.
#define		_REF_ADC6					ADMUX |=(1<<MUX2) | (1<<MUX1) | (ADMUX &=~((1<<MUX4) | (1<<MUX3) | (1<<MUX0))); // Main인버터 출력주파수값 변환용.
#define		_DAN_ADC7					ADMUX |=(ADMUX &=~((1<<MUX4) | (1<<MUX3))) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0); // Dancer센서 입력값 변환용.
#define		_ADC4_P_ADC2_N_DIFFERENTIAL	ADMUX |=(1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (ADMUX &=~((1<<MUX1) | (1<<MUX0))); // Step_down 검출 출력전류 변환용

#define		_ADC_ENABLE					ADCSRA |=(1<<ADEN);
#define		_ADC_DISABLE				ADCSRA &=~(1<<ADEN);		
#define		_ADC_AUTO_TRIGGER_ENABLE	ADCSRA |=(1<<ADATE);
#define		_ADC_AUTO_TRIGGER_DISABLE	ADCSRA &=~(1<<ADATE);		// ADC를 Auto Trigger로 선택하면 초기값은 'Free Running'이다. ADC는 변환이 완료되면 트리거 할 수있는 자체 인터럽트를 가지고있다.
#define		_ADC_INTERRUPT_ENABLE		ADCSRA |=(1<<ADIE);
#define		_ADC_INTERRUPT_DISABLE		ADCSRA &=~(1<<ADIE);
//#define		_ADC_STOP					ADCSRA &=~(1<<ADSC);
#define		_ADC_START					ADCSRA |=(1<<ADSC);			// Auto Tigger에서 'Free Runnig'으로 선택하고 ADC Interrupt를 허용하면 현재의 변환이 완료되어을 때 'ADSC'은 '1'로 유지되면서 즉시 다시 변환이 시작...
																	//  ...되어서 Free Running' Mode가 된다. 그리고 이 Mode에서는 변환완료 때의 'ADIF'가 해제되었는지에 관계없이 동작한다.
																	// ADC변환에서의 첫 변환값은 반듯이 버린다.
*/
																	
#define		_DIVID_VAL					200;//120Hz의 1주기를 200등분.	


bool	b_dummy_convesion = false;



void clear_eeprom()
{
	unsigned int eeprom_addr_buffer = 0;
	
	
	for(unsigned int i=0 ; i<=511 ; i++){
		while(bit_is_set(EECR, EEWE));							// eeprom이 'Busy'가 아닐 때까지 대기하다가...
		
		EEAR = eeprom_addr_buffer;
		EEDR = 0xFF;
		
		_EEPROM_MST_WRITE;
		_EEPROM_WRITE;										//    ...Free Running으로 기록된 3개 ADC값을 eeprom에 적재한 후...
		eeprom_addr_buffer++;
	}
	
	while(bit_is_set(EECR, EEWE));							//         ...기록의 진행이 종료되었는지를 확인한다.
}


void write_eeprom(unsigned int	eeprom_data)
{
	unsigned char eeprom_addr_buffer = 0;
	unsigned int	byte_mask = 0;
	
	if(eeprom_data <= 255){
		for(unsigned char i=0 ; i<6 ; i++){
			while(bit_is_set(EECR, EEWE));							// eeprom이 'Busy'가 아닐 때까지 대기하다가...
			
			EEAR = eeprom_addr_buffer;
			EEDR = eeprom_data;
			
			_EEPROM_MST_WRITE;
			_EEPROM_WRITE;										//    ...Free Running으로 기록된 3개 ADC값을 eeprom에 적재한 후...
			eeprom_addr_buffer++;
		}
	}
	else{
		while(bit_is_set(EECR, EEWE));							// eeprom이 'Busy'가 아닐 때까지 대기하다가...
		
		EEAR = eeprom_addr_buffer;
		
		byte_mask = eeprom_data;
		byte_mask = byte_mask & 0xFF00;
		byte_mask = (byte_mask >> 8);
		EEDR = byte_mask;
		
		_EEPROM_MST_WRITE;
		_EEPROM_WRITE;										//    ...Free Running으로 기록된 3개 ADC값을 eeprom에 적재한 후...
		while(bit_is_set(EECR, EEWE));
		
		eeprom_addr_buffer++;
		EEAR = eeprom_addr_buffer;
		EEDR = eeprom_data;
		_EEPROM_MST_WRITE;
		_EEPROM_WRITE;
	}
	
	while(bit_is_set(EECR, EEWE));							//         ...기록의 진행이 종료되었는지를 확인한다.
}



			
	
void porter_initial()
{
	DDRA=0b00000000;
	PORTA=0b00000000;

	DDRB=0b01101111;									// PB0은 'LED2' / PB4는 'RUN'
	PORTB=0b00000000;

	DDRC=0b11111111;									// PC는 모두 FND Segment.
	PORTC=0b00000000;
	
	DDRD=0b01111111;									// PD5(OC1A) : PWM-A출력.
	PORTD=0b00000000;	
}	




void timer0_initial()										// 위치센서 1차측의 Inverting용 주파수는 Simulation에 의하여 Sinewave 120Hz로 결정되었다. 그러므로 반주기의 시간은 4.16666666ms 이다.
{
	if(bit_is_set(TIFR, TOV0))								// 만일 Timer0 의 Overflow 가 set되어 있다면 이를 지운다.
		TIFR |= (TOV0 | 1);
					
//	TIMSK |=(1<<TOIE0);										//  ...Timer0 을 Interrupt 로 사용한다.
}


void time2_initial()
{
	TCCR2 |=(TCCR2 &=~(1<<CS22) | (1<<CS21)) | (1<<CS20);	// 분주가 없다. 최대 15937.5ns 의 지연을 할 수가 있다. Inverting 제어에...
									                        //  ...서의 시간이 이보다 더 길어야 한다면 시스템Clock을 읽는 지연을 사용한다.
}


void timer1_pwm_initial()
{
/*
	TCCR1A |=(1<<COM1A1) | (1<<COM1A0);
	TCCR1A &=~(1<<WGM10);
	TCCR1A &=~(1<<WGM11);
	TCCR1B &=~(1<<WGM12);
	TCCR1B |=(1<<WGM13);	
	TCCR1B &=~(1<<CS12);									//시스템주파수를 분주없이 Timer에 공급.
	TCCR1B &=~(1<<CS11);									// 16MHz(외부osc)에서 시스템주파수를 분주없이 Timer1에 공급. 
	TCCR1B |=(1<<CS10);
*/

	TCCR1A |=(1<<COM1A1) | (1<<COM1A0) | (TCCR1A &=~((1<<WGM11) | (1<<WGM10)));
	TCCR1B |=(TCCR1B &=~((1<<CS12) | (1<<CS11) | (1<<WGM12))) | (1<<WGM13) | (1<<CS10);
			
	ICR1 = 1600;											// Step-down 출력전압용 PWM주파수: 5KHz/2oous									
	OCR1A = 1600;											// 0~10VDC 의 Step-down 출력전압을 '0'V 로 출력하게 하는 Timer1(16bit) PWM Match값인 '1600'을 초기화.	
}

/*
void	adc_trig_handdling()
{
	_SLEEP_ENABLE;											//  MCU의 Sleep 진입을 허용하고...
	_ADC_ENABLE;											//   ...ADC장치를 허용한 후...
	_ADC_START;												//    ...ADC변환을 지시한 다음...
	asm("SLEEP" ::);										//	   ...Sleepd으로 진입시켜서 변환을 실행하고... => ADC변환의 시작은 MCU가 Sleep되면 시작된다. 또 Interrupt에서의 복귀는 이 'SLEEP' 명령행 다음 부터 시작한다.
	_ADC_DISABLE;											//      ...실행 후 Interrut로 부터 복귀하면 ADC를 차단한 후...
	_SLEEP_DISABLE;											//		 ...sleep도 차단한다.
}


void adc_initial()
{
	ADCSRA |=(1<<ADPS2) | (1<<ADPS1) | (1<<ADPS2);			// 시스템주파수 16MHz를 128분주하여 ADC에 125KHz의 주파수를 공급.
	ADMUX |=(1<<REFS1) | (1<<REFS0);						// ADC의 Vref를 내부 2.56V로 설정.
	
	_ADC_INTERRUPT_ENABLE;									// ADC의 인터럽트 허용.	
	
	adc_trig_handdling();									//   ...최초의 변환인 ADC dummy변환을 실행한 후...
	while(!(b_dummy_convesion))								//     ...dummy변환이 완료되었는지를 보고 안 되었다면 여기에 붙잡아 둔다.
	;	
}

		
void	adc7()	
{
	ADMUX |=(ADMUX &=~((1<<MUX4) | (1<<MUX3))) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);		// Build-in시의 ADC채널은 Single-end로 'ADC0'으로 초기화 되었다가 이 초기화에서 'ADC7'로 설정.
}
*/


void delay_ms(unsigned int ms_num)
{
	unsigned int one_ms;
		
	while(ms_num)
	{
		for(one_ms=0 ; one_ms<=1000 ; one_ms++)
		{
			_delay_loop_1(5);								// 187.5ns(16MHz에서 계수 '1'의 지연시간)를 5번 하여 950ns~1us 부근의 단위시간 지연 얻기.
		}													// 950ns*1000회=약 0.950ms~1ms 를 단위 지연시간으로 얻어서 넘어오는 계수값 만큼의 ms 지연을획득.
			
		ms_num--;
	}
}


void delay_us(unsigned int us_num)
{	
	for(  ; us_num>0 ; us_num--)
		{
			_delay_loop_1(5);								// 187.5ns(16MHz에서 계수 '1'의 지연시간)를 5번 하여 950ns~1us 부근의 단위시간 지연 얻기.
		}													// 약 1us의 단위 지연시간으로 얻어서 넘어오는 계수값 만큼의 us 지연을획득.		
}



void	set_upper_turn_on()
{
		_UPPER_TURN_ON;
	//	asm("nop"::);
		_UPPER_CTRL_ON;
}

void	set_upper_turn_off()
{
		_UPPER_CTRL_OFF;
	//	asm("nop"::);
		_UPPER_TURN_OFF;
}

void	set_lower_turn_on()
{
		_LOWER_TURN_ON;
	//	asm("nop"::);
		_LOWER_CTRL_ON;
}

void	set_lower_turn_off()
{
		_LOWER_CTRL_OFF;
	//	asm("nop"::);
		_LOWER_TURN_OFF;
}

void	set_discharge_on()
{
		_UPPER_CTRL_ON;
		_LOWER_CTRL_ON;
}

void	set_discharge_off()
{
		_UPPER_CTRL_OFF;
		_LOWER_CTRL_OFF;
}


void	set_active_step3()
{
	asm("nop"::);	// 지연1
	asm("nop"::);	// 지연2
	asm("nop"::);	// 지연3
}




int main(void)
{			
	delay_ms(100);		
	porter_initial();	
	
	timer0_initial();
	
	time2_initial();	
//	timer1_pwm_initial();
//	adc_initial();
//	adc_trig_handdling();	// ADC의 Dummy변환.
//	delay_ms(100);
	
//	TCNT1 = 0;
//	OCR1A = 67;												// Timer1(16bit) PWM Match값을 Step-down 출력전압이 10V가 되게하는 '67'로 설정.
	

			unsigned char	turn_on_unit = 0;
			unsigned char*	turnon_unit_addr = (unsigned char*)0x0200;	// 한 메모리를 지정하여서 Data를 Access 할려고 하는 까닭은 사용중인 MCU의 여러 비 수율된 상황에서 'Dancer_sensor.cpp'에서 확인할 수 있는...
			unsigned int	TCNT1_val = 0;								//  ...것과 같이 임의의 변수명을 지정하여 적재하면 이 부분 역시 되지를 않는 불량부분이기 때문이다. 불량부분들이 너무나 많아서 시험이 어려움.
			unsigned char	basic_turnon_num = 20;
			unsigned int	get_ctrl_turnon_time = 0;
			unsigned int	deadtime_flow_time_num = 35;
			
			bool	b_wave_side_tag = true;
			
			
			*((unsigned char*)turnon_unit_addr) = 0;					// 사용할려는 Add. '0x0200'의 값을 초기화.
	//		check_count = *((unsigned char*)turnon_unit_addr);
	/*													
			clear_eeprom();
			write_eeprom(check_count);
			_LED1_ON;
			while(1);								
	*/
//	sei();
	
	while(1)
	{
		switch(*((unsigned char*)turnon_unit_addr)){					// 일단 반주기의 제어방법으로 단위시간을 '개'로 정의한다. 그러므로 반주기의 시간 4166.66us를 200등분 하여서 그 1개를 20.83us으로 결정한는데...
			case 0:	turn_on_unit = 7;	break;							//  ...정숫값으로 20us로 한다. 그러므로 '7'개의 값으로 할 때 140us의 시간이 된다.
			case 1:	turn_on_unit = 14;	break;
			case 2:	turn_on_unit = 28;	break;
			case 3:	turn_on_unit = 60;	break;
			case 4:	turn_on_unit = 28;	break;
			case 5:	turn_on_unit = 14;	break;
			case 6:	turn_on_unit = 7;	break;
			default:	break;
		}		
		
		get_ctrl_turnon_time = (basic_turnon_num * turn_on_unit)/4;		// 단위시간용 변수값은 'basic_turnon_num'이고 '20'이다. 여기서 제어시간 설정값을 곱하여 제어시간값을 얻었을 때 이를 다시 '4'로 나누는 것...
																		//  .은 이 시간의 제어를 4분주된 Timer0 으로 처리하기 때문이다.
		
		if(b_wave_side_tag)
		{	
			set_upper_turn_on();
			TCNT1 = 0;
			_TIMER1_64_START;						
			
			do 
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < get_ctrl_turnon_time);

			set_upper_turn_off();
			TCNT1 = 0;
			_TIMER1_64_START;			

			do 
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < 3);
	
			
			set_discharge_on();
			do 
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val==deadtime_flow_time_num - 3);
			set_discharge_off();


			do 
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < deadtime_flow_time_num);
		}
		else
		{
			set_lower_turn_on();
			TCNT1 = 0;
			_TIMER1_64_START;
			
			do
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < get_ctrl_turnon_time);

			set_lower_turn_off();
			TCNT1 = 0;
			_TIMER1_64_START;

			do
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < 3);
			
			
			set_discharge_on();
			do
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val==deadtime_flow_time_num - 3);
			set_discharge_off();


			do
			{
				TCNT1_val = TCNT1;
			} while (TCNT1_val < deadtime_flow_time_num);			
		}
	
		
		*((unsigned char*)turnon_unit_addr) = *((unsigned char*)turnon_unit_addr) + 1;
		
		if(*((unsigned char*)turnon_unit_addr)==7){
			*((unsigned char*)turnon_unit_addr) = 0;
			
			if(b_wave_side_tag)
				b_wave_side_tag = false;
			
			else
				b_wave_side_tag = true;						
		}		
	}
}




/*
ISR(ADC_vect)
{
	cli();
	
	if(!(b_dummy_convesion)){
		b_dummy_convesion = true;						// ADC초기화에서 dummy변환이 실행되었다면 이 후의 ADC Interruupt에서는 이 곳을 건너뛴다.
		sei();
		return;
	}	
	
//	sensor_sec_output_adc_result = ADC;
//	_ADC_INTERRUPT_DISABLE;								// 변환이 완료되면 ADC Interrupt를 차단한다.
	
	sei();
}
*/

