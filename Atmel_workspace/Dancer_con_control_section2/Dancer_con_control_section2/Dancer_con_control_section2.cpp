/*
 * Dancer_con_control_section2.cpp
 *
 * Created: 2018-05-07 오전 9:01:17
 *  Author: soondo
 */ 


/*
;문서번호 : EL-TENSION-PGM-03
;연결문서번호 : EL-TENSION-CIR-01~(구성회로)/EL-TENSION-BOX-ARRA-01(제어기 외함과 단자대 정의도면)
;참조문서번호 : 관련문건 전체
;참조문서번호 : '오차수정의 최적점을 찾는 구조의 재 설계'(2018. 4. 20/금요일 완성)
;프로그램 시작일 : 2018. 5. 7(월)~
;
;
;
;+++++++++++++++++++++++******************        파일명 : Dancer_con_control_section2(ATmega16A ;16MHz)       *****************+++++++++++++++++++++
;
;
;                      ==============================   프로젝트, 파일 및 제어 등의 정의와 도해   ================================
;
;1. 이 프로그램파일은 철선신선기의 장력제어장치인 Dancer Roll 제어기기인 상품명 'Dancer_con' 제품의 시스템 gcc 프로그램이다.
;
;2. 이 프로그램은 FND의 숫자표시 부분만 제외한 제어 전체를 처리하며 마무리 된 '2차 제어구조의 설계'를 따르는 첫 정규 프로그램이다.
;
;3. 이 프로그램은 gcc OOP 의 구조로 제작되며 표시부분의 PCB를 분리하여서 UART로 통신을 확인한 프로그램을 복사하였다. 그러므로 다른 프로젝트와 마찮가지로
;  파일명 번호가 붙으며 맨 끝의 완성되는 파일에는 파일번호와 함께 'FINAL'이라는 명칭이 붙는다. -> 'Dancer_con_control_section2_final'
;
;4. 제어의 기본형식 
;   시스템의 제어기준은 Spooler인버터와 전동기이다. 이 장치의 속도가 상대적으로 Main측 보다도 느리면 위치센서의 값이 위치설정값 보다도 더 커지는 방향이고 
;  이를 '+'오차라고 정의하며 시스템은 이를 수정하기 위하여 Spool인버터에 출력하고 있는 현재의 출력값 보다 더 큰 '+' 값을 출력한다. 이 정의가 기본형식이고
;
;   이는 기본이므로 이 제어의 형태를 표시하지 않는다.
;
;5. 기본형식 반대의 제어형태
;   이는 Spooler가 Main측 속도 보다도 더 느릴 때 위치센서의 값이 위치설정값 보다도 더 작아지는 방향으로서 이를 '-' 오차라고 하고 Spooler는 이를 수정하기
;  위하여 Spool인버터에 현재의 출력값 보다 더 작은값을 출력하는 '-' 값을 출력한다.
;
;   이 제어의 형태는 선택을 하여야 하고 선택시 '현재오차'용 FND의 백 자리에 '-'를 표시한다.
;
;
;
;
;                    ==============================      프로젝트 진행의 현재싯점에서의 기획      ================================
;
;   시험장치를 제작하고 프로그램을 작성할 시의 첫 번째의 기획은 이 8bit, 16MHz의 MCU 하나에서 표시와 제어를 실행하도록 계획하였었다. 그러나 우선의 제어 대
;  상인 Dancer Roll의 위치제어의 Feedding 속도가 평균적으로 20M/초 일수도 있다는 정보에서 아직 시험을 하여 보지는 않았으나 MCU 제어의 속도가  연선 생산의
;  속도를 따르지 못할 수도 있다는 고려에서 제어상의 표시와 제어부분을 분리하였다.
;
;   궁극적으로 이 시험을 진행하면서 MCU를 ARM으로 바꾸는 것으로 결정하였으므로 시험과 동시에 결과와 관계없이 ARM MCU로 회로설계를 하고 Artwork을 진행한다. 
*/



#include <avr/io.h>
#include <util/delay_basic.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
// #include <stdio.h>
#include <stdlib.h>


#define TX_ENABLE_  UCSRB |=(1<<TXEN);
#define TX_DISABLE_ UCSRB &=~(1<<TXEN);
#define RX_ENABLE_  UCSRB |=(1<<RXEN);
#define RX_DISABLE_ UCSRB &=~(1<<RXEN);

#define		_LED2_ON					PORTB &=~(1<<PB0);

#define		_SLEEP_ENABLE				MCUCR |=(1<<SE);			// Build-in의 초기화에서는 'Idle Mode'로 된다.
#define		_SLEEP_DISABLE				MCUCR &=~(1<<SE);

#define		_EEPROM_MST_WRITE			EECR |=(1<<EEMWE);
#define		_EEPROM_WRITE				EECR |=(1<<EEWE);

#define		_SPEED_VOLTAGE_CHNNEL			ADMUX |=(1<<MUX4) | (ADMUX &=~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0))); // Step_down 출력전압 변환용.
#define		_STEP_DOWN_LOAD_A_CHNNEL		ADMUX |=(1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (ADMUX &=~((1<<MUX1) | (1<<MUX0))); // Step_down 부하감시 출력전류 변환용.
#define		_SENSOR_A_CHNNEL				ADMUX |=(1<<MUX2) | (1<<MUX0) | (ADMUX &=~((1<<MUX4) | (1<<MUX3) | (1<<MUX1))); // Step_down 검출 출력전압 변환용.
#define		_REF_VOLTAGE_CHNNEL				ADMUX |=(1<<MUX2) | (1<<MUX1) | (ADMUX &=~((1<<MUX4) | (1<<MUX3) | (1<<MUX0))); // Main인버터 출력주파수값 변환용.
#define		_SENSOR_V_CHNNEL				ADMUX |=(1<<MUX2) | (1<<MUX1) | (1<<MUX0) | (ADMUX &=~((1<<MUX4) | (1<<MUX3))); // Dancer센서 입력값 변환용.

#define		_ADC_ENABLE					ADCSRA |=(1<<ADEN);
#define		_ADC_DISABLE				ADCSRA &=~(1<<ADEN);		
#define		_ADC_AUTO_TRIGGER_ENABLE	ADCSRA |=(1<<ADATE);
#define		_ADC_AUTO_TRIGGER_DISABLE	ADCSRA &=~(1<<ADATE);		// ADC를 Auto Trigger로 선택하면 초기값은 'Free Running'이다. ADC는 변환이 완료되면 트리거 할 수있는 자체 인터럽트를 가지고있다.
#define		_ADC_INTERRUPT_ENABLE		ADCSRA |=(1<<ADIE);
#define		_ADC_INTERRUPT_DISABLE		ADCSRA &=~(1<<ADIE);
//#define		_STOP_ADC					ADCSRA &=~(1<<ADSC);
#define		_ADC_START					ADCSRA |=(1<<ADSC);			// Auto Tigger에서 'Free Runnig'으로 선택하고 ADC Interrupt를 허용하면 현재의 변환이 완료되어을 때 'ADSC'은 '1'로 유지되면서 즉시 다시 변환이 시작...
																	//  ...되어서 Free Running' Mode가 된다. 그리고 이 Mode에서는 변환완료 때의 'ADIF'가 해제되었는지에 관계없이 동작한다.
																	// ADC변환에서의 첫 변환값은 반듯이 버린다.
																	



	
void porter_initial()
{
	DDRA=0b00000000;
	PORTA=0b00000000;

	DDRB=0b00001111;									// PB0은 'LED2' / PB4는 'RUN'
	PORTB=0b00000001;

	DDRC=0b11111111;									// PC는 모두 FND Segment.
	PORTC=0b00000000;
	
	DDRD=0b01111110;									// PD5(OC1A) : PWM-A출력.
	PORTD=0b01000000;	
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

bool	b_dummy_convesion;
void	adc_trig_handdling()
{
	_SLEEP_ENABLE;											//  MCU의 Sleep 진입을 허용하고...
	_ADC_ENABLE;											//   ...ADC장치를 허용한 후...
	_ADC_START;												//    ...ADC변환을 지시한 다음...
	asm("SLEEP" ::);										//	   ...Sleepd으로 진입시켜서 변환을 실행하고... => ADC변환의 시작은 MCU가 Sleep되면 시작된다. 또 Interrupt에서의 복귀는 이 'SLEEP' 명령행 다음 부터 시작한다.
	_ADC_DISABLE											//      ...실행 후 Interrut로 부터 복귀하면 ADC를 차단한 후...
	_SLEEP_DISABLE;											//		 ...sleep도 차단한다.
}


void adc_initial()
{
	ADCSRA |=(1<<ADPS2) | (1<<ADPS1) | (1<<ADPS2);			// 시스템주파수 16MHz를 128분주하여 ADC에 125KHz의 주파수를 공급.
	ADMUX |=(1<<REFS1) | (1<<REFS0);						// ADC의 Vref를 내부 2.56V로 설정.
															// Build-in시의 ADC채널은 Single-end로 'ADC0'으로 초기화 된다.
															
	b_dummy_convesion = true;								// Interrupt로 가면 dummy변환일 때 색인하기 위한 Tag를 하고...
	adc_trig_handdling();									//  ...최초의 변환인 ADC dummy변환을 실행한 후...
	
	while(!(b_dummy_convesion))							//     ...dummy변환이 완료되었는지를 보고 안 되었다면 여기에 붙잡아 둔다.
		;
}


void usart_initial(unsigned int baud)
{
	//	UCSRC |=(1<<UPM1) | (1<<UPM0) | (1<<USMS1) | (1<<USBS0); ...... 이들 bit는 모두 초기값 '0' ; Parity를 사용 안 하고 Stop bit는 1bit.
	
	UCSRC |=(1<<URSEL) | (1<<UCSZ1) | (UCSRC &=~((1<<UCSZ2) | (1<<UCSZ0)));	// data bit를 7bit로 설정 -> 'UCSZ2':'1', 'UCSZ1'과 'UCSZ0'은 '0'.

	UCSRC &=~(1<<URSEL);
	UBRRH = (unsigned char)(baud>>8);						// 비동기 Data bit의 Sampling Clock은 Baud Rata의 16배 주파수로 시스템에서 설정된다.
	UBRRL = (unsigned char)baud;							// Vcc:5V / 시스템클록:16MHz / 103:Baud는 9600bps.
	
	UCSRB |=(1<<RXCIE);										//   ...수신완료 Interrupt 허용.	
	TX_ENABLE_;												// 송.수신장치를 허용.
	RX_ENABLE_;
}


/*
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


void write_eeprom()
{
	unsigned int eeprom_addr_buffer = 0;
	
		
	for(unsigned char i=0 ; i<6 ; i++){	
		while(bit_is_set(EECR, EEWE));							// eeprom이 'Busy'가 아닐 때까지 대기하다가...			
		
		EEAR = eeprom_addr_buffer;
		EEDR = GET_ADC_RESULT_VAL[0+i];
		
		_EEPROM_MST_WRITE;
		_EEPROM_WRITE;										//    ...Free Running으로 기록된 3개 ADC값을 eeprom에 적재한 후...
		eeprom_addr_buffer++;
	}
	
	while(bit_is_set(EECR, EEWE));							//         ...기록의 진행이 종료되었는지를 확인한다.
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



class Cusart_communication
{
		unsigned char	transmit_buffer_data;
		unsigned char	receive_buffer_data;					
		unsigned char	query_code[2];
		
	public:	
			bool	b_chk_interrupt_tag;
			
		Cusart_communication(){
			transmit_buffer_data = 0;
			receive_buffer_data = 0;
			query_code[0] = 'Q';
			query_code[1] = 'C';
			b_chk_interrupt_tag = false;
		}
								
		void	display_data_to_slave(unsigned char);		
		void	query_to_slave();
		void	get_query_data_from_slave();
};	
void Cusart_communication::display_data_to_slave(unsigned char display_val)
{
	transmit_buffer_data = display_val;
	
	while(bit_is_clear(UCSRA, UDRE))
		;
	UDR = transmit_buffer_data;		
}
void Cusart_communication::query_to_slave()
{
	while(bit_is_clear(UCSRA, UDRE))						// 데이타레지스터가 비어 있는지를 보고...
		;
	UDR = query_code[0];									//   ...비었다면 보낼 data가 있는지의 질의Code 2개 중 첫 번째를 전송한 후...
	while(bit_is_clear(UCSRA, UDRE))						//    ...전송이 되었는지를 확인하여...
		;		
	UDR = query_code[1];									//       ...전송이 되어서 데이타레지스터가 비어 있다면 질의Code의 두 번째 byte를 전송한다. -> 이 Frame 뒤의 응답 1byte는 Slave가 Master에 보내는 Data이다.
															//																							 그리고 Master데이타는 pol 없이 그냥 보낸다.	
}
void Cusart_communication::get_query_data_from_slave()
{
	while(bit_is_clear(UCSRA, RXC))							// 수신데이타가 없으면 대기하다가...
		;	
	receive_buffer_data = UDR;								//   수신Buffer로 Data가 들어오면 읽는다.
}

Cusart_communication commun_data;




class Cstart_section
{
	
};



class Cadc
{
	/*	
		#define		_SPEED_VOLTAGE_CHNNEL		; Step_down 출력전압 변환용.
		#define		_STEP_DOWN_LOAD_A_CHNNEL	; Step_down 부하 감시용 출력전류 변환용.
		#define		_SENSOR_A_CHNNEL			; 위치센서 입력이 전류(4~20mA)일 때의 변환용.
		#define		_REF_VOLTAGE_CHNNEL			; Main인버터 출력주파수값 변환용.
		#define		_SENSOR_V_CHNNEL			; 위치센서 입력이 전압(0~10VDC)일 때의 변환용.
			
		단일변환 : ADC=Vin*1024/Vref => Vin=ADC*Vref/1024
		차동변환 : ADC=(Vpos - Vneg)*Gain*512/Vref => (Vpos_Vneg)=ADC*Vref/Gain*512
	*/
		float	adc_speed_output_result_val;	
		float	adc_step_down_A_monit_val;
		float	adc_sensor_input_A_val;
		float	adc_ref_input_voltage_val;
		float	adc_sensor_input_V_val;
		
	public:
		Cadc():adc_speed_output_result_val(0), adc_step_down_A_monit_val(0), adc_sensor_input_A_val(0), adc_ref_input_voltage_val(0), adc_sensor_input_V_val(0) {}
	
		float	adc_chnnel_speed_output_voltage();
		float	adc_chnnel_step_down_load_monit();
		float	adc_chnnel_sensor_A_input();
		float	adc_chnnel_ref_input_voltage();
		float	adc_chnnel_sensor_V_input();
};
float	Cadc::adc_chnnel_speed_output_voltage()
{
		_SPEED_VOLTAGE_CHNNEL;
		adc_trig_handdling();
		adc_speed_output_result_val = ADC;
		
		return adc_speed_output_result_val;
}

float	Cadc::adc_chnnel_step_down_load_monit()
{
		_STEP_DOWN_LOAD_A_CHNNEL;
		adc_trig_handdling();
		adc_step_down_A_monit_val = ADC;
		
		return adc_step_down_A_monit_val;
}

float	Cadc::adc_chnnel_sensor_A_input()
{
		_SENSOR_A_CHNNEL;
		adc_trig_handdling();
		adc_sensor_input_A_val = ADC;
		
		return adc_sensor_input_A_val;
}

float	Cadc::adc_chnnel_ref_input_voltage()
{
		_REF_VOLTAGE_CHNNEL;
		adc_trig_handdling();
		adc_ref_input_voltage_val = ADC;
		
		return adc_ref_input_voltage_val;
}

float	Cadc::adc_chnnel_sensor_V_input()
{
		_SENSOR_V_CHNNEL;
		adc_trig_handdling();
		adc_sensor_input_V_val = ADC;
		
		return adc_sensor_input_V_val;
}	

Cadc adc_result_read;
	



// 시스템 'RUN' 전의 Dancer Roll을 움직여서 표시를 확인하는 과정
class Cpre_process_drive_start
{	
		unsigned char	dancer_position_display_val;
		
	public:		
		Cpre_process_drive_start():dancer_position_display_val(0) {}
	/*
		{
			unsigned char* position_display;
			position_display = (unsigned char*)malloc(sizeof(unsigned char));
			*position_display = dancer_position_display_val;
		}
	*/
		void	check_dancer_roll_swing();				
};
void	Cpre_process_drive_start::check_dancer_roll_swing()										  // Dancer Roll을 움직여서 위치변환을 표시확인을 수행하는 이 함수에서...
{																			  
	dancer_position_display_val = static_cast<float>(adc_result_read.adc_chnnel_sensor_V_input());//    ...이 값을 FND 표시용 Slave에 보내는 변수에 형 변환하여 적재한 후...
	
	commun_data.display_data_to_slave(dancer_position_display_val);								  //        ...이 값을 표시하기 위하여 UART로 Slave에 보낸다. 
}




class Ctolerance_removal_buffer
{
		unsigned char	get_tolerance_removal_template_num;
		unsigned char	get_tolerance_removal_template_old_num1;
		unsigned char	get_tolerance_removal_template_old_num2;
				float	get_start_point_tolerance_val;
				float	get_end_point_tolerance_val;
				float	get_relative_tolerance_val;
				float	get_trt_before_speed_val;
				bool	get_tolerance_val_signed_tag;				
		
	public:
		Ctolerance_removal_buffer():get_tolerance_removal_template_num(0), get_start_point_tolerance_val(0), get_end_point_tolerance_val(0),
							 get_relative_tolerance_val(0), get_trt_before_speed_val(0), get_tolerance_val_signed_tag(false) {}
								 		
		void	buff_tolerance_removal_num();
		void	buff_tolerance_removal_tag();
		void	buff_tolerance_removal_val();		
};



class Ctolerance_removal_template_num_table
{		
		unsigned char	get_setting_speed_ocr1a_val; 
		unsigned char	set_removal_schedule_count;		
		static const unsigned char	trt_table_num[20];
		
	public:
		unsigned char*	get_trt_table_num;
		
		Ctolerance_removal_template_num_table();
		unsigned char	h_tolerance_removal();				 
	
};
const unsigned char Ctolerance_removal_template_num_table::trt_table_num[20]={
	1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20 };


	
int main(void)
{						
	porter_initial();	
	timer1_pwm_initial();
	usart_initial(103);										// 시스템Clock 16MHz에서 Baud=9600bps 설정.
	adc_initial();

	delay_ms(200);
	sei();	
	TCNT1 = 0;
//	OCR1A = 67;												// Timer1(16bit) PWM Match값을 Step-down 출력전압이 10V가 되게하는 '67'로 설정.
//	_LED2_ON;	
	
	Cpre_process_drive_start Position;
							
	while(1){
		Position.check_dancer_roll_swing();
		delay_ms(300);
	}


	/*	
	_ADC0_P_ADC1_N_DIFFERENTIAL;							// 채널을 차동의 ADC0('+')/ADC1('-')로 설정 => Step-down 출력전압의 검출변환용.	
	_DAN_ADC7;												// Dancer센서용 Single-end 입력채널(ADC7).
	_REF_ADC6;												// Main인버터 출력주파수 대응출력값 입력채널(ADC6).
	_ADC_INTERRUPT_ENABLE;
	_ADC_AUTO_TRIGGER_ENABLE;								// Free Running을 위하여 필요.
	
	
	while(1);

	delay_ms(500);
		
	while(1){
		while(bit_is_clear(PIND, PD7));						// TACT Switch를 눌릴 때까지 대기하다가...
												
		if(bit_is_set(PIND, PD7)){							//   ...눌렀다면...
			delay_ms(100);									//	  ...100ms 후...
								
			if(bit_is_set(PIND, PD7)){						//      ...확실히 눌린게 맞는지를 보고확실하다면...
				while(bit_is_set(PIND, PD7));				//       ...이제 눌렀던 TACT Switch를 놓기를 대기하다가...
				delay_ms(100);								//		  ...놓으면 100ms 후에...
								
				_SLEEP_ENABLE;								//			...MCU의 Sleep 진입과...
				_ADC_ENABLE;								//			 ...ADC를 허용한 후...
				sei();										//			  ...Gloval Interrupt를 허용과...	
				_START_ADC;									//			   ...ADC변환을 지시하고...
				
				for(static unsigned char i=0 ; i<4 ; i++){				
					asm("SLEEP" ::);						//				  ...Sleepd으로 진입시킨다. => ADC변환의 시작은 MCU가 Sleep되면 시작된다. 또 Interrupt에서의 복귀는 이 'SLEEP' 명령행 다음 부터 시작한다.
				}								
				_ADC_DISABLE								// 연속변환된 3개의 ADC변환이 완료되었다면 ADC를 차단하고...
				_SLEEP_DISABLE;								//  ...'SLEEP'도 차단한 후...
				
				write_eeprom();								//    ...기록된 3개 ADC변환값을 eeprom에 적재한다.
				
				_LED1_ON
				GET_ADC_INTERRUPT_COUNT_NUM = 0;												
			}
		}
	}
	*/							
}



/* - Master와 Slave는 1byte의 송.수신을 9개 bit의 길이로 설정, 사용한다. 9개 bit 길이의 'TXB8'과 'RXB8'은 Multi- drop용으로 설계되고 사용되나 여기서는 Master의 송신시에만 사용하는데 패리티는 사용
    하지 않으며 1bit의 정지bit를 사용한다. 그리고 하나의 외함에서 가까이 Master와 Slave가 근접하여 있으므로 'FE', DOR' 및 'PE'는 검사하지 않는다.
	 만일 Master가 전송하는 Data가 Pol인 Query의 2개 byte라면 TXB8 ='1'로 처리, 전송하고 Data인 1개 byte라면 TXB8='0'으로 보낸다.
	
   - 이 프로젝트에서 Master의 Slave로의 전송은 연속된 2개 byte의 Pol용 Query와 바로 보내는 1개 byte Data의 두 종류로 구성된다. 그리고 Slave에서 Maste로 보내는 것은 1개 byte의 Data 한 종류이다.
    그러므로 Master가 Slave에 전송하는 Data가 Pol이여서 TX8='1'이라면 Query의 2개 byte Frame으로서 Data의 1개 byte와는 구분되므로 Slave는 이 TX8='1' 만 확인하면 Query인지 알 수가 있다. 즉 2개 
	byte 각각의 내용을 확인하지 않아도 되는 것이다. 그러나 Frame수신에서의 확인Precedure가 이루어지는 것이 통상이므로 확인을 한다.
	
   - Slave가 Master로 보내는 Data는 언제나 1byte이므로 TXB8은 항상 '0'이다.	
*/

ISR(USART_RXC_vect)
{
	cli();	
	commun_data.get_query_data_from_slave();
	commun_data.b_chk_interrupt_tag = true;
	sei();
}



ISR(ADC_vect)
{
	cli();
	if(!(b_dummy_convesion)){
		b_dummy_convesion = true;					// ADC초기화에서 dummy변환이 실행되었다면 이 후의 ADC Interruupt에서는 이 곳을 건너뛴다.
		return;
	}

	
	sei();
}



