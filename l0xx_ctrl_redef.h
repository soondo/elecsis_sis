/*
 * l0xx_ctrl_redef.h
 *
 *  Created on: 2018. 10. 14.
 *      Author: soondo
 */

#ifndef L0XX_CTRL_REDEF_H_
#define L0XX_CTRL_REDEF_H_


// #######################     SysTick제어와 결과Flag들의 재 정의     ########################

#define		_SYSTICK_INT_ENABLE		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk																// SysTick을 사용할 때의 인터럽트 허용.
#define		_SYSTICK_INT_DISABLE		if((SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) == 2)	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk
																																																	// SysTick의 인터럽트가 허용되어 있을 때의 차단.

#define		_SYSTICK_START				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk																// SysTick 장치에 Clock이 공급되어 있을 때 계수시작.
#define		_SYSTICK_STOP					if((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 1)		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk
																																																	// SysTick의 계수시작이 허용되어 있을 때 정지처리.
#define		_CHK_SYSTICK_START		(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)															// SysTick의 계수시작이 허용되었는지를 확인.




// ##########################        RCC와 PWR관련 재 정의        ###########################

		// *** 전원ON Reset 후에 RCC_CR은 0300UL 값으로 초기화되는데, 이 값은 MSI이 선택된 것이고 Flag이 확인된 값이다(Start는 MSI 2.1MHz로 시작).
		// *** Clock원의 선택, PLL사용시의 처리, 시스템클록으로의 결정 등의 Macro들.


		// *** HSE장치를 가동하면서 HSE clock감시장치(CSS)도 작동시킨다. HSE clock이 안정되었다면 CSS는 Multi- plexer에서 이 HSE를 선택할 수 있을 것이다.
#define		_HSE_OSC_ON							RCC->CR |= 	RCC_CR_CSSHSEON |	RCC_CR_HSEON 								  // HSE 장치를 가동. 가동 후에는 Clock이 안정적으로 발생하는지를 Flag으로 확인할 것.
#define		_HSE_BYP_ON							RCC->CR |= RCC_CR_CSSHSEON | RCC_CR_HSEON | RCC_CR_HSEBYP  // HSE X-tal을 생략하고 OSC_IN에 외부Clock을 공급할 때...
																												                        	  	  	  	  	  	  	  	  	  	  //   ...(시스템Reset으로 수정되지 않는다) HSE, HSEBYP 두 bit를 '1'로 한다.

#define		_HSE_CLK_TO_SYSCLK			RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSE
																																// HSE의 Clock이 발생되고 있고 이를 시스템클록(SYSCLK)으로 삼고자 한다면 연결장치를 활성. 활성 후 Flag을 확인할 것.

		// *** HSI16을 사용하겠다면
#define		_HSI16_OSC_ON						RCC->CR |= RCC_CR_HSION
#define		_HSI16_ON_AND_DIVIDE			RCC->CR |= RCC_CR_HSION | RCC_CR_HSIDIVEN		// 내부RC 16MHz 장치가동과 분주처리. 가동 후에는 Clock이 안정적으로 발생하는지를 Flag으로 확인할 것.
																																					// 'HSIDIVEN' ---> '1' : 4분주(기본은 '0' : '분주없음)
#define		_HSI16_CLK_TO_SYSCLK		RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI
																																// HSI16의 Clock이 발생되고 있고 이를 시스템클록(SYSCLK)으로 삼고자 한다면 연결장치를 활성.

		// *** PLL을 사용하겠다면
#define		_PLL_EQUIP_OFF						RCC->CR &= ~RCC_CR_PLLON																	// 우선 PLL장치를 한 번 OFF하고...
#define		_PLL_HSE_CLK_SETTING		RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL | 	RCC_CFGR_PLLDIV))) | RCC_CFGR_PLLSRC_HSE |	RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLDIV2
    																															// HSE를 Source로 하겠다면( 체배:8, 분주:2 로 설정 ...... 32MHz의 PLL출력 Clock을 생성)

#define		_PLL_HSI_CLK_SETTING			RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL | 	RCC_CFGR_PLLDIV))) | RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2
																																// HSI16을 Source로 하겠다면( 체배:4, 분주:2 로 설정 ...... 32MHz의 PLL출력 Clock을 생성)
																																												 //    ...일례로 PLL입력클록:HSI16, 체배:4배, 분주:2분주 하여 PLL출력클록을...
																																												 //      ...32MHz의 생성으로 설정시킨 후...
#define		_PLL_EQUIP_ON						RCC->CR |= RCC_CR_PLLON																	 //        ...시스템클록의 설계에서 PLL이 필요하다면 PLL장치를 가동한다.
																																												 //      *** (위의 설정은 반드시 PLL장치가 ON되기 전에 이루어져야 한다)

#define		_PLL_TO_SYSCLK					RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL   // PLL을 시스템클록으로 선택하기 전에 먼저 연결BIT들을 한 번 지운 후...
																																												  //  ...출력되고 있는 PLL클록을 시스템클록(SYSCLK)으로 삼고자 하면 연결장치를 활성한다.



// *****************************       APB1, 2 Clock 연결관련 Macro들     ****************************

          // - APB1, 2는 AHB Clock(HCLK)를 분주하여 각 Bus에 연결하는 Bridge이다.
#define		_EQUIP_PWR_RESET				RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST		// 장치들을 사용하고자 하고 공급전압을 바꾸고자 할 때는 전원을 차단, 장치의 Clock공급을 차단한다.
#define		_EQUIP_PWR_ON						RCC->APB1ENR |= RCC_APB1ENR_PWREN			    // 장치들을 사용하고자 한다면 장치의 전원을공급, Clock을 허용시킨다.
#define		_APB2_SYSCFG_CTRL_ON		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN        // 시스템구성제어기를 사용할 때는 이 장치에 Clock을 허용한다  ---> 1, 2번 공통 아닐까?
#define		_SET_PWR_LEVEL18				PWR->CR = (PWR->CR & (~PWR_CR_VOS)) | PWR_CR_VOS_0		// 장치의 공급전압을 1.8V로 결정한다.
#define		_AHB_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE))						// SYSCLK을 '1'분주하여 AHB장치에서 출력되어 HCLK으로 사용 ---> AHB Bus에 올라간다.
#define		_APB1_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1))					// APB1용 CLK을 '2'분주하여 HCLK을 분주없이 PCLK1로 사용 ---> APB1 Bus에 올라간다.
#define		_APB2_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2))					// APB2용 CLK을 '1'분주하여 HCLK을 분주없이 PCLK2로 사용 ---> APB2 Bus에 올라간다.



// ****************************     각 GPIO의 Clock허용 Macro처리     ****************************

#define		_GPIOA_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPAEN				//Porter A 의 Clock허용 .......... (1UL<<0)
#define		_GPIOB_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPBEN				// Porter B 의 Clock허용.......... (1UL<<1)
#define		_GPIOC_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPCEN				// Porter C 의 Clock허용.......... (1UL<<2)
#define		_GPIOD_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPDEN				// Porter D 의 Clock허용.......... (1UL<<3)
#define		_GPIOE_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPEEN				// Porter E 의 Clock허용.......... (1UL<<4)
#define		_GPIOH_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPHEN				// Porter H 의 Clock허용 ......... (1UL<<7)

#define		_GPIOA_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPAEN			//Portter A의 Clock 차단
#define		_GPIOB_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPBEN			//Portter B의 Clock 차단
#define		_GPIOC_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPCEN			//Portter C의 Clock 차단
#define		_GPIOD_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPDEN			//Portter D의 Clock 차단
#define		_GPIOE_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPEEN			//Portter E의 Clock 차단
#define		_GPIOH_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPHEN			//Portter H의 Clock 차단




// ====================================   GPIOx 들의 입.출력만의 재 정의  ====================================

     /*
 	 	 이들 재 정의는 Cortex-m0+/stm32l0x3 에서 GPIOx 의 Pin들을 구성하는 4개의 Mode가 Typing
		하기에는 가독성이 떨어지므로 GPIOx Pin의 4개 Mode설정 중에서 Alternate function과 Analog의
		Mode설정은 기존의 정의대로 사용하고 핀의 입력과 출력의 설정만 재 정의한다. 																*/

     /*
  	  	  '~GPIO_MODER_MODE0'는 GPIO '0'번 핀의 Mode를 설정하는 2개 bit가 모두 (1:1)의 초기값으로 Analog mode이다.
		이를 입력mode(0:0)로 설정하기 위한 Complemetary이다.																	*/

// ***  GPIO Pin들의 입력재 정의  *** //
// GPIOA Pin들의 입력설정 Macro
#define		i_PA0		GPIO_MODER_MODE0		// 입력선택 '0:0'을 처리하기 위하여 '.. _MODE0' 두 bit만 (1:1)인 (uint32_t)값을 Complementary하여 'RCC->MODER' (uint32_t)값과 AND 시킨다. 또는...
#define		i_PA1		(3ul << 2*1)						//   ...b1:b0을 (1:1)하여 이 두 bit를 Pin'1'에 이동, 고정시킨 후 전체(uint32_t)를 Complementary한 후 'RCC->MODER' (uint32_t)값과 AND 시킨다.
#define		i_PA2		GPIO_MODER_MODE2
#define		i_PA3		GPIO_MODER_MODE3
#define		i_PA4		GPIO_MODER_MODE4
#define		i_PA5		GPIO_MODER_MODE5
#define		i_PA6		GPIO_MODER_MODE6
#define		i_PA7		GPIO_MODER_MODE7
#define		i_PA8		GPIO_MODER_MODE8
#define		i_PA9		GPIO_MODER_MODE9
#define		i_PA10		GPIO_MODER_MODE10
#define		i_PA11		GPIO_MODER_MODE11
#define		i_PA12		GPIO_MODER_MODE12
#define		i_PA13		GPIO_MODER_MODE13
#define		i_PA14		GPIO_MODER_MODE14
#define		i_PA15		GPIO_MODER_MODE15

// GPIOB Pin들의 입력설정 Macro
#define		i_PB0		GPIO_MODER_MODE0
#define		i_PB1		GPIO_MOCER_MODE1
#define		i_PB2		GPIO_MODER_MODE2
#define		i_PB3		GPIO_MODER_MODE3
#define		i_PB4		GPIO_MODER_MODE4
#define		i_PB5		GPIO_MODER_MODE5
#define		i_PB6		GPIO_MODER_MODE6
#define		i_PB7		GPIO_MODER_MODE7
#define		i_PB8		GPIO_MODER_MODE8
#define		i_PB9		GPIO_MODER_MODE9
#define		i_PB10		GPIO_MODER_MODE10
#define		i_PB11		GPIO_MODER_MODE11
#define		i_PB12		GPIO_MODER_MODE12
#define		i_PB13		GPIO_MODER_MODE13
#define		i_PB14		GPIO_MODER_MODE14
#define		i_PB15		GPIO_MODER_MODE15

// GPIOC Pin들의 입력설정 Macro
#define		i_PC0		GPIO_MODER_MODE0
#define		i_PC1		GPIO_MOCER_MODE1
#define		i_PC2		GPIO_MODER_MODE2
#define		i_PC3		GPIO_MODER_MODE3
#define		i_PC4		GPIO_MODER_MODE4
#define		i_PC5		GPIO_MODER_MODE5
#define		i_PC6		GPIO_MODER_MODE6
#define		i_PC7		GPIO_MODER_MODE7
#define		i_PC8		GPIO_MODER_MODE8
#define		i_PC9		GPIO_MODER_MODE9
#define		i_PC10		GPIO_MODER_MODE10
#define		i_PC11		GPIO_MODER_MODE11
#define		i_PC12		GPIO_MODER_MODE12
#define		i_PC13		GPIO_MODER_MODE13
#define		i_PC14		GPIO_MODER_MODE14
#define		i_PC15		GPIO_MODER_MODE15

// GPIOD Pin들의 입력설정 Macro
#define		i_PD0		GPIO_MODER_MODE0
#define		i_PD1		GPIO_MOCER_MODE1
#define		i_PD2		GPIO_MODER_MODE2
#define		i_PD3		GPIO_MODER_MODE3
#define		i_PD4		GPIO_MODER_MODE4
#define		i_PD5		GPIO_MODER_MODE5
#define		i_PD6		GPIO_MODER_MODE6
#define		i_PD7		GPIO_MODER_MODE7
#define		i_PD8		GPIO_MODER_MODE8
#define		i_PD9		GPIO_MODER_MODE9
#define		i_PD10		GPIO_MODER_MODE10
#define		i_PD11		GPIO_MODER_MODE11
#define		i_PD12		GPIO_MODER_MODE12
#define		i_PD13		GPIO_MODER_MODE13
#define		i_PD14		GPIO_MODER_MODE14
#define		i_PD15		GPIO_MODER_MODE15

// GPIOE Pin들의 입력설정 Macro
#define		i_PE0		GPIO_MODER_MODE0
#define		i_PE1		GPIO_MOCER_MODE1
#define		i_PE2		GPIO_MODER_MODE2
#define		i_PE3		GPIO_MODER_MODE3
#define		i_PE4		GPIO_MODER_MODE4
#define		i_PE5		GPIO_MODER_MODE5
#define		i_PE6		GPIO_MODER_MODE6
#define		i_PE7		GPIO_MODER_MODE7
#define		i_PE8		GPIO_MODER_MODE8
#define		i_PE9		GPIO_MODER_MODE9
#define		i_PE10		GPIO_MODER_MODE10
#define		i_PE11		GPIO_MODER_MODE11
#define		i_PE12		GPIO_MODER_MODE12
#define		i_PE13		GPIO_MODER_MODE13
#define		i_PE14		GPIO_MODER_MODE14
#define		i_PE15		GPIO_MODER_MODE15

// GPIOH Pin들의 입력설정 Macro
#define		i_PH0		GPIO_MODER_MODE0
#define		i_PH1		GPIO_MOCER_MODE1
#define		i_PH2		GPIO_MODER_MODE2
#define		i_PH3		GPIO_MODER_MODE3
#define		i_PH4		GPIO_MODER_MODE4
#define		i_PH5		GPIO_MODER_MODE5
#define		i_PH6		GPIO_MODER_MODE6
#define		i_PH7		GPIO_MODER_MODE7
#define		i_PH8		GPIO_MODER_MODE8
#define		i_PH9		GPIO_MODER_MODE9
#define		i_PH10		GPIO_MODER_MODE10
#define		i_PH11		GPIO_MODER_MODE11
#define		i_PH12		GPIO_MODER_MODE12
#define		i_PH13		GPIO_MODER_MODE13
#define		i_PH14		GPIO_MODER_MODE14
#define		i_PH15		GPIO_MODER_MODE15


// ***  GPIO Pin들의 출력재 정의  *** //
// GPIOA Pin들의 출력설정 Macro
#define		o_PA0		GPIO_MODER_MODE0_1					// 출력은 2개 bit가 0:1이므로 'MODEx_1'(1:0) 값을 Complementary하여 (uint32_t)중 'b1'을 '0'처리, 'RCC->MODER'과 AND 처리한다. 또는...
#define		o_PA1		(2ul << 2*1)										//   ...'b1:b0을 1:0으로 만든 후 Pin1에 이동하고 Complementary하여 (uint32_t)중 Pin1의 'b1'을 '0'처리, 'RCC->MODER'과 AND 처리한다.
#define		o_PA2		GPIO_MODER_MODE2_1
#define		o_PA3		GPIO_MODER_MODE3_1
#define		o_PA4		GPIO_MODER_MODE4_1
#define		o_PA5		GPIO_MODER_MODE5_1
#define		o_PA6		GPIO_MODER_MODE6_1
#define		o_PA7		GPIO_MODER_MODE7_1
#define		o_PA8		GPIO_MODER_MODE8_1
#define		o_PA9		GPIO_MODER_MODE9_1
#define		o_PA10		GPIO_MODER_MODE10_1
#define		o_PA11		GPIO_MODER_MODE11_1
#define		o_PA12		GPIO_MODER_MODE12_1
#define		o_PA13		GPIO_MODER_MODE13_1
#define		o_PA14		GPIO_MODER_MODE14_1
#define		o_PA15		GPIO_MODER_MODE15_1

// GPIOB Pin들의 출력설정 Macro
#define		o_PB0		GPIO_MODER_MODE0_1
#define		o_PB1		GPIO_MOCER_MODE1_1
#define		o_PB2		GPIO_MODER_MODE2_1
#define		o_PB3		GPIO_MODER_MODE3_1
#define		o_PB4		GPIO_MODER_MODE4_1
#define		o_PB5		GPIO_MODER_MODE5_1
#define		o_PB6		GPIO_MODER_MODE6_1
#define		o_PB7		GPIO_MODER_MODE7_1
#define		o_PB8		GPIO_MODER_MODE8_1
#define		o_PB9		GPIO_MODER_MODE9_1
#define		o_PB10		GPIO_MODER_MODE10_1
#define		o_PB11		GPIO_MODER_MODE11_1
#define		o_PB12		GPIO_MODER_MODE12_1
#define		o_PB13		GPIO_MODER_MODE13_1
#define		o_PB14		GPIO_MODER_MODE14_1
#define		o_PB15		GPIO_MODER_MODE15_1

// GPIOC Pin들의 출력설정 Macro
#define		o_PC0		GPIO_MODER_MODE0_1
#define		o_PC1		GPIO_MOCER_MODE1_1
#define		o_PC2		GPIO_MODER_MODE2_1
#define		o_PC3		GPIO_MODER_MODE3_1
#define		o_PC4		GPIO_MODER_MODE4_1
#define		o_PC5		GPIO_MODER_MODE5_1
#define		o_PC6		GPIO_MODER_MODE6_1
#define		o_PC7		GPIO_MODER_MODE7_1
#define		o_PC8		GPIO_MODER_MODE8_1
#define		o_PC9		GPIO_MODER_MODE9_1
#define		o_PC10		GPIO_MODER_MODE10_1
#define		o_PC11		GPIO_MODER_MODE11_1
#define		o_PC12		GPIO_MODER_MODE12_1
#define		o_PC13		GPIO_MODER_MODE13_1
#define		o_PC14		GPIO_MODER_MODE14_1
#define		o_PC15		GPIO_MODER_MODE15_1

// GPIOD Pin들의 출력설정 Macro
#define		o_PD0		GPIO_MODER_MODE0_1
#define		o_PD1		GPIO_MOCER_MODE1_1
#define		o_PD2		GPIO_MODER_MODE2_1
#define		o_PD3		GPIO_MODER_MODE3_1
#define		o_PD4		GPIO_MODER_MODE4_1
#define		o_PD5		GPIO_MODER_MODE5_1
#define		o_PD6		GPIO_MODER_MODE6_1
#define		o_PD7		GPIO_MODER_MODE7_1
#define		o_PD8		GPIO_MODER_MODE8_1
#define		o_PD9		GPIO_MODER_MODE9_1
#define		o_PD10		GPIO_MODER_MODE10_1
#define		o_PD11		GPIO_MODER_MODE11_1
#define		o_PD12		GPIO_MODER_MODE12_1
#define		o_PD13		GPIO_MODER_MODE13_1
#define		o_PD14		GPIO_MODER_MODE14_1
#define		o_PD15		GPIO_MODER_MODE15_1

// GPIOE Pin들의 출력설정 Macro
#define		o_PE0		GPIO_MODER_MODE0_1
#define		o_PE1		GPIO_MOCER_MODE1_1
#define		o_PE2		GPIO_MODER_MODE2_1
#define		o_PE3		GPIO_MODER_MODE3_1
#define		o_PE4		GPIO_MODER_MODE4_1
#define		o_PE5		GPIO_MODER_MODE5_1
#define		o_PE6		GPIO_MODER_MODE6_1
#define		o_PE7		GPIO_MODER_MODE7_1
#define		o_PE8		GPIO_MODER_MODE8_1
#define		o_PE9		GPIO_MODER_MODE9_1
#define		o_PE10		GPIO_MODER_MODE10_1
#define		o_PE11		GPIO_MODER_MODE11_1
#define		o_PE12		GPIO_MODER_MODE12_1
#define		o_PE13		GPIO_MODER_MODE13_1
#define		o_PE14		GPIO_MODER_MODE14_1
#define		o_PE15		GPIO_MODER_MODE15_1

// GPIOH Pin들의 출력설정 Macro
#define		o_PH0		GPIO_MODER_MODE0_1
#define		o_PH1		GPIO_MOCER_MODE1_1
#define		o_PH2		GPIO_MODER_MODE2_1
#define		o_PH3		GPIO_MODER_MODE3_1
#define		o_PH4		GPIO_MODER_MODE4_1
#define		o_PH5		GPIO_MODER_MODE5_1
#define		o_PH6		GPIO_MODER_MODE6_1
#define		o_PH7		GPIO_MODER_MODE7_1
#define		o_PH8		GPIO_MODER_MODE8_1
#define		o_PH9		GPIO_MODER_MODE9_1
#define		o_PH10		GPIO_MODER_MODE10_1
#define		o_PH11		GPIO_MODER_MODE11_1
#define		o_PH12		GPIO_MODER_MODE12_1
#define		o_PH13		GPIO_MODER_MODE13_1
#define		o_PH14		GPIO_MODER_MODE14_1
#define		o_PH15		GPIO_MODER_MODE15_1


#endif /* L0XX_CTRL_REDEF_H_ */
