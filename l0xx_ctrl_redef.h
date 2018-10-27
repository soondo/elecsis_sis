/*
 * l0xx_ctrl_redef.h
 *
 *  Created on: 2018. 10. 14.
 *      Author: soondo
 */

#ifndef L0XX_CTRL_REDEF_H_
#define L0XX_CTRL_REDEF_H_


// #######################     SysTick����� ���Flag���� �� ����     ########################

#define		_SYSTICK_INT_ENABLE		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk																// SysTick�� ����� ���� ���ͷ�Ʈ ���.
#define		_SYSTICK_INT_DISABLE		if((SysTick->CTRL & SysTick_CTRL_TICKINT_Msk) == 2)	SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk
																																																	// SysTick�� ���ͷ�Ʈ�� ���Ǿ� ���� ���� ����.

#define		_SYSTICK_START				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk																// SysTick ��ġ�� Clock�� ���޵Ǿ� ���� �� �������.
#define		_SYSTICK_STOP					if((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) == 1)		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk
																																																	// SysTick�� ��������� ���Ǿ� ���� �� ����ó��.
#define		_CHK_SYSTICK_START		(SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)															// SysTick�� ��������� ���Ǿ������� Ȯ��.




// ##########################        RCC�� PWR���� �� ����        ###########################

		// *** ����ON Reset �Ŀ� RCC_CR�� 0300UL ������ �ʱ�ȭ�Ǵµ�, �� ���� MSI�� ���õ� ���̰� Flag�� Ȯ�ε� ���̴�(Start�� MSI 2.1MHz�� ����).
		// *** Clock���� ����, PLL������ ó��, �ý���Ŭ�������� ���� ���� Macro��.


		// *** HSE��ġ�� �����ϸ鼭 HSE clock������ġ(CSS)�� �۵���Ų��. HSE clock�� �����Ǿ��ٸ� CSS�� Multi- plexer���� �� HSE�� ������ �� ���� ���̴�.
#define		_HSE_OSC_ON							RCC->CR |= 	RCC_CR_CSSHSEON |	RCC_CR_HSEON 								  // HSE ��ġ�� ����. ���� �Ŀ��� Clock�� ���������� �߻��ϴ����� Flag���� Ȯ���� ��.
#define		_HSE_BYP_ON							RCC->CR |= RCC_CR_CSSHSEON | RCC_CR_HSEON | RCC_CR_HSEBYP  // HSE X-tal�� �����ϰ� OSC_IN�� �ܺ�Clock�� ������ ��...
																												                        	  	  	  	  	  	  	  	  	  	  //   ...(�ý���Reset���� �������� �ʴ´�) HSE, HSEBYP �� bit�� '1'�� �Ѵ�.

#define		_HSE_CLK_TO_SYSCLK			RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSE
																																// HSE�� Clock�� �߻��ǰ� �ְ� �̸� �ý���Ŭ��(SYSCLK)���� ����� �Ѵٸ� ������ġ�� Ȱ��. Ȱ�� �� Flag�� Ȯ���� ��.

		// *** HSI16�� ����ϰڴٸ�
#define		_HSI16_OSC_ON						RCC->CR |= RCC_CR_HSION
#define		_HSI16_ON_AND_DIVIDE			RCC->CR |= RCC_CR_HSION | RCC_CR_HSIDIVEN		// ����RC 16MHz ��ġ������ ����ó��. ���� �Ŀ��� Clock�� ���������� �߻��ϴ����� Flag���� Ȯ���� ��.
																																					// 'HSIDIVEN' ---> '1' : 4����(�⺻�� '0' : '���־���)
#define		_HSI16_CLK_TO_SYSCLK		RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_HSI
																																// HSI16�� Clock�� �߻��ǰ� �ְ� �̸� �ý���Ŭ��(SYSCLK)���� ����� �Ѵٸ� ������ġ�� Ȱ��.

		// *** PLL�� ����ϰڴٸ�
#define		_PLL_EQUIP_OFF						RCC->CR &= ~RCC_CR_PLLON																	// �켱 PLL��ġ�� �� �� OFF�ϰ�...
#define		_PLL_HSE_CLK_SETTING		RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL | 	RCC_CFGR_PLLDIV))) | RCC_CFGR_PLLSRC_HSE |	RCC_CFGR_PLLMUL8 | RCC_CFGR_PLLDIV2
    																															// HSE�� Source�� �ϰڴٸ�( ü��:8, ����:2 �� ���� ...... 32MHz�� PLL��� Clock�� ����)

#define		_PLL_HSI_CLK_SETTING			RCC->CFGR = (RCC->CFGR & (~(RCC_CFGR_PLLMUL | 	RCC_CFGR_PLLDIV))) | RCC_CFGR_PLLSRC_HSI | RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2
																																// HSI16�� Source�� �ϰڴٸ�( ü��:4, ����:2 �� ���� ...... 32MHz�� PLL��� Clock�� ����)
																																												 //    ...�Ϸʷ� PLL�Է�Ŭ��:HSI16, ü��:4��, ����:2���� �Ͽ� PLL���Ŭ����...
																																												 //      ...32MHz�� �������� ������Ų ��...
#define		_PLL_EQUIP_ON						RCC->CR |= RCC_CR_PLLON																	 //        ...�ý���Ŭ���� ���迡�� PLL�� �ʿ��ϴٸ� PLL��ġ�� �����Ѵ�.
																																												 //      *** (���� ������ �ݵ�� PLL��ġ�� ON�Ǳ� ���� �̷������ �Ѵ�)

#define		_PLL_TO_SYSCLK					RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_SW)) | RCC_CFGR_SW_PLL   // PLL�� �ý���Ŭ������ �����ϱ� ���� ���� ����BIT���� �� �� ���� ��...
																																												  //  ...��µǰ� �ִ� PLLŬ���� �ý���Ŭ��(SYSCLK)���� ����� �ϸ� ������ġ�� Ȱ���Ѵ�.



// *****************************       APB1, 2 Clock ������� Macro��     ****************************

          // - APB1, 2�� AHB Clock(HCLK)�� �����Ͽ� �� Bus�� �����ϴ� Bridge�̴�.
#define		_EQUIP_PWR_RESET				RCC->APB1RSTR |= RCC_APB1RSTR_PWRRST		// ��ġ���� ����ϰ��� �ϰ� ���������� �ٲٰ��� �� ���� ������ ����, ��ġ�� Clock������ �����Ѵ�.
#define		_EQUIP_PWR_ON						RCC->APB1ENR |= RCC_APB1ENR_PWREN			    // ��ġ���� ����ϰ��� �Ѵٸ� ��ġ�� ����������, Clock�� ����Ų��.
#define		_APB2_SYSCFG_CTRL_ON		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN        // �ý��۱�������⸦ ����� ���� �� ��ġ�� Clock�� ����Ѵ�  ---> 1, 2�� ���� �ƴұ�?
#define		_SET_PWR_LEVEL18				PWR->CR = (PWR->CR & (~PWR_CR_VOS)) | PWR_CR_VOS_0		// ��ġ�� ���������� 1.8V�� �����Ѵ�.
#define		_AHB_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_HPRE))						// SYSCLK�� '1'�����Ͽ� AHB��ġ���� ��µǾ� HCLK���� ��� ---> AHB Bus�� �ö󰣴�.
#define		_APB1_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE1))					// APB1�� CLK�� '2'�����Ͽ� HCLK�� ���־��� PCLK1�� ��� ---> APB1 Bus�� �ö󰣴�.
#define		_APB2_CLK_DIV						RCC->CFGR = (RCC->CFGR & (~RCC_CFGR_PPRE2))					// APB2�� CLK�� '1'�����Ͽ� HCLK�� ���־��� PCLK2�� ��� ---> APB2 Bus�� �ö󰣴�.



// ****************************     �� GPIO�� Clock��� Macroó��     ****************************

#define		_GPIOA_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPAEN				//Porter A �� Clock��� .......... (1UL<<0)
#define		_GPIOB_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPBEN				// Porter B �� Clock���.......... (1UL<<1)
#define		_GPIOC_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPCEN				// Porter C �� Clock���.......... (1UL<<2)
#define		_GPIOD_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPDEN				// Porter D �� Clock���.......... (1UL<<3)
#define		_GPIOE_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPEEN				// Porter E �� Clock���.......... (1UL<<4)
#define		_GPIOH_CLK_ENABLE				RCC->IOPENR |= RCC_IOPENR_IOPHEN				// Porter H �� Clock��� ......... (1UL<<7)

#define		_GPIOA_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPAEN			//Portter A�� Clock ����
#define		_GPIOB_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPBEN			//Portter B�� Clock ����
#define		_GPIOC_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPCEN			//Portter C�� Clock ����
#define		_GPIOD_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPDEN			//Portter D�� Clock ����
#define		_GPIOE_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPEEN			//Portter E�� Clock ����
#define		_GPIOH_CLK_DISABLE				RCC->IOPENR &= ~RCC_IOPENR_IOPHEN			//Portter H�� Clock ����




// ====================================   GPIOx ���� ��.��¸��� �� ����  ====================================

     /*
 	 	 �̵� �� ���Ǵ� Cortex-m0+/stm32l0x3 ���� GPIOx �� Pin���� �����ϴ� 4���� Mode�� Typing
		�ϱ⿡�� �������� �������Ƿ� GPIOx Pin�� 4�� Mode���� �߿��� Alternate function�� Analog��
		Mode������ ������ ���Ǵ�� ����ϰ� ���� �Է°� ����� ������ �� �����Ѵ�. 																*/

     /*
  	  	  '~GPIO_MODER_MODE0'�� GPIO '0'�� ���� Mode�� �����ϴ� 2�� bit�� ��� (1:1)�� �ʱⰪ���� Analog mode�̴�.
		�̸� �Է�mode(0:0)�� �����ϱ� ���� Complemetary�̴�.																	*/

// ***  GPIO Pin���� �Է��� ����  *** //
// GPIOA Pin���� �Է¼��� Macro
#define		i_PA0		GPIO_MODER_MODE0		// �Է¼��� '0:0'�� ó���ϱ� ���Ͽ� '.. _MODE0' �� bit�� (1:1)�� (uint32_t)���� Complementary�Ͽ� 'RCC->MODER' (uint32_t)���� AND ��Ų��. �Ǵ�...
#define		i_PA1		(3ul << 2*1)						//   ...b1:b0�� (1:1)�Ͽ� �� �� bit�� Pin'1'�� �̵�, ������Ų �� ��ü(uint32_t)�� Complementary�� �� 'RCC->MODER' (uint32_t)���� AND ��Ų��.
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

// GPIOB Pin���� �Է¼��� Macro
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

// GPIOC Pin���� �Է¼��� Macro
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

// GPIOD Pin���� �Է¼��� Macro
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

// GPIOE Pin���� �Է¼��� Macro
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

// GPIOH Pin���� �Է¼��� Macro
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


// ***  GPIO Pin���� ����� ����  *** //
// GPIOA Pin���� ��¼��� Macro
#define		o_PA0		GPIO_MODER_MODE0_1					// ����� 2�� bit�� 0:1�̹Ƿ� 'MODEx_1'(1:0) ���� Complementary�Ͽ� (uint32_t)�� 'b1'�� '0'ó��, 'RCC->MODER'�� AND ó���Ѵ�. �Ǵ�...
#define		o_PA1		(2ul << 2*1)										//   ...'b1:b0�� 1:0���� ���� �� Pin1�� �̵��ϰ� Complementary�Ͽ� (uint32_t)�� Pin1�� 'b1'�� '0'ó��, 'RCC->MODER'�� AND ó���Ѵ�.
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

// GPIOB Pin���� ��¼��� Macro
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

// GPIOC Pin���� ��¼��� Macro
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

// GPIOD Pin���� ��¼��� Macro
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

// GPIOE Pin���� ��¼��� Macro
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

// GPIOH Pin���� ��¼��� Macro
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
