/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jun 4, 2024
 *      Author: Karan Patel
 */

#include "stm32f407xx_gpio_driver.h"



//Peripheral clock setup

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx,uint8_t EnorDi){

	if(EnorDi==ENABLE){
		 if(pGPIOx==GPIOA){
			 GPIOA_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOB){
			 GPIOB_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOC){
			 GPIOC_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOD){
			 GPIOD_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOE){
			 GPIOE_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOF){
			 GPIOF_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOG){
			 GPIOG_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOH){
		     GPIOH_PCLK_EN();
		 }
		 else if(pGPIOx==GPIOI){
			  GPIOI_PCLK_EN();
	     }
	}
	else {
		if(pGPIOx==GPIOA){
					 GPIOA_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOB){
					 GPIOB_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOC){
					 GPIOC_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOD){
					 GPIOD_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOE){
					 GPIOE_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOF){
					 GPIOF_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOG){
					 GPIOG_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOH){
				     GPIOH_PCLK_DI();
				 }
				 else if(pGPIOx==GPIOI){
					  GPIOI_PCLK_DI();
			     }
	}
}

//  GPIO_INIT

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/


//Init and Deinit
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){

	GPIO_PeriClockControl(pGPIOHandle->pGPIOX,ENABLE);
	//1) Configure the mode of the GPIO pin.
	uint32_t temp=0;
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<=GPIO_MODE_ANALOG){
	  //the non interrupt mode.
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOX->MODER&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);//Clearing.
		pGPIOHandle->pGPIOX->MODER|=temp;
	}
	else {
     if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_FT){
          //Configure the FTSR.
    	 EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	 //Clear the corresponding RTSR bit.
    	 EXTI->RTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     }
     else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_IT_RT){
    	  //Configure the RTSR.
    	    	 EXTI->RTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	    	 //Clear the corresponding FTSR bit.
    	    	 EXTI->FTSR&=~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     }
     else {
    	 EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
    	    	    	 //Clear the corresponding FTSR bit.
    	 EXTI->FTSR|=(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
     }
   //2)Configure the GPIO port selection in SYSCFG_EXTICR(To select which port provides an interrupt.
     uint8_t temp1= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)/4;
     uint8_t temp2= (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)%4;
     uint8_t portcode=(GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOX));
     SYSCFG_PCLK_EN();
     SYSCFG->EXTICR[temp1]=portcode<<(temp2*4);
   //3)Enable the exti interrupt delivery using IMR.

     EXTI->IMR|=(1<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	temp=0;
	//2) Configure the speed.

	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->OSPEEDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OSPEEDR|=temp;

	temp=0;

	//3) Configure the pupd settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl<<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOX->PUPDR&=~(0x3<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->PUPDR|=temp;
	temp=0;
	//4) Configure the optype.
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER &=~(0x1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOX->OTYPER|=temp;

	//5) Configure the alternate functionality.
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_MODE_ALTFN){
       uint8_t temp1,temp2;
       temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
       temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;
       pGPIOHandle->pGPIOX->AFR[temp1]&=~(0XF<<(4*temp2));
       pGPIOHandle->pGPIOX->AFR[temp1]|=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode<<(4*temp2));
		}
}

//  GPIO_DEINIT

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	 if(pGPIOx==GPIOA){
				 GPIOA_REG_RESET();
			 }
			 else if(pGPIOx==GPIOB){
				 GPIOB_REG_RESET();
			 }
			 else if(pGPIOx==GPIOC){
				 GPIOC_REG_RESET();
			 }
			 else if(pGPIOx==GPIOD){
				 GPIOD_REG_RESET();
			 }
			 else if(pGPIOx==GPIOE){
				 GPIOE_REG_RESET();
			 }
			 else if(pGPIOx==GPIOF){
				 GPIOF_REG_RESET();
			 }
			 else if(pGPIOx==GPIOG){
				 GPIOG_REG_RESET();
			 }
			 else if(pGPIOx==GPIOH){
				 GPIOH_REG_RESET();
			 }
			 else if(pGPIOx==GPIOI){
				 GPIOI_REG_RESET();
		     }

}

//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
   uint8_t value;
   value=(uint8_t)((pGPIOx->IDR >>PinNumber)&0x00000001);
   return value;
}
//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/

uint16_t  GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	 uint16_t value;
	   value=(uint16_t)pGPIOx->IDR;
	   return value;
}
//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t Value){
       if(Value==GPIO_PIN_SET){
    	   // write 1 to output data register at the bit field corresponding to pin number.
    	   pGPIOx->ODR|=(1<<PinNumber);
       }
       else {
    	   //write 0
    	   pGPIOx->ODR&=~(1<<PinNumber);
       }
}
//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t Value){
    pGPIOx->ODR=Value;
}
//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR^=(1<<PinNumber);
}
//Data read and write.
//  GPIO_ReadFromInputPin

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/

///IRQ configuration and ISR handling.
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi){
	 if(EnorDi==ENABLE){
		 if(IRQNumber<=31){
			//Program ISER0 register.
           *NVIC_ISER0|=(1<<IRQNumber);
		 }
		 else if(IRQNumber>31 &&IRQNumber<64){
			 //Program ISER1 register.
		  *NVIC_ISER1|=(1<<(IRQNumber)%32);
		 }
		 else if(IRQNumber>=64 &&IRQNumber<96){
			 //Program ISER2 register
			 *NVIC_ISER2|=(1<<(IRQNumber)%64);
		 }
	 }
	 else {
		 if(IRQNumber<=31){
			 *NVIC_ICER0|=(1<<IRQNumber);
		   }
         else if(IRQNumber>31 &&IRQNumber<64){
        	 *NVIC_ICER1|=(1<<(IRQNumber)%32);
		   }
		 else if(IRQNumber>=64 &&IRQNumber<96){
			 *NVIC_ICER2|=(1<<(IRQNumber)%64);
		   }
	 }
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	//First found the ipr register.
	// Lower 4 bits are not applicable for every register.
     uint8_t iprx=IRQNumber/4;
     uint8_t iprx_section=IRQNumber%4;
     uint8_t shift_amount=(8*iprx_section)+(8-NO_PR_BITS_IMPLEMENTED);
     *(NVIC_PR_BASE_ADDR+(4*iprx))|=(IRQPriority<<shift_amount);
}
void GPIO_IRQHandling(uint8_t PinNumber){
   // Clear the exti pr register corresponding to the pin number.
	if(EXTI->PR&(1<<PinNumber)){
		// CLEAR
		EXTI->PR|=(1<<PinNumber);
	}
}
