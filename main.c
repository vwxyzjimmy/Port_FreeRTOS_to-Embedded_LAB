#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include "reg.h"
#include "blink.h"
#include "asm_func.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"


#define HEAP_MAX (64 * 1024) //64 KB

void vApplicationTickHook() {}
void vApplicationStackOverflowHook() {}
void vApplicationIdleHook() {}
void vApplicationMallocFailedHook() {}

void user_task(void);
void setup_mpu(void);

void init_usart1(void);
void usart1_send_char(const char ch);
void print_sys(char str[255]);
void setup_mpu(void)
{
	//set region 0: flash (0x00000000), 1MB, allow execution, full access, enable all subregion
	//?????
	REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(0x00000000, 0);
	REG(MPU_BASE + MPU_RASR_OFFSET) = MPU_RASR_VALUE(MPU_XN_DISABLE, MPU_AP_FULL_ACCESS, MPU_TYPE_FLASH, 0, MPU_REGION_SIZE_1MB);


	//set region 1: sram (0x20000000), 128KB, forbid execution, full access, enable all subregion
	//?????
	REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(0x20000000, 1);
	REG(MPU_BASE + MPU_RASR_OFFSET) = MPU_RASR_VALUE(MPU_XN_DISABLE, MPU_AP_FULL_ACCESS, MPU_TYPE_SRAM, 0, MPU_REGION_SIZE_128KB);

	//set region 3: GPIOD, 32B, forbid execution, full access, enable all subregion
	//?????
	REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(USART1_BASE , 2);
	REG(MPU_BASE + MPU_RASR_OFFSET) = MPU_RASR_VALUE(MPU_XN_ENABLE, MPU_AP_PRIV_ACCESS, MPU_TYPE_PERIPHERALS, 0, MPU_REGION_SIZE_32B);

	//set region 2: RCC_AHB1ENR, 32B, forbid execution, full access, enable all subregion
	//?????
	REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(RCC_BASE + RCC_AHB1ENR_OFFSET, 3);
	REG(MPU_BASE + MPU_RASR_OFFSET) = MPU_RASR_VALUE(MPU_XN_ENABLE, MPU_AP_FULL_ACCESS, MPU_TYPE_PERIPHERALS, 0, MPU_REGION_SIZE_32B);

	//set region 3: GPIOD, 32B, forbid execution, full access, enable all subregion
	//?????
	REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(GPIO_BASE(GPIO_PORTD), 4);
	REG(MPU_BASE + MPU_RASR_OFFSET) = MPU_RASR_VALUE(MPU_XN_ENABLE, MPU_AP_FULL_ACCESS, MPU_TYPE_PERIPHERALS, 0, MPU_REGION_SIZE_32B);


	//disable region 4 ~ 7
	for (int i=5;i<8;i++)
	{
		//?????
		REG(MPU_BASE + MPU_RBAR_OFFSET) = MPU_RBAR_VALUE(0x00000000, i);
		REG(MPU_BASE + MPU_RASR_OFFSET) = 0;
	}

	//enable the default memory map as a background region for privileged access (PRIVDEFENA)
	//?????
	SET_BIT(MPU_BASE + MPU_CTRL_OFFSET, MPU_PRIVDEFENA_BIT);

	//enable mpu
	//?????
	SET_BIT(MPU_BASE + MPU_CTRL_OFFSET, MPU_ENABLE_BIT);

}

void task1(){
	printf("task1\r\n");
	blink(LED_ORANGE);
}
void task2(){
	printf("task2\r\n");
	blink(LED_GREEN);
}
int main(void)
{
	init_usart1();
	REG(AIRCR_BASE) = NVIC_AIRCR_RESET_VALUE | NVIC_PRIORITYGROUP_4;

	xTaskCreate(task1, "task1", 1000, NULL, 1, NULL);
	xTaskCreate(task2, "task2", 1000, NULL, 1, NULL);
	vTaskStartScheduler();

	blink(LED_BLUE);	//should not go here
	while (1)
		;

	//start_user((uint32_t *)user_task, psp_init);
}

void user_task(void)
{
	printf("Try to create task1 \r\n");
	xTaskCreate(task1, "task1", 1000, NULL, 1, NULL);
	printf("Create task1 and try to create task2\r\n");
	xTaskCreate(task2, "task2", 1000, NULL, 1, NULL);
	printf("Create task2\r\n");
	printf("Start to schedual tasks\r\n");
	vTaskStartScheduler();

	blink(LED_BLUE);	//should not go here
	while (1)
		;
}

void svc_handler_c(uint32_t LR, uint32_t MSP)
{
	/*
	printf("[SVC Handler] LR: 0x%X\r\n", (unsigned int)LR);
	printf("[SVC Handler] MSP Backup: 0x%X \r\n", (unsigned int)MSP);
	printf("[SVC Handler] Control: 0x%X\r\n", (unsigned int)read_ctrl());
	printf("[SVC Handler] SP: 0x%X \r\n", (unsigned int)read_sp());
	printf("[SVC Handler] MSP: 0x%X \r\n", (unsigned int)read_msp());
	printf("[SVC Handler] PSP: 0x%X \r\n\n", (unsigned int)read_psp());
	*/
	uint32_t *stack_frame_ptr;
	if (LR & 0x4) //Test bit 2 of EXC_RETURN
	//if (READ_BIT(((uint32_t *)LR), 2)) //Test bit 2 of EXC_RETURN
	{
		stack_frame_ptr = (uint32_t *)read_psp(); //if 1, stacking used PSP
		//printf("[SVC Handler] Stacking used PSP: 0x%X \r\n\n", (unsigned int)stack_frame_ptr);
	}
	else
	{
		stack_frame_ptr = (uint32_t *)MSP; //if 0, stacking used MSP
		//printf("[SVC Handler] Stacking used MSP: 0x%X \r\n\n", (unsigned int)stack_frame_ptr);
	}

	uint32_t stacked_r0 = *(stack_frame_ptr);
	uint32_t stacked_r1 = *(stack_frame_ptr+1);
	uint32_t stacked_return_addr = *(stack_frame_ptr+6);

	uint16_t svc_instruction = *((uint16_t *)stacked_return_addr - 1);
	uint8_t svc_num = (uint8_t)svc_instruction;
	/*
	printf("[SVC Handler] Stacked R0: 0x%X \r\n", (unsigned int)stacked_r0);
	printf("[SVC Handler] Stacked R1: 0x%X \r\n", (unsigned int)stacked_r1);
	printf("[SVC Handler] SVC number: 0x%X \r\n\n", (unsigned int)svc_num);
	*/
	if (svc_num == 0xA)
		//return r0 + r1
		 *(stack_frame_ptr) = (stacked_r0 + stacked_r1);

	else if (svc_num == 0x4){
		for (unsigned int i = 0; i < (unsigned int)stacked_r1; i++)
			usart1_send_char( *((char *)stacked_r0++) );
 	}

	else
		//return 0
		*(stack_frame_ptr) = 0;
}

void init_usart1(void)
{
	//RCC EN GPIO
	SET_BIT(RCC_BASE + RCC_AHB1ENR_OFFSET, GPIO_EN_BIT(GPIO_PORTB));

	//MODER => 10
	SET_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_MODER_OFFSET, MODERy_1_BIT(6));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_MODER_OFFSET, MODERy_0_BIT(6));

	SET_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_MODER_OFFSET, MODERy_1_BIT(7));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_MODER_OFFSET, MODERy_0_BIT(7));

	//OT => 0
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OTYPER_OFFSET, OTy_BIT(6));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OTYPER_OFFSET, OTy_BIT(7));

	//OSPEEDR => 10
	SET_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_1_BIT(6));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_0_BIT(6));

	SET_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_1_BIT(7));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_OSPEEDR_OFFSET, OSPEEDRy_0_BIT(7));

	//PUPDR = 00 => No pull-up, pull-down
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_PUPDR_OFFSET, PUPDRy_1_BIT(6));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_PUPDR_OFFSET, PUPDRy_0_BIT(6));

	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_PUPDR_OFFSET, PUPDRy_1_BIT(7));
	CLEAR_BIT(GPIO_BASE(GPIO_PORTB) + GPIOx_PUPDR_OFFSET, PUPDRy_0_BIT(7));

	//AF sel
	WRITE_BITS(GPIO_BASE(GPIO_PORTB) + GPIOx_AFRL_OFFSET, AFRLy_3_BIT(6), AFRLy_0_BIT(6), 7);
	WRITE_BITS(GPIO_BASE(GPIO_PORTB) + GPIOx_AFRL_OFFSET, AFRLy_3_BIT(7), AFRLy_0_BIT(7), 7);

	//RCC EN USART1
	SET_BIT(RCC_BASE + RCC_APB2ENR_OFFSET, USART1EN);

	//baud rate
	const unsigned int BAUDRATE = 115200;
	const unsigned int SYSCLK_MHZ = 168;
	const double USARTDIV = SYSCLK_MHZ * 1.0e6 / 8 / 2 / BAUDRATE;

	const uint32_t DIV_MANTISSA = (uint32_t)USARTDIV;
	const uint32_t DIV_FRACTION = (uint32_t)((USARTDIV - DIV_MANTISSA) * 16);

	WRITE_BITS(USART1_BASE + USART_BRR_OFFSET, DIV_MANTISSA_11_BIT, DIV_MANTISSA_0_BIT, DIV_MANTISSA);
	WRITE_BITS(USART1_BASE + USART_BRR_OFFSET, DIV_FRACTION_3_BIT, DIV_FRACTION_0_BIT, DIV_FRACTION);

	//usart1 enable
	SET_BIT(USART1_BASE + USART_CR1_OFFSET, UE_BIT);

	//set TE
	SET_BIT(USART1_BASE + USART_CR1_OFFSET, TE_BIT);

	//set RE
	SET_BIT(USART1_BASE + USART_CR1_OFFSET, RE_BIT);

	//set RXNEIE
	SET_BIT(USART1_BASE + USART_CR1_OFFSET, RXNEIE_BIT);

	//set NVIC
	SET_BIT(NVIC_ISER_BASE + NVIC_ISERn_OFFSET(1), 5); //IRQ37 => (m+(32*n)) | m=5, n=1
}

void usart1_send_char(const char ch)
{
	//wait util TXE == 1

	while (!READ_BIT(USART1_BASE + USART_SR_OFFSET, TXE_BIT))
		;

	REG(USART1_BASE + USART_DR_OFFSET) = (uint8_t)ch;

}

void usart1_handler(void)
{
	if (READ_BIT(USART1_BASE + USART_SR_OFFSET, ORE_BIT))
	{
		for (unsigned int i = 0; i < 5000000; i++)
			;
		char ch = (char)REG(USART1_BASE + USART_DR_OFFSET);

		if (ch == '\r')
			usart1_send_char('\n');

		usart1_send_char(ch);
		usart1_send_char('~');
	}

	else if (READ_BIT(USART1_BASE + USART_SR_OFFSET, RXNE_BIT))
	{
		char ch = (char)REG(USART1_BASE + USART_DR_OFFSET);

		if (ch == '\r')
			usart1_send_char('\n');

		usart1_send_char(ch);
	}
}

void *_sbrk(int incr)
{
	extern uint8_t _mybss_vma_end; //Defined by the linker script
	static uint8_t *heap_end = NULL;
	uint8_t *prev_heap_end;

	if (heap_end == NULL)
		heap_end = &_mybss_vma_end;

	prev_heap_end = heap_end;
	if (heap_end + incr > &_mybss_vma_end + HEAP_MAX)
		return (void *)-1;

	heap_end += incr;
	return (void *)prev_heap_end;
}

int _write(int file, char *ptr, int len)
{
	//sys_call_write(ptr, len);

	for (unsigned int i = 0; i < len; i++)
		usart1_send_char(*ptr++);

	return len;
}

int _close(int file)
{
	return -1;
}

int _lseek(int file, int ptr, int dir)
{
	return 0;
}

int _read(int file, char *ptr, int len)
{
	return 0;
}

int _fstat(int file, struct stat *st)
{
	st->st_mode = S_IFCHR;
	return 0;
}

int _isatty(int file)
{
	return 1;
}
