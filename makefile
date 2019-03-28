CROSS-COMPILER = arm-none-eabi-
RTOS = tasks.c list.c queue.c port.c heap_4.c timers.c
all: mpu.bin

mpu.bin: main.c blink.c startup.c vector_table.s asm_func.s  $(RTOS)
	$(CROSS-COMPILER)gcc -std=c11 -Wall -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostartfiles -T stm32f4.ld main.c blink.c startup.c $(RTOS) vector_table.s asm_func.s -o mpu.elf
	$(CROSS-COMPILER)objcopy -O binary mpu.elf mpu.bin

flash:
	st-flash --reset write mpu.bin 0x8000000

clean:
	rm -f *.o *.elf *.bin
