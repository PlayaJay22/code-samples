/*
 * libs.c
 *
 *  Created on: Oct 10, 2022
 *      Author: Jason
 */

#include <stdint.h>
#include <stdio.h>
#include "libs.h"
#include "led.h"

//global variables
uint32_t TaskStackPointers[MAX_TASKS] = {IDLE_STACK_START, T1_STACK_START, T2_STACK_START, T3_STACK_START, T4_STACK_START};
uint32_t TaskHandlers[MAX_TASKS] = {(uint32_t)idle_task, (uint32_t)task1_handler, (uint32_t)task2_handler, (uint32_t)task3_handler, (uint32_t)task4_handler};
uint8_t currentTask = 1;  //task 1 is first task running
uint32_t g_tick_count = 0;

TCB_t userTasks[MAX_TASKS];

void task1_handler(void)
{
	while(1)
	{
		printf("Task 1 is executing...\n");
		led_on(LED_GREEN);
		task_delay(1000); //1000ms
		led_off(LED_GREEN);
		task_delay(1000);
	}
}


void task2_handler(void)
{
	while(1)
	{
		led_on(LED_ORANGE);
		task_delay(500);
		led_off(LED_ORANGE);
		task_delay(500);
	}
}

void task3_handler(void)
{
	while(1)
	{
		led_on(LED_BLUE);
		task_delay(250);
		led_off(LED_BLUE);
		task_delay(250);
	}
}


void task4_handler(void)
{
	while(1)
	{
		led_on(LED_RED);
		task_delay(125);
		led_off(LED_RED);
		task_delay(125);
	}
}

void idle_task(void)
{
	while(1);
}

__attribute__ ((naked)) void PendSV_Handler (void)
{
	//save current task

	//1. get current task's SP into R0
	__asm volatile ("MRS r0, PSP");

	//using R0, copy registers R4-R11 into current task stack
	__asm volatile ("STMDB R0!, {R4-R11}");

	//save LR
	__asm volatile ("PUSH {LR}");

	//save new location of R0 into PSP
	__asm volatile ("BL save_psp_value");

	//retrieve next task//

	//1. Decide next task to run
	__asm ("BL update_next_task");

	//2. get it's past PSP value
	__asm ("BL get_psp_value");

	//3. using that PSP value, retrieve SF2(R4-R11)
	__asm ("LDMIA r0!, {R4-R11}");

	//4. update PSP and exit
	__asm ("MSR PSP, R0");

	//retrieve LR to return back to proper place
	__asm volatile ("POP {LR}");

	//manually trigger exception exit by writing EXC_RETURN to PC
	__asm volatile ("BX LR");
}

void update_global_tick_count(void)
{
	g_tick_count++;
}
void unblock_tasks(void)
{
	for(int i = 1; i < MAX_TASKS; i++)
	{
		if(userTasks[i].currentState != TASK_READY_STATE)
			if(userTasks[i].blockCount == g_tick_count)
				userTasks[i].currentState = TASK_READY_STATE;

	}

}




void init_systick_timer(uint32_t tick_hz)
{
	/* SYSTICK program order
	 *  1. Program reload value to give 1ms delay between timer triggers
		2. Clear current value.
		3. Program Control and Status register
	 */
	uint32_t count_value = SYSTICK_TIM_CLK/tick_hz - 1;   //== 16000. This equates to 1ms
	// reloadValue = clockFrequency / DesiredTimerFrequency
	// reloadValue = 16MHz / (1 / 0.001s)
	// reloadValue = 16K = 16000

	uint32_t volatile * const pSTK_LOAD = (uint32_t*)0xE000E014;
	*pSTK_LOAD &= ~(0x00FFFFFF);  //clear value
	*pSTK_LOAD |= count_value;  //reload value

	uint32_t volatile * const pSTK_VAL = (uint32_t*)0xE000E018;
	*pSTK_VAL = 0;  //clear current value

	uint32_t volatile * const pSTK_CTRL = (uint32_t*)0xE000E010;
	*pSTK_CTRL |= (1 << 2); //set clock source to processor clock (HSI)
	*pSTK_CTRL |= (1 << 1); //enable SYSTICK exception request
	*pSTK_CTRL |= (1 << 0); //counter enable

}

void SysTick_Handler(void)
{

	//TODO. Investigate systick handler
	update_global_tick_count();
	unblock_tasks();


	uint32_t volatile * const pICSR = (uint32_t*)0xE000ED04;
	*pICSR |= (1 << 28); //Pend the PendSV exception
}

//initialize the MSP to the start of scheduler stack address
__attribute__ ((naked)) void init_scheduler_stack(uint32_t stackStart)
{
	__asm volatile ("MSR MSP, %0":: "r"(stackStart));
	__asm volatile ("BX LR");
}

void init_tasks_stack(void)
{
	uint32_t* pStackPointer;



	for(int i = 0; i < MAX_TASKS; i++) //loop each task stack
	{
		userTasks[i].currentState = TASK_READY_STATE;
		userTasks[i].pspValue = TaskStackPointers[i];
		userTasks[i].taskHandler = (void*)TaskHandlers[i];

		//initialize xPSR, PC, LR
		pStackPointer = (uint32_t*)userTasks[i].pspValue;

		pStackPointer--; //move to xPSR location in stack
		*pStackPointer = (0x1 << 24); //set T-bit to 1

		pStackPointer--; //move to PC location in stack
		*pStackPointer = (uint32_t)userTasks[i].taskHandler; //set PC to task

		pStackPointer--; //move to LR location in stack
		*pStackPointer = 0xFFFFFFFD; //return to thread mode using PSP

		//initialize the remaining 13 registers to 0
		for(int j = 0; j < 13; j++)
		{
			pStackPointer--;
			*pStackPointer = 0x0;
		}


		userTasks[i].pspValue = (uint32_t)pStackPointer; //write current stack position to the stackPointer array
	}
}

uint32_t get_psp_value(void)
{
	return userTasks[currentTask].pspValue;
}

void save_psp_value(uint32_t pspValue)
{
	userTasks[currentTask].pspValue = pspValue;
}

void update_next_task (void)
{
	//Go through all tasks and look for a READY task. Else assert idle task.

	uint8_t iterCount = 0;

	do
	{
		iterCount++;
		currentTask++;
		currentTask %= MAX_TASKS;

		if (currentTask == 0) //the idle task
			currentTask++; //skip idle task

		if (iterCount == MAX_TASKS) //we have failed to find a READY task
		{
			currentTask = 0; //assert the idle task
			break; //exit while loop
		}

	} while(userTasks[currentTask].currentState == TASK_BLOCKED_STATE);  //end loop when we have found a READY task

}

void task_delay(uint32_t tickCount)  //A sysTick/Tick happens every 1ms
{
	INTERRUPT_DISABLE(); //MACRO to prevent race conditions

	if(currentTask) //only block task if it's NOT idle task
	{
		userTasks[currentTask].blockCount = g_tick_count + tickCount;
		userTasks[currentTask].currentState = TASK_BLOCKED_STATE;
		schedule(); //allow other tasks to run now
	}

	INTERRUPT_ENABLE();
}

void schedule(void)  //trigger PendSV exception
{
	uint32_t volatile * const pICSR = (uint32_t*)0xE000ED04;
	*pICSR |= (1 << 28); //Pend the PendSV exception
}


__attribute__((naked)) void switch_sp_to_psp(void)
{
	//1. initialize the PSP with TASK1 stack start
	__asm volatile("PUSH {LR}");  //preserve LR coming from main
	__asm volatile("BL get_psp_value");
	__asm volatile("MSR PSP, r0"); //copy Task1 stack pointer into PSP
	__asm volatile("POP {LR}");

	//2. Change SP to PSP using CONTROL register
	__asm volatile("MOV r0, #0x02");
	__asm volatile("MSR CONTROL, r0");
	__asm volatile("BX LR");
}

void enable_processor_faults(void)
{
	uint32_t volatile * const pSHCSR = (uint32_t*)(SCB_ADDRESS_BASE + 0x24);  //register to enable USG, BUS, MEM , SVC faults
	uint32_t volatile * const pCCR = (uint32_t*)(SCB_ADDRESS_BASE + 0x14);  //register to enable DIV_0_TRP
	*pSHCSR |= (1 << 16); //enable mem manage
	*pSHCSR |= (1 << 17); //enable bus fault
	*pSHCSR |= (1 << 18); //enable usage fault

	*pCCR |= (1 << 4); //enable divide by zero trap
}

__attribute__ ((naked)) void UsageFault_Handler(void)
{
	__asm ("MRS r0, MSP");
	__asm ("B UsageFault_Handler_c");
	//The processor will automatically send R0 as parameter 0 to the called function
}

void UsageFault_Handler_c(uint32_t* pBaseStackFrame) //R0 gets copied into pBaseStackFrame
//used to display the stack frame
{
	uint32_t volatile * const pUFSR = (uint32_t*)0xE000ED2A;

	printf("inside UsageFault handler\n");
	printf("UFSR is:%lx\n",(*pUFSR & 0xFFFF));
	printf("pBaseStackFrame = %p\n", pBaseStackFrame);
	printf("R0: %lx\n", pBaseStackFrame[0]);
	printf("R1: %lx\n", pBaseStackFrame[1]);
	printf("R2: %lx\n", pBaseStackFrame[2]);
	printf("R3: %lx\n", pBaseStackFrame[3]);
	printf("R12: %lx\n", pBaseStackFrame[4]);
	printf("LR: %lx\n", pBaseStackFrame[5]);
	printf("PC: %lx\n", pBaseStackFrame[6]);
	printf("xPSR: %lx\n", pBaseStackFrame[7]);
	while(1);
}

void MemManage_Handler(void)
{
	printf("inside MemManage handler\n");
}

void HardFault_Handler(void)
{
	printf("inside HardFault handler\n");
}


