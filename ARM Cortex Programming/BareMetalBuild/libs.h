/*
 * libs.h
 *
 *  Created on: Oct 10, 2022
 *      Author: Jason
 */

#ifndef LIBS_H_
#define LIBS_H_

//function declarations
void task1_handler(void);
void task2_handler(void);
void task3_handler(void);
void task4_handler(void);
void SysTick_Handler(void);
void init_systick_timer(uint32_t tick_hz);
__attribute__ ((naked)) void init_scheduler_stack(uint32_t stackStart);
void init_tasks_stack(void);
uint32_t get_psp_value(void);
void save_psp_value(uint32_t pspValue);
__attribute__((naked)) void switch_sp_to_psp(void);
void enable_processor_faults(void);

__attribute__ ((naked)) void UsageFault_Handler(void);
void UsageFault_Handler_c(uint32_t* pBaseStackFrame);
void MemManage_Handler(void);
void HardFault_Handler(void);
void task_delay(uint32_t tickCount);
void idle_task(void);

void update_global_tick_count(void);
void unblock_tasks(void);
void schedule(void);




// stack memory definitions
#define SIZE_TASK_STACK		1024U
#define SIZE_SCHED_STACK	1024U

#define SRAM_START			0x20000000
#define SRAM_SIZE			((128) * (1024))
#define SRAM_END			((SRAM_START) + (SRAM_SIZE))


#define IDLE_STACK_START	SRAM_END
#define T1_STACK_START		(SRAM_END) - (1 * SIZE_TASK_STACK)
#define T2_STACK_START		(SRAM_END) - (2 * SIZE_TASK_STACK)
#define T3_STACK_START		(SRAM_END) - (3 * SIZE_TASK_STACK)
#define T4_STACK_START		(SRAM_END) - (4 * SIZE_TASK_STACK)

#define SCHED_STACK_START	(SRAM_END) - (5 * SIZE_TASK_STACK)

//systick timer
#define TICK_HZ 			1000U	   //desired Tick frequency (or the frequency at which the systick exception triggers
#define HSI_CLOCK 			16000000U  //processor clock speed in Hz
#define SYSTICK_TIM_CLK		HSI_CLOCK

// faults
#define SCB_ADDRESS_BASE 0xE000ED00

//tasks
#define MAX_TASKS			5
#define TASK_READY_STATE	0x00
#define TASK_BLOCKED_STATE	0xFF


//global variable declarations
extern uint32_t TaskStackPointers[MAX_TASKS];
extern uint32_t TaskHandlers[MAX_TASKS];
extern uint8_t currentTask;
extern uint32_t g_tick_count;


//struct for tasks (Task control Block)
typedef struct
{
	uint32_t pspValue;
	uint32_t blockCount;
	uint8_t currentState;
	void (*taskHandler) (void);
} TCB_t;

extern TCB_t userTasks[MAX_TASKS];

#define INTERRUPT_DISABLE() do{__asm volatile ("MOV R0, #0x1"); __asm volatile("MSR PRIMASK,R0"); } while(0)
#define INTERRUPT_ENABLE()  do{__asm volatile ("MOV R0, #0x0"); __asm volatile("MSR PRIMASK,R0"); } while(0)

#endif /* LIBS_H_ */
