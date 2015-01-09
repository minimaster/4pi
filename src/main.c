
#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <irq/irq.h>
#include <tc/tc.h>
#include <systick/systick.h>
#include <utility/trace.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parameters.h"
#include "serial.h"
#include "samadc.h"
#include "stepper_control.h"
#include "planner.h"
#include "gcode_parser.h"
#include "sdcard.h"
//#include "heaters.h"


//--------------------------
// EXTERN FUNCTIONS
//--------------------------
extern void adc_sample();
extern void samserial_init();

extern void motor_setup();
extern void motor_setopts(unsigned char axis, unsigned char ustepbits, unsigned char current);
extern void motor_enaxis(unsigned char axis, unsigned char en);
extern void motor_setdir(unsigned char axis, unsigned char dir);
extern void motor_step(unsigned char axis);
extern void motor_unstep();

extern void heaters_setup();
extern void manage_heaters(void);
//extern void heater_soft_pwm(void);
extern void ConfigureTc_1(void);


//extern void sprinter_mainloop();
extern void initadc(int);
extern void samserial_setcallback(void (*c)(unsigned char));


#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

//--------------------------
// GLOBAL VARIABLES
//--------------------------
/// Global timestamp in milliseconds since start of application.
volatile unsigned long timestamp = 1;

  
//----------------------------------------------------------
//SYSTICK --> INTERRUPT call every 1ms 
//----------------------------------------------------------
void SysTick_Handler(void)
{
	
	timestamp++;
	
    if(timestamp%10==0)
        adc_sample();
    
    
    //temp control goes in here
    //temp0 = chan 5 = adc_read(5) etc (returns unsigned absolute millivolt value).
    //temp1 = chan 3
    //temp2 = chan 1
    //temp3 = chan 2
    
    //if(timestamp%1000==0)//every 1 second
	//{
	//  for(i=1;i<9;i++)
    //  	printf("Channel %u : %u mV\n", i,adc_read(i));
    //	
	
	if(timestamp%250==0) //every 100 ms
    {
		manage_heaters();
    }
	    
}
unsigned long oldtimestamp=1;
void do_periodic()
{
	if (timestamp==oldtimestamp)
		return;
	oldtimestamp=timestamp;
	if (timestamp % 500 == 0)
	{
		sdcard_handle_state();
	}
	
}


int main()
{
	
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- %s\r\n", BOARD_NAME);
    printf("-- Compiled: %s %s --\r\n", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    //PIO_InitializeInterrupts(0);
	
	//-------- Init parameters --------------
	printf("INIT Parameters\r\n");
	init_parameters();
	
	//-------- Load parameters from Flash --------------
	printf("Load parameters from Flash\r\n");
	FLASH_LoadSettings();
	
    //-------- Init UART --------------
	printf("USB Seriel INIT\r\n");
	samserial_init();
	
	//-------- Init ADC without Autostart --------------
	printf("Init ADC\r\n");
    initadc(0);
	
	//-------- Init Motor driver --------------
	printf("Init Motors\r\n");
    motor_setup();
	
	//-------- Init Heater I/O  --------------
	printf("Init Heaters\r\n");
    heaters_setup();
	
    //-------- Start SYSTICK (1ms) --------------
	printf("Configuring systick.\r\n");
	SysTick_Configure(1, BOARD_MCK/1000, SysTick_Handler);
	
	//-------- Timer 0 for Stepper --------------
	printf("Init Stepper IO\r\n");
    stepper_setup();	//Timer for Stepper


	//-------- Timer 0 for Stepper --------------
	printf("Configuring Timer 0 Stepper\r\n");
    ConfigureTc0_Stepper();	//Timer for Stepper
	
	//-------- Timer 1 for heater PWM --------------
	printf("Configuring Timer 1 PWM.\r\n");
	ConfigureTc_1();

	//-------- Init Planner Values --------------
	printf("Plan Init\r\n");
	plan_init();
	
	printf("G-Code parser init\r\n");
	gcode_init(usb_printf);
	
	//-------- Check for SD card presence -------
//	sdcard_handle_state();
	
	//motor_enaxis(0,1);
    //motor_enaxis(1,1);
	printf("Main loop\r\n");
	while (1)
	{
  		//uncomment to use//sprinter_mainloop();
    	//main loop events go here

		do_periodic();

		gcode_update();
/*    	
		if(buflen < (BUFSIZE-1))
			get_command();

    	if(buflen > 0)
		{
			
			//-------- Check and execute G-CODE --------------
			process_commands();

			//-------- Increment G-Code FIFO  --------------
			buflen = (buflen-1);
			bufindr++;
			if(bufindr == BUFSIZE) bufindr = 0;
			
		}
*/		  
    }
}

/**
 *
 *
 *
 *
 */

#include "utility/trace.h"

#undef WEAK
#define WEAK
#define FAIL() printf("%s()\r\n", __func__)


extern unsigned int _estack;

struct backtrace_frame_t
{
    void * fp;
    void * sp;
    void * lr;
    void * pc;
};

int backtrace(void ** array, int size)
{
    void * top_frame_p;
    void * current_frame_p;
    struct backtrace_frame_t * frame_p;
    int frame_count;

    top_frame_p = __builtin_frame_address(0);
    current_frame_p = top_frame_p;
    frame_p = (struct backtrace_frame_t*)((void**)(current_frame_p)-3);
    frame_count = 0;

    if (__builtin_return_address(0) != frame_p->lr)
    {
        printf("backtrace error: __builtin_return_address(0) != frame_p->lr\r\n");
        return frame_count;
    }

    if (current_frame_p != NULL
        && current_frame_p > (void*)&frame_count
        && current_frame_p < (void*)&_estack)
    {
        while (frame_count < size
               && current_frame_p != NULL
               && current_frame_p > (void*)&frame_count
               && current_frame_p < (void*)&_estack)
        {
            frame_p = (struct backtrace_frame_t*)((void**)(current_frame_p)-3);
            array[frame_count] = frame_p->lr;
            frame_count++;
            current_frame_p = frame_p->fp;
        }
    }

    return frame_count;
}

//------------------------------------------------------------------------------
// Default irq handler
//------------------------------------------------------------------------------
void IrqHandlerNotUsed(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Provide weak aliases for each Exception handler to the IrqHandlerNotUsed.
// As they are weak aliases, any function with the same name will override
// this definition.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// System interrupt
//------------------------------------------------------------------------------
WEAK void NMI_Handler(void)
{
	FAIL();
    while(1);
}
#if 1
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void HardFault_Handler(void)
{
	FAIL();

	printf("HFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_HFSR);
	printf("CFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_CFSR);
	printf("MMAR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_MMAR);
	printf("BFAR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_BFAR);
	printf("AFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_AFSR);

	static void *bt[20];
	int c = backtrace(bt, 20);
	int i;

	if (0 == c)
	{
        printf("Stack dump:\r\n");
        printf("&i:0x%X, &c:0x%X\r\n", (unsigned)&i, (unsigned)&c);
        TRACE_DumpMemory((unsigned char*)&i, (char*)&_estack - (char*)&i, (unsigned)&_estack);
	}
	else
	{
	    printf("Stack dump: %d frames\r\n", c);
	    for (i = 0; i < c; i++)
	        printf("0x%X\r\n", (unsigned)bt[i]);
	}

	while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void MemManage_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void BusFault_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void UsageFault_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void SVC_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void DebugMon_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
WEAK void PendSV_Handler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// for Cortex M3
//------------------------------------------------------------------------------
//WEAK void SysTick_Handler(void)
//{
//	FAIL();
//   while(1);
//}

//------------------------------------------------------------------------------
// External interrupt
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// for SAM7/9
//------------------------------------------------------------------------------
void SYS_IrqHandler( void )
{
	FAIL();
    while(1);
}


//------------------------------------------------------------------------------
// SUPPLY CONTROLLER
//------------------------------------------------------------------------------
WEAK void SUPC_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// RESET CONTROLLER
//------------------------------------------------------------------------------
WEAK void RSTC_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// REAL TIME CLOCK
//------------------------------------------------------------------------------
WEAK void RTC_IrqHandler(void)
{
    while(1);
}

//------------------------------------------------------------------------------
// REAL TIME TIMER
//------------------------------------------------------------------------------
WEAK void RTT_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// WATCHDOG TIMER
//------------------------------------------------------------------------------
WEAK void WDT_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// PMC
//------------------------------------------------------------------------------
WEAK void PMC_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// EFC0
//------------------------------------------------------------------------------
WEAK void EFC0_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// EFC1
//------------------------------------------------------------------------------
WEAK void EFC1_IrqHandler(void)
{
	FAIL();
    while(1);
}
//------------------------------------------------------------------------------
// DBGU
//------------------------------------------------------------------------------
WEAK void DBGU_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// HSMC4
//------------------------------------------------------------------------------
WEAK void HSMC4_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Parallel IO Controller A
//------------------------------------------------------------------------------
//WEAK void PIOA_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// Parallel IO Controller B
//------------------------------------------------------------------------------
//WEAK void PIOB_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// Parallel IO Controller C
//------------------------------------------------------------------------------
//WEAK void PIOC_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// USART 0
//------------------------------------------------------------------------------
WEAK void USART0_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// USART 1
//------------------------------------------------------------------------------
WEAK void USART1_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// USART 2
//------------------------------------------------------------------------------
WEAK void USART2_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// USART 3
//------------------------------------------------------------------------------
WEAK void USART3_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Multimedia Card Interface
//------------------------------------------------------------------------------
//WEAK void MCI0_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// TWI 0
//------------------------------------------------------------------------------
WEAK void TWI0_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// TWI 1
//------------------------------------------------------------------------------
WEAK void TWI1_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Serial Peripheral Interface 0
//------------------------------------------------------------------------------
WEAK void SPI0_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Serial Synchronous Controller 0
//------------------------------------------------------------------------------
WEAK void SSC0_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// Timer Counter 0
//------------------------------------------------------------------------------
//WEAK void TC0_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// Timer Counter 1
//------------------------------------------------------------------------------
//WEAK void TC1_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// Timer Counter 2
//------------------------------------------------------------------------------
WEAK void TC2_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// PWM Controller
//------------------------------------------------------------------------------
WEAK void PWM_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// ADC controller0
//------------------------------------------------------------------------------
//WEAK void ADCC0_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

//------------------------------------------------------------------------------
// ADC controller1
//------------------------------------------------------------------------------
WEAK void ADCC1_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// HDMA
//------------------------------------------------------------------------------
WEAK void HDMA_IrqHandler(void)
{
	FAIL();
    while(1);
}

//------------------------------------------------------------------------------
// USB Device High Speed UDP_HS
//------------------------------------------------------------------------------
//WEAK void UDPD_IrqHandler(void)
//{
//	FAIL();
//    while(1);
//}

#endif
