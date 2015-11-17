
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


static void dump_pmc_regs();

#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

//--------------------------
// GLOBAL VARIABLES
//--------------------------
/// Global timestamp in milliseconds since start of application.
volatile unsigned long timestamp = 0;


//----------------------------------------------------------
//SYSTICK --> INTERRUPT call every 1ms
//----------------------------------------------------------
void SysTick_Handler(void)
{
	timestamp++;
}

unsigned long oldtimestamp=1;
void do_periodic(void)
{
	if (timestamp == oldtimestamp)
		return;

	oldtimestamp = timestamp;

    if (timestamp % 10 == 0)
        adc_sample();

    if (timestamp % 250 == 0) //every 100 ms
    {
        manage_heaters();
    }

	if (timestamp % 500 == 0)
	{
		sdcard_handle_state();
	}

    //temp control goes in here
    //temp0 = chan 5 = adc_read(5) etc (returns unsigned absolute millivolt value).
    //temp1 = chan 3
    //temp2 = chan 1
    //temp3 = chan 2

    //if(timestamp%1000==0)//every 1 second
    //{
    //  for(i=1;i<9;i++)
    //      printf("Channel %u : %u mV\n", i,adc_read(i));
    //

}

extern void * current_block;
int main()
{
    TRACE_CONFIGURE(DBGU_STANDARD, 1000000u /*115200*/, BOARD_MCK);
    printf("-- %s\r\n", BOARD_NAME);
    printf("-- Compiled: %s %s --\r\n", __DATE__, __TIME__);
    dump_pmc_regs();

    // If they are present, configure Vbus & Wake-up pins
    //PIO_InitializeInterrupts(0);

    //-------- Start SYSTICK (1ms) --------------
    printf("Configuring systick.\r\n");
    SysTick_Configure(1, BOARD_MCK/1000, SysTick_Handler);

    //-------- Init UART --------------
    printf("USB Seriel INIT\r\n");
    samserial_init();

    //-------- Init parameters --------------
	printf("INIT Parameters\r\n");
	init_parameters();

	//-------- Load parameters from Flash --------------
	printf("Load parameters from Flash\r\n");
	FLASH_LoadSettings();

	//-------- Init ADC without Autostart --------------
	printf("Init ADC\r\n");
    initadc(0);

	//-------- Init Motor driver --------------
	printf("Init Motors\r\n");
    motor_setup();

	//-------- Init Heater I/O  --------------
	printf("Init Heaters\r\n");
    heaters_setup();

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
	dump_pmc_regs();
	printf("Main loop\r\n");
	while (1)
	{
		do_periodic();
		gcode_update();
    }
}

void dump_pmc_regs()
{
#define P(reg) printf("PMC " #reg " 0x04%X: 0x%04X\r\n", offsetof(AT91S_PMC, PMC_##reg), AT91C_BASE_PMC->PMC_##reg)
    P(SCSR);
    P(PCSR);
    P(UCKR);
    P(MOR);
    P(MCFR);
    P(PLLAR);
    P(MCKR);
    P(PCKR[0]); P(PCKR[1]); P(PCKR[2]); P(PCKR[3]);
    P(PCKR[4]); P(PCKR[5]); P(PCKR[6]); P(PCKR[7]);
    P(SR); P(IMR);
    P(FSMR);
    P(FSPR);
#undef P
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

#define STR(x) #x
#define EXCEPTION_HANDLER(name) \
__attribute__((used)) static const char name ## _str[] = STR(name);\
__attribute__((naked)) void name(void) {     \
    __asm volatile (                         \
        " tst lr, #4                     \n" \
        " ite eq                         \n" \
        " mrseq r0, msp                  \n" \
        " mrsne r0, psp                  \n" \
        " ldr r1, "STR(name)"_string     \n" \
        " ldr r2, "STR(name)"_handler    \n" \
        " bx r2                          \n" \
        STR(name)"_handler: .word fault_handler\n" \
        STR(name)"_string: .word "STR(name)"_str\n" \
    ); }


extern uint _estack;

void fault_handler(const uint32_t *fault_stack, const char * fname)
{
    uint r0 = fault_stack[ 0 ];
    uint r1 = fault_stack[ 1 ];
    uint r2 = fault_stack[ 2 ];
    uint r3 = fault_stack[ 3 ];
    uint r12 = fault_stack[ 4 ];
    uint lr = fault_stack[ 5 ];
    uint pc = fault_stack[ 6 ];
    uint psr = fault_stack[ 7 ];

    printf("%s()\r\n", fname);
    printf("pc:0x%X lr:0x%X psr:0x%X r12:0x%X\r\n", pc, lr, psr, r12);
    printf("r0:0x%X r1:0x%X r2:0x%X r3:0x%X\r\n", r0, r1, r2, r3);

    printf("HFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_HFSR);
    printf("CFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_CFSR);
    printf("MMAR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_MMAR);
    printf("BFAR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_BFAR);
    printf("AFSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_AFSR);
    printf("ICSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_ICSR);
    printf("SHCSR: 0x%X\r\n", AT91C_BASE_NVIC->NVIC_HANDCSR);

    unsigned char i;
    uintptr_t ia = (uintptr_t)&i;
    ia += 3;
    ia &= ~(uintptr_t)3;

    printf("Stack dump:\r\n");
    printf("&i:0x%X, stack top:0x%X\r\n", ia, (unsigned)&_estack);
    TRACE_DumpMemory((unsigned char *)ia, (unsigned char*)&_estack - &i, ia);

    while(1);
}



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
        printf("backtrace error: __builtin_return_address(0):0x%X != frame_p->lr:0x%X\r\n",
            (uintptr_t)__builtin_return_address(0), (uintptr_t)frame_p->lr);
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


#if 1
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
EXCEPTION_HANDLER(HardFault_Handler)
EXCEPTION_HANDLER(IrqHandlerNotUsed)
EXCEPTION_HANDLER(NMI_Handler)
EXCEPTION_HANDLER(MemManage_Handler)
EXCEPTION_HANDLER(BusFault_Handler)
EXCEPTION_HANDLER(UsageFault_Handler)

EXCEPTION_HANDLER(SVC_Handler)
EXCEPTION_HANDLER(DebugMon_Handler)
EXCEPTION_HANDLER(PendSV_Handler)
EXCEPTION_HANDLER(SYS_IrqHandler)
EXCEPTION_HANDLER(SUPC_IrqHandler)
EXCEPTION_HANDLER(RSTC_IrqHandler)
EXCEPTION_HANDLER(RTC_IrqHandler)
EXCEPTION_HANDLER(RTT_IrqHandler)
EXCEPTION_HANDLER(WDT_IrqHandler)
EXCEPTION_HANDLER(PMC_IrqHandler)
EXCEPTION_HANDLER(EFC0_IrqHandler)
EXCEPTION_HANDLER(EFC1_IrqHandler)
EXCEPTION_HANDLER(DBGU_IrqHandler)
EXCEPTION_HANDLER(HSMC4_IrqHandler)
//EXCEPTION_HANDLER(PIOA_IrqHandler)
//EXCEPTION_HANDLER(PIOB_IrqHandler)
//EXCEPTION_HANDLER(PIOC_IrqHandler)
EXCEPTION_HANDLER(USART0_IrqHandler)
EXCEPTION_HANDLER(USART1_IrqHandler)
EXCEPTION_HANDLER(USART2_IrqHandler)
EXCEPTION_HANDLER(USART3_IrqHandler)
//EXCEPTION_HANDLER(MCI0_IrqHandler)
EXCEPTION_HANDLER(TWI0_IrqHandler)
EXCEPTION_HANDLER(TWI1_IrqHandler)
EXCEPTION_HANDLER(SPI0_IrqHandler)
EXCEPTION_HANDLER(SSC0_IrqHandler)
//EXCEPTION_HANDLER(TC0_IrqHandler)
//EXCEPTION_HANDLER(TC1_IrqHandler)
EXCEPTION_HANDLER(TC2_IrqHandler)
EXCEPTION_HANDLER(PWM_IrqHandler)
//EXCEPTION_HANDLER(ADCC0_IrqHandler)
EXCEPTION_HANDLER(ADCC1_IrqHandler)
EXCEPTION_HANDLER(HDMA_IrqHandler)
//EXCEPTION_HANDLER(UDPD_IrqHandler)

EXCEPTION_HANDLER(RESERVED0_IrqHandler)
EXCEPTION_HANDLER(RESERVED1_IrqHandler)
EXCEPTION_HANDLER(RESERVED2_IrqHandler)
EXCEPTION_HANDLER(RESERVED3_IrqHandler)
EXCEPTION_HANDLER(RESERVED4_IrqHandler)

#endif
