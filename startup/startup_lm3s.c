/**
 * LM3S Devices Startup code.
 *  This module performs:
 *    - Set the initial SP
 *    - Set the vector table entries with the exceptions ISR address
 *    - Initialize data and bss
 *    - Setup the microcontroller system. 			
 *    - Call the application's entry point.
 *  After Reset the Cortex-M3 processor is in Thread mode, 
 *  priority is Privileged, and the Stack is set to Main.
 */
#define STACK_SIZE       0x00000100
__attribute__ ((section(".co_stack")))
unsigned long pulStack[STACK_SIZE];      

#define WEAK __attribute__ ((weak))           

__attribute__ ((used))
void WEAK  Reset_Handler(void);
void WEAK  NMI_Handler(void);
void WEAK  HardFault_Handler(void);
void WEAK  MemManage_Handler(void);
void WEAK  BusFault_Handler(void);
void WEAK  UsageFault_Handler(void);
void WEAK  SVC_Handler(void);
void WEAK  DebugMon_Handler(void);
void WEAK  PendSV_Handler(void);
void WEAK  SysTick_Handler(void);
void WEAK  GPIOPortA_IRQHandler(void);
void WEAK  GPIOPortB_IRQHandler(void);
void WEAK  GPIOPortC_IRQHandler(void);
void WEAK  GPIOPortD_IRQHandler(void);
void WEAK  GPIOPortE_IRQHandler(void);
void WEAK  UART0_IRQHandler(void);
void WEAK  UART1_IRQHandler(void);
void WEAK  SSI0_IRQHandler(void);
void WEAK  I2C0_IRQHandler(void);
void WEAK  PWMFault_IRQHandler(void);
void WEAK  PWMGen0_IRQHandler(void);
void WEAK  PWMGen1_IRQHandler(void);
void WEAK  PWMGen2_IRQHandler(void);
void WEAK  QEI0_IRQHandler(void);
void WEAK  ADCSeq0_IRQHandler(void);
void WEAK  ADCSeq1_IRQHandler(void);
void WEAK  ADCSeq2_IRQHandler(void);
void WEAK  ADCSeq3_IRQHandler(void);
void WEAK  Watchdog_IRQHandler(void);
void WEAK  Timer0A_IRQHandler(void);
void WEAK  Timer0B_IRQHandler(void);
void WEAK  Timer1A_IRQHandler(void);
void WEAK  Timer1B_IRQHandler(void);
void WEAK  Timer2A_IRQHandler(void);
void WEAK  Timer2B_IRQHandler(void);
void WEAK  Comp0_IRQHandler(void);
void WEAK  Comp1_IRQHandler(void);
void WEAK  Comp2_IRQHandler(void);
void WEAK  SysCtrl_IRQHandler(void);
void WEAK  FlashCtrl_IRQHandler(void);
void WEAK  GPIOPortF_IRQHandler(void);
void WEAK  GPIOPortG_IRQHandler(void);
void WEAK  GPIOPortH_IRQHandler(void);
void WEAK  UART2_IRQHandler(void);
void WEAK  SSI1_IRQHandler(void);
void WEAK  Timer3A_IRQHandler(void);
void WEAK  Timer3B_IRQHandler(void);
void WEAK  I2C1_IRQHandler(void);
void WEAK  QEI1_IRQHandler(void);
void WEAK  CAN0_IRQHandler(void);
void WEAK  CAN1_IRQHandler(void);
void WEAK  CAN2_IRQHandler(void);
void WEAK  Ethernet_IRQHandler(void);
void WEAK  Hibernate_IRQHandler(void);
void WEAK  USB0_IRQHandler(void);
void WEAK  PWMGen3_IRQHandler(void);
void WEAK  uDMA_IRQHandler(void);
void WEAK  uDMAErr_IRQHandler(void);

extern unsigned long _sidata;   
extern unsigned long _sdata; 
extern unsigned long _edata;
extern unsigned long _sbss;
extern unsigned long _ebss;
extern void _eram;

extern int main(void);           
extern void SystemInit(void);   
void Default_Reset_Handler(void);
static void Default_Handler(void);


__attribute__ ((used,section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
  (void *)&pulStack[STACK_SIZE],       /* The initial stack pointer         */
  Reset_Handler,                       /* The reset handler                 */
  NMI_Handler,                         /* The NMI handler                   */ 
  HardFault_Handler,                   /* The hard fault handler            */
  MemManage_Handler,                   /* The MPU fault handler             */
  BusFault_Handler,                    /* The bus fault handler             */
  UsageFault_Handler,                  /* The usage fault handler           */ 
  0,0,0,0,                             /* Reserved                          */
  SVC_Handler,                         /* SVCall handler                    */
  DebugMon_Handler,                    /* Debug monitor handler             */
  0,                                   /* Reserved                          */
  PendSV_Handler,                      /* The PendSV handler                */
  SysTick_Handler,                     /* The SysTick handler               */ 
 
  GPIOPortA_IRQHandler,                /*  0: GPIO Port A                   */
  GPIOPortB_IRQHandler,                /*  1: GPIO Port B                   */
  GPIOPortC_IRQHandler,                /*  2: GPIO Port C                   */
  GPIOPortD_IRQHandler,                /*  3: GPIO Port D                   */
  GPIOPortE_IRQHandler,                /*  4: GPIO Port E                   */ 
  UART0_IRQHandler,                    /*  5: UART0 Rx and Tx               */
  UART1_IRQHandler,                    /*  6: UART1 Rx and Tx               */
  SSI0_IRQHandler,                     /*  7: SSI0 Rx and Tx                */
  I2C0_IRQHandler,                     /*  8: I2C0 Master and Slave         */
  PWMFault_IRQHandler,                 /*  9: PWM Fault                     */
  PWMGen0_IRQHandler,                  /* 10: PWM Generator 0               */
  PWMGen1_IRQHandler,                  /* 11: PWM Generator 1               */
  PWMGen2_IRQHandler,                  /* 12: PWM Generator 2               */
  QEI0_IRQHandler,                     /* 13: Quadrature Encoder 0          */ 
  ADCSeq0_IRQHandler,                  /* 14: ADC Sequence 0                */
  ADCSeq1_IRQHandler,                  /* 15: ADC Sequence 1                */
  ADCSeq2_IRQHandler,                  /* 16: ADC Sequence 2                */ 
  ADCSeq3_IRQHandler,                  /* 17: ADC Sequence 3                */ 
  Watchdog_IRQHandler,                 /* 18: Watchdog timer                */
  Timer0A_IRQHandler,                  /* 19: Timer 0 subtimer A            */
  Timer0B_IRQHandler,                  /* 20: Timer 0 subtimer B            */
  Timer1A_IRQHandler,                  /* 21: Timer 1 subtimer A            */
  Timer1B_IRQHandler,                  /* 22: Timer 1 subtimer B            */
  Timer2A_IRQHandler,                  /* 23: Timer 2 subtimer A            */ 
  Timer2B_IRQHandler,                  /* 24: Timer 2 subtimer B            */
  Comp0_IRQHandler,                    /* 25: Analog Comparator 0           */
  Comp1_IRQHandler,                    /* 26: Analog Comparator 1           */
  Comp2_IRQHandler,                    /* 27: Analog Comparator 2           */
  SysCtrl_IRQHandler,                  /* 28: System Control (PLL, OSC, BO) */ 
  FlashCtrl_IRQHandler,                /* 29: FLASH Control                 */
  GPIOPortF_IRQHandler,                /* 30: GPIO Port F                   */
  GPIOPortG_IRQHandler,                /* 31: GPIO Port G                   */
  GPIOPortH_IRQHandler,                /* 32: GPIO Port H                   */
  UART2_IRQHandler,                    /* 33: UART2 Rx and Tx               */
  SSI1_IRQHandler,                     /* 34: SSI1 Rx and Tx                */
  Timer3A_IRQHandler,                  /* 35: Timer 3 subtimer A            */
  Timer3B_IRQHandler,                  /* 36: Timer 3 subtimer B            */ 
  I2C1_IRQHandler,                     /* 37: I2C1 Master and Slave         */
  QEI1_IRQHandler,                     /* 38: Quadrature Encoder 1          */  
  CAN0_IRQHandler,                     /* 39: CAN0                          */
  CAN1_IRQHandler,                     /* 40: CAN1                          */ 
  CAN2_IRQHandler,                     /* 41: CAN2                          */
  Ethernet_IRQHandler,                 /* 42: Ethernet                      */
  Hibernate_IRQHandler,                /* 43: Hibernate                     */
  USB0_IRQHandler,                     /* 44: USB0                          */
  PWMGen3_IRQHandler,                  /* 45: PWM Generator 3               */
  uDMA_IRQHandler,                     /* 46: uDMA Software Transfer        */
  uDMAErr_IRQHandler,                  /* 47: uDMA Error                    */
};


/**
  * @brief  This is the code that gets called when the processor first
  *         starts execution following a reset event. Only the absolutely
  *         necessary set is performed, after which the application
  *         supplied main() routine is called. 
  * @param  None
  * @retval None
  */
void Default_Reset_Handler(void)
{
  unsigned long *pulSrc, *pulDest;
  pulSrc = &_sidata;

  for(pulDest = &_sdata; pulDest < &_edata; )
  {
    *(pulDest++) = *(pulSrc++);
  }
  
  __asm("  ldr     r0, =_sbss\n"
        "  ldr     r1, =_ebss\n"
        "  mov     r2, #0\n"
        "  .thumb_func\n"
        "zero_loop:\n"
        "    cmp     r0, r1\n"
        "    it      lt\n"
        "    strlt   r2, [r0], #4\n"
        "    blt     zero_loop");

  main();
}

#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler
#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
#pragma weak GPIOPortA_IRQHandler = Default_Handler
#pragma weak GPIOPortB_IRQHandler = Default_Handler
#pragma weak GPIOPortC_IRQHandler = Default_Handler
#pragma weak GPIOPortD_IRQHandler = Default_Handler
#pragma weak GPIOPortE_IRQHandler = Default_Handler
#pragma weak UART0_IRQHandler = Default_Handler
#pragma weak UART1_IRQHandler = Default_Handler
#pragma weak SSI0_IRQHandler = Default_Handler
#pragma weak I2C0_IRQHandler = Default_Handler
#pragma weak PWMFault_IRQHandler = Default_Handler
#pragma weak PWMGen0_IRQHandler = Default_Handler
#pragma weak PWMGen1_IRQHandler = Default_Handler
#pragma weak PWMGen2_IRQHandler = Default_Handler
#pragma weak QEI0_IRQHandler = Default_Handler
#pragma weak ADCSeq0_IRQHandler = Default_Handler
#pragma weak ADCSeq1_IRQHandler = Default_Handler
#pragma weak ADCSeq2_IRQHandler = Default_Handler
#pragma weak ADCSeq3_IRQHandler = Default_Handler
#pragma weak Watchdog_IRQHandler = Default_Handler
#pragma weak Timer0A_IRQHandler = Default_Handler
#pragma weak Timer0B_IRQHandler = Default_Handler
#pragma weak Timer1A_IRQHandler = Default_Handler
#pragma weak Timer1B_IRQHandler = Default_Handler
#pragma weak Timer2A_IRQHandler = Default_Handler
#pragma weak Timer2B_IRQHandler = Default_Handler
#pragma weak Comp0_IRQHandler = Default_Handler
#pragma weak Comp1_IRQHandler = Default_Handler
#pragma weak Comp2_IRQHandler = Default_Handler
#pragma weak SysCtrl_IRQHandler = Default_Handler
#pragma weak FlashCtrl_IRQHandler = Default_Handler
#pragma weak GPIOPortF_IRQHandler = Default_Handler
#pragma weak GPIOPortG_IRQHandler = Default_Handler
#pragma weak GPIOPortH_IRQHandler = Default_Handler
#pragma weak UART2_IRQHandler = Default_Handler
#pragma weak SSI1_IRQHandler = Default_Handler
#pragma weak Timer3A_IRQHandler = Default_Handler
#pragma weak Timer3B_IRQHandler = Default_Handler
#pragma weak I2C1_IRQHandler = Default_Handler
#pragma weak QEI1_IRQHandler = Default_Handler
#pragma weak CAN0_IRQHandler = Default_Handler
#pragma weak CAN1_IRQHandler = Default_Handler
#pragma weak CAN2_IRQHandler = Default_Handler
#pragma weak Ethernet_IRQHandler = Default_Handler
#pragma weak Hibernate_IRQHandler = Default_Handler
#pragma weak USB0_IRQHandler = Default_Handler
#pragma weak PWMGen3_IRQHandler = Default_Handler
#pragma weak uDMA_IRQHandler = Default_Handler
#pragma weak uDMAErr_IRQHandler = Default_Handler

static void Default_Handler(void) 
{
  while (1) { }
}
