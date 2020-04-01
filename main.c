//#include "main.h"
#include "hal_LCD.h"
#include "driverlib.h"

//#include "TempSensorMode.h"

// Backup Memory variables to track states through LPM3.5
volatile unsigned char * S1buttonDebounce = &BAKMEM2_L;       // S1 button debounce flag
volatile unsigned char * S2buttonDebounce = &BAKMEM2_H;       // S2 button debounce flag
//volatile unsigned char * tempSensorRunning = &BAKMEM3_H;      // Temp Sensor running flag
                // mode flag
volatile unsigned int holdCount = 0;
volatile unsigned int temperature = 25;
volatile unsigned int a;

void initTimers(void);
void Init_GPIO(void);

// TimerA0 UpMode Configuration Parameter
Timer_A_initUpModeParam initUpParam_A0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,          // SMCLK/1 = 2MHz
        30000,                                  // 15ms debounce period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable Timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR,                       // Clear value
        true                                    // Start Timer
};

/*
 * main.c
 */
int main(void) {
    // Stop Watchdog timer
    WDT_A_hold(WDT_A_BASE);
        Init_GPIO();
        Init_LCD();

        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);

        *S1buttonDebounce = *S2buttonDebounce = 0;
        __enable_interrupt();

        displayScrollText("WELCOME TO");


   // int i = 0x01;

    while(1)
    {
        //LCD_E_selectDisplayMemory(LCD_E_BASE, LCD_E_DISPLAYSOURCE_MEMORY);
        __bis_SR_register(LPM3_bits | GIE);         // enter LPM3
    }

    __no_operation();
}

/*
 * GPIO Initialization
 */

void Init_GPIO()
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsInputPin(GPIO_PORT_P1, GPIO_PIN1);

    // Configure button S1 (P1.2) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN2, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN2);
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN2);

    // Configure button S2 (P2.6) interrupt
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P8,
                         GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

         GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4,
                             GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set P4.1 and P4.2 as Secondary Module Function Input, LFXT.
    GPIO_setAsPeripheralModuleFunctionInputPin(
           GPIO_PORT_P4,
           GPIO_PIN1 + GPIO_PIN2,
           GPIO_PRIMARY_MODULE_FUNCTION
           );

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PMM_unlockLPM5();
}

/*
 * PORT1 Interrupt Service Routine
 * Handles S1 button press interrupt
 */
#pragma vector = PORT1_VECTOR
__interrupt void PORT1_ISR(void)

{
    P1OUT |= BIT0;    // Turn LED1 On
    switch(__even_in_range(P1IV, P1IV_P1IFG7))
    {
        case P1IV_NONE : break;
        case P1IV_P1IFG0 : break;
        case P1IV_P1IFG1 : break;
        case P1IV_P1IFG2 :
            if ((*S1buttonDebounce) == 0)
            {
                *S1buttonDebounce = 1;                        // First high to low transition
                holdCount = 0;
                ++temperature;
                showChar((temperature % 10) + '0',pos5);
                showChar(((temperature/10)%10) + '0',pos4);
                a = 0x300;
                initTimers();

                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }
        case P1IV_P1IFG3 : break;
        case P1IV_P1IFG4 : break;
        case P1IV_P1IFG5 : break;
        case P1IV_P1IFG6 : break;
        case P1IV_P1IFG7 : break;
    }
}

/*
 * PORT2 Interrupt Service Routine
 * Handles S2 button press interrupt
 */
#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR(void)
{
    P4OUT |= BIT0;    // Turn LED2 On
    switch(__even_in_range(P2IV, P2IV_P2IFG7))
    {
        case P2IV_NONE : break;
        case P2IV_P2IFG0 : break;
        case P2IV_P2IFG1 : break;
        case P2IV_P2IFG2 : break;
        case P2IV_P2IFG3 : break;
        case P2IV_P2IFG4 : break;
        case P2IV_P2IFG5 : break;
        case P2IV_P2IFG6 :
            if ((*S2buttonDebounce) == 0)
            {
                *S2buttonDebounce = 1;                        // First high to low transition
                holdCount = 0;
                --temperature;
                showChar((temperature % 10) + '0',pos5);
                showChar(((temperature/10)%10) + '0',pos4);
                a = 0x1000;
                initTimers();
                // Start debounce timer
                Timer_A_initUpMode(TIMER_A0_BASE, &initUpParam_A0);
            }

        case P2IV_P2IFG7 : break;
    }
}

/*
 * Timer A0 Interrupt Service Routine
 * Used as button debounce timer
 */
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR (void)
{


    // Button S1 released
    if (P1IN & BIT2)
    {
        *S1buttonDebounce = 0;                                   // Clear button debounce
        P1OUT &= ~BIT0;
    }

    // Button S2 released
    if (P2IN & BIT6)
    {
        *S2buttonDebounce = 0;                                   // Clear button debounce
        P4OUT &= ~BIT0;
    }
}

void initTimers(void)
    {
         // Set up the interrupt using CCROIFG to toggle red LED1
         // Set up the interrupt using TA1IFG toggle green LED2
         Timer_A_initUpModeParam initUpParam = { 0 };
         initUpParam.clockSource = TIMER_A_CLOCKSOURCE_ACLK;        // Use ACLK (slower clock)
         initUpParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;  // Input clock = ACLK / 1 = 32KHz
         initUpParam.timerPeriod =0xFFFF/4;            // Half the time
         initUpParam.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;  // Enable TAR -> 0 interrupt
         initUpParam.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE; //Enable compare interrupt
         initUpParam.timerClear = TIMER_A_DO_CLEAR;               // Clear TAR & clock divider
         initUpParam.startTimer = false;                          // Don't start the timer, yet

         Timer_A_initUpMode( TIMER_A1_BASE, &initUpParam );

         Timer_A_initCompareModeParam initCcr2Param = { 0 };
         initCcr2Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;                 // Use CCR2 for compare
         initCcr2Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE; // Since directly driving LED, interrup not req'd
         initCcr2Param.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_RESET;        // Toggle provides a 1 sec period based on CCR0 and CCR2 values
         initCcr2Param.compareValue = a;              // Compare value: 4000 = 1/2 second

         Timer_A_initCompareMode( TIMER_A1_BASE, &initCcr2Param );

         Timer_A_clearTimerInterrupt( TIMER_A1_BASE );    // Clear/enable flags and start timer
         Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0 + TIMER_A_CAPTURECOMPARE_REGISTER_2); //Clear CCROIFG, CCR1IFG

         Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE);

//         Timer_A_initUpModeParam initUpParam1 = { 0 };
//         initUpParam1.clockSource = TIMER_A_CLOCKSOURCE_ACLK;        // Use ACLK (slower clock)
//         initUpParam1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1; // Input clock = ACLK / 1 = 32KHz
//         initUpParam1.timerPeriod = 0xFFFF/4;  // Half the time
//         initUpParam1.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;  // Enable TAR -> 0 interrupt
//         initUpParam1.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE; //Enable compare interrupt
//         initUpParam1.timerClear = TIMER_A_DO_CLEAR;               // Clear TAR & clock divider
//         initUpParam1.startTimer = false;                          // Don't start the timer, yet
//
//         Timer_A_initUpMode( TIMER_A1_BASE, &initUpParam1 );
//
//         Timer_A_initCompareModeParam initCcr1Param = { 0 };
//         initCcr1Param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;                 // Use CCR2 for compare
//         initCcr1Param.compareInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE; // Since directly driving LED, interrupt not req'd
//         initCcr1Param.compareOutputMode = TIMER_A_OUTPUTMODE_TOGGLE_RESET;        // Toggle provides a 1 sec period based on CCR0 and CCR2 values
//         initCcr1Param.compareValue = 0x1000;                // Compare value: 4000 = 1/2 second
//
//         Timer_A_initCompareMode( TIMER_A1_BASE, &initCcr1Param );
//
//         Timer_A_clearTimerInterrupt( TIMER_A1_BASE );    // Clear/enable flags and start timer
//         Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0 + TIMER_A_CAPTURECOMPARE_REGISTER_1); //Clear CCROIFG, CCR1IFG
//
//         Timer_A_startCounter(TIMER_A1_BASE,TIMER_A_UP_MODE );
    }
