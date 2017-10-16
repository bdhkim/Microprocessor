/*
 * EC 450 HW 3
 * Dong Hyun Kim
 * U68030607
 * This program uses the Joystick controller to change the volume when moved in vertical direction,
 * while changing the frequency when moved in the horizontal direction.
 * This is done by using the WDT interrupt, which is constantly checking to see
 * if there has been any change to the frequency, which influences the volume and the pitch of the sound.
 * Namely, this is resultBuffer[0] for the frequency, and resultBuffer[1] for the volume.
 *
 * In terms of locking and unlocking the frequency, port 5.1 is used to lock the frequency, and 3.5 to unlock.
 * Blue LED, or port 5.6, is enabled when the lock is on.
 *
 * The limitation, however, is that there is a set max volume that the MSP432 cannot exceed.
 * This is due to the ratio set in the setCompareValue function.
 *
 */


// Full set of include files including graphics and driver libraries
// and also the LCD driver which is part of the project itself
#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>

/********************************************
 * Global Variables shared between handlers
 ********************************************/


/* ADC results buffer */
uint16_t resultsBuffer[2];           // latest readings from the analog inputs
uint16_t buttonPressed;              // 1 if joystick button is pressed
uint16_t print_flag;                 // flag to signal main to redisplay - set by ADC14


/***************************************************************
 * WDT system
 * The WDT is used to trigger sampling the ADC every 8192 cycles
 * (interval = 8192/10MHz = .8192ms)
 ***************************************************************/



#define PERIOD 100
#define DUTY_CYCLE_STEP 10
#define INITIAL_DUTY_CYCLE 10
#define AUDIO_COUNT 50
#define DEBOUNCE_COUNT 2000

unsigned int debounce_counter;
unsigned int counter;



void WDT_A_IRQHandler(void)
{
    MAP_ADC14_toggleConversionTrigger(); // trigger the next conversion


    MAP_Timer_A_setCompareValue ( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0, 100 + resultsBuffer[0]/1000);   //x-coordinate frequency control
    // This function constantly checks if there is any change in the frequency. By dividing this frequency by 1000, we effectively put the frequency level to what humans can hear.

    MAP_Timer_A_setCompareValue ( TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, resultsBuffer[1]/200);   //y-coordinate volume control
    //the 'volume' is the ratio between Register0 divided by Register4.
    //"200" is the number that changes this proportion. IF 200 becomes smaller, the initial volume is louder.

}



void init_WDT(){
    MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKITERATIONS_8192);
    MAP_WDT_A_startTimer(); // start the timer
}

/*************************************************
 * ADC14 Subsystem
 *************************************************/
/*
 * ADC Interrupt handler
 * This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer
 */


void TA0_0_Handler(){

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);

    if (--counter == 0){
        TIMER_A0->CCTL[4]^=TIMER_A_CCTLN_OUTMOD_7; // toggle between mode 0 andmode 7 reset/set
        counter=AUDIO_COUNT;
    }


}


void ADC14_IRQHandler(void)
{
    uint64_t status;
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM1 conversion completed */
    if(status & ADC_INT1)
    {
        /* Store ADC14 conversion results */
        resultsBuffer[0] = MAP_ADC14_getResult(ADC_MEM0);
        resultsBuffer[1] = MAP_ADC14_getResult(ADC_MEM1);

        /* Determine if JoyStick button is pressed */
        buttonPressed = (P4IN & GPIO_PIN1)?0:1;

        print_flag=1;  // signal main to refresh the display
    }
}

/*
 * ADC Setup
 */
void init_ADC(){
    /* Configure Pin 6.0 (A15) and 4.4 (A9) to be analog inputs ('tertiary function') */
    /* see the port 4 and port 6 pinout details of the MSP432p401r */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8)
     * drive from the internal ASD oscillator
     * with predivider 64 and divider 8, no routing to internal pins
     */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, ADC_NOROUTE);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
     * Basic operation is sequence mode with
     *   ADC-MEM0 -- A15
     *   ADC_MEM1 -- A9
     *
     *   NO automatic repeats
     */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, false); // use MEM...MEM1 channels
    // configure each memory channel:

    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     * is complete and enabling conversions
     */
    MAP_ADC14_enableInterrupt(ADC_INT1);


    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Enable conversions (must be triggered using ASC14_toggle_conversion()) */
    MAP_ADC14_enableConversion();
    //MAP_ADC14_toggleConversionTrigger();

}



/***********************************************************************************************************
 * DISPLAY Section
*/

// Color parameters for drawing on the screen - see grlib.h
#define TEXTCOL GRAPHICS_COLOR_YELLOW
#define BACKCOL GRAPHICS_COLOR_BLACK
#define DOTCOL GRAPHICS_COLOR_LIGHT_GREEN
#define DOTCOL_PRESSED GRAPHICS_COLOR_RED
#define RADIUS 2

// Graphics Globals (used by put_dot and ADC14 handler)

Graphics_Context g_sContext;    // graphics context for grlib
uint16_t xscreen, yscreen;      // current screen location coordinates


// Draw a dot (small circle) on the screen at position x,y of color dotcolor
// also ERASE previous dot (remembered in globals xscreen, yscreen)
void put_dot(uint16_t x,uint16_t y, uint32_t dotcolor){
    // erase previous dot
    Graphics_setForegroundColor(&g_sContext, BACKCOL);
    Graphics_fillCircle(&g_sContext,xscreen,yscreen,RADIUS);
    // draw the requested circle
    Graphics_setForegroundColor(&g_sContext, dotcolor);
    Graphics_fillCircle(&g_sContext,x,y,RADIUS);
    xscreen=x;
    yscreen=y;
}

// text printout of joystick readings on the screen
void print_current_results(uint16_t *results){
    char string[8];

    Graphics_setForegroundColor(&g_sContext, TEXTCOL);

    sprintf(string, "X: %5d", results[0]);
    Graphics_drawString(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    20,
                                    116,
                                    OPAQUE_TEXT);

    sprintf(string, "Y: %5d", results[1]);
    Graphics_drawString(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    76,
                                    116,
                                    OPAQUE_TEXT);
}



void init_display(){
	/*
	 * All init code for the display
	 *
	 */
    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128,&g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, TEXTCOL);
    Graphics_setBackgroundColor(&g_sContext, BACKCOL);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawString(&g_sContext,
                                    "J:",
                                    AUTO_STRING_LENGTH,
                                    0,
                                    116,
                                    OPAQUE_TEXT);
    xscreen=0;
    yscreen=0;  // just use origin, first write is a background
}

/************************************************************************************************************/


/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_1, // 10MHz
PERIOD-1, //
 TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE , // Disable CCR0 interrupt
 TIMER_A_DO_CLEAR // Clear value
};

const Timer_A_CompareModeConfig ccr0_Config ={
TIMER_A_CAPTURECOMPARE_REGISTER_0,
TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
TIMER_A_OUTPUTMODE_OUTBITVALUE,
PERIOD-1    //This determines the frequency
};



const Timer_A_CompareModeConfig ccr4_Config ={
TIMER_A_CAPTURECOMPARE_REGISTER_4,
TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,
TIMER_A_OUTPUTMODE_RESET_SET,
INITIAL_DUTY_CYCLE
};


void init_timer(){ // initialization and start of timer
 /* Configuring Timer_A1 for Up Mode */
 MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
 // configure channels
 MAP_Timer_A_initCompare(TIMER_A0_BASE, &ccr0_Config);
 MAP_Timer_A_initCompare(TIMER_A0_BASE, &ccr4_Config);
 counter=AUDIO_COUNT;
 MAP_Timer_A_registerInterrupt(TIMER_A0_BASE,TIMER_A_CCR0_INTERRUPT,TA0_0_Handler);

MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start TA0 in upmode
}



/**********************************
 * Main function
 **********************************/

void main(void)
{
    // Variables for refresh of display
    unsigned dotcolor;          // color of the dot to display
    uint16_t xdisplay,ydisplay; // screen coordinates to display

    /* Halting WDT and disabling master interrupts */
	MAP_CS_setDCOFrequency(10000000); // 10 MHz

	init_WDT();

	init_timer();

    init_button(); // initialize the button

    init_display(); // setup the display
    print_flag=0;   //clear print flag until there is a result
    init_ADC();

    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    /* Enable Interrupts at the NVIC level */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableInterrupt(INT_WDT_A);
    MAP_Timer_A_enableInterrupt(TIMER_A0_BASE);


    TIMER_A0->CCTL[0]^=TIMER_A_CCTLN_OUTMOD_4;

    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger
        if (print_flag)
        {
        	print_flag=0;
            dotcolor=buttonPressed ? DOTCOL_PRESSED: DOTCOL;
            xdisplay=resultsBuffer[0]/128;
            ydisplay=127-resultsBuffer[1]/128;

        	print_current_results(resultsBuffer);
        	put_dot(xdisplay,ydisplay,dotcolor);

        }
    }
}


void PORT5_IRQHandler(){     //LOCK THE FREQUENCY by disabling the watchdog interrupt

    if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5) & GPIO_PIN1){ // check that it is the button interrupt flag

        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN1); // clear the flag to allow for another interrupt later.

        //P2-> OUT &= ~BIT4; //turn off green LED

        P5->DIR |= BIT6; //Blue LED
        P5->OUT |= BIT6; //Blue LED

        MAP_Interrupt_disableInterrupt(INT_WDT_A);


    }
}

void PORT3_IRQHandler(){     //UNLOCK THE FREQUENCY by re-enabling the interrupt
    if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P3) & GPIO_PIN5){ // check that it is the button interrupt flag

           MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3,GPIO_PIN5); // clear the flag to allow for another interrupt later.

           P5-> OUT &= ~BIT6; //turn off blue LED

           MAP_Interrupt_enableInterrupt(INT_WDT_A);

           //initialize the graphics again!
           void put_dot(uint16_t x,uint16_t y, uint32_t dotcolor);
           void print_current_results(uint16_t *results);
           void init_display();

       }


}


void init_button(){
// All GPIO's are already inputs if we are coming in after a reset
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5,GPIO_PIN1);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5,GPIO_PIN1);
    MAP_GPIO_registerInterrupt(GPIO_PORT_P5,PORT5_IRQHandler);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P5,GPIO_PIN1);

    //for second button
    //BUTTON 2    ---> PORT 3.5
    MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P3,GPIO_PIN5);
    MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P3,GPIO_PIN5,GPIO_HIGH_TO_LOW_TRANSITION);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P3,GPIO_PIN5);
    MAP_GPIO_registerInterrupt(GPIO_PORT_P3,PORT3_IRQHandler);
    MAP_GPIO_enableInterrupt(GPIO_PORT_P3,GPIO_PIN5);

}



