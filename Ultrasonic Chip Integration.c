// Final Project with Ultrasonic Chip 
// Dong Hyun Kim
// Mertcan Cokbas

// • Goal and Design of the project
// The goal of this project was to build a system that would measure the distance of a near object 
// and the rate of change in distance. We also aimed to display both of those data for the user. 
// To accomplish this goal, we designed our MSP432 board to connect to an ultrasonic chip (HC-SR04) 
// that would allow us to measure how far or close an object in centimeters and inches.  

// • Description of the implementation including system level and component level interaction
// In order to first connect the ultrasonic chip to MSP432, we had to match the voltage levels of 
// these devices; MSP432 works with 3.3 volts, whereas the ultrasonic chip works with 5 volts. 
// To match the voltage levels, we used a bi-directional logic level converter (BOB-12009), 
// which required a 6-pin header. A small problem arose here where the level converter seemed 
// not to be working because it was not touching the header. This problem was solved by soldering 
// the 6-pin header to the level converter so that they would be completely connected. 
// The way the ultrasonic chip works is that once it receives a trigger signal from the MSP432 board, 
// ultrasonic chip sends 8 cycles of 40KHz ultrasound wave to the environment; after the 8th cycle, 
// the echo signal become HI, and it stays HI until the ultrasound wave comes back. Knowing this process, 
// our challenge was to first trigger a pulse and accurately measure the length of the echo pulse. 



#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <stdint.h>

#include <stdbool.h>
/********************************************
 * Global Variables shared between handlers
 ********************************************/
volatile unsigned int counter=50;

void init_display(void);
void init_timer(void);
void init_button(void);
void init_button1(void);

volatile unsigned int  counter1=0;   volatile unsigned int counter2=0;
 volatile unsigned int  check2=0;  volatile unsigned int  check15=0;
volatile unsigned int  check8=0; volatile unsigned int  check7=0; volatile unsigned int counter11=0;
uint32_t start=0; uint32_t finish=0; uint32_t distance=0; uint32_t finish1=0;
uint32_t current=0; uint32_t more_current=0; uint16_t distance2=10;
uint32_t distance1=0; uint16_t prev=0; uint16_t speed=0; uint16_t inches=0;

volatile unsigned int counter3=0; volatile unsigned int counter4=0; volatile unsigned int counter5=0;
volatile unsigned int counter12=0;
#define TEXTCOL GRAPHICS_COLOR_YELLOW
#define BACKCOL GRAPHICS_COLOR_BLACK


// Graphics Globals (used by put_dot and ADC14 handler)
uint16_t xscreen, yscreen;
Graphics_Context g_sContext;    // graphics context for grlib
/***************************************************************
 * WDT system
 * The WDT is used to trigger sampling the ADC every 8192 cycles
 * (interval = 8192/10MHz = .8192ms)
 ***************************************************************/
void print_current_results(uint16_t results){
    char string[8];



    Graphics_setForegroundColor(&g_sContext, TEXTCOL);

    sprintf(string, "%3d cm", results);
    Graphics_drawString(&g_sContext,
                                    (int8_t *)string,
                                    6,
                                    0,
                                    66,
                                    OPAQUE_TEXT);

    inches= (results*2)/5;
    sprintf(string, "%3d inches", inches);
        Graphics_drawString(&g_sContext,
                                        (int8_t *)string,
                                        10,
                                        0,
                                        76,
                                        OPAQUE_TEXT);
    sprintf(string, "%3d cm/s", speed);
       Graphics_drawString(&g_sContext,
                                       (int8_t *)string,
                                       8,
                                       64,
                                       66,
                                       OPAQUE_TEXT);


counter12++;
}


/*

void WDT_A_IRQHandler(void)
{
current=MAP_Timer32_getValue(TIMER32_0_BASE);

check8=1;
  P3->OUT |= BIT6;

  while(check8)
{more_current=MAP_Timer32_getValue(TIMER32_0_BASE);
if(more_current-current > 150)
{check8=0;}
}

  P3->OUT &= ~BIT6;
 // more_current=MAP_Timer32_getValue(TIMER32_0_BASE);
counter4++;




}
*/

void init_WDT(){
    MAP_WDT_A_initIntervalTimer(WDT_A_CLOCKSOURCE_SMCLK,WDT_A_CLOCKITERATIONS_8192K);
    MAP_WDT_A_startTimer(); // start the timer

}




/**********************************
 * Main function
 **********************************/
uint8_t xdisplay=64,ydisplay=64;    // screen coordinates to disolay
volatile unsigned int check14=1; volatile unsigned int check12=0; volatile unsigned int check32=0;
void main(void)
{
    MAP_WDT_A_holdTimer();

            //P3->DIR |= BIT6;
                  //  P3->OUT &= ~BIT6;


    MAP_GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN6);

    /* Halting WDT and disabling master interrupts */
    //MAP_CS_setDCOFrequency(12000000); // 12 MHz

        MAP_CS_setDCOFrequency(10000000); /* 10MHz */

        init_display(); // setup the display
        print_current_results(distance2);
        //init_WDT();

init_timer();


    MAP_Interrupt_disableSleepOnIsrExit();   // Specify that after an interrupt, the CPU wakes up
    MAP_Interrupt_enableMaster();

    /* Enable Interrupts at the NVIC level */
    init_button();
        init_button1();
   // MAP_Interrupt_enableInterrupt(INT_TA_0);


    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT,
           TIMER32_FREE_RUN_MODE);
           /* Starting the timer */
           MAP_Timer32_startTimer(TIMER32_0_BASE, true);//10us=100cycle


    while(1)
    {
        MAP_PCM_gotoLPM0();
        __no_operation(); //  For debugger


        if(check7)
        { //MAP_Interrupt_disableInterrupt(INT_WDT_A);

distance2=distance1;


if(distance>prev)
{//speed= ((distance2-prev)*10.0);
    speed=(((distance-prev)*17000)/1000000);
}

else
{//speed= ((prev-distance2)*10.0);
    speed=(((prev-distance)*17000)/1000000);
}




print_current_results(distance2);

prev=distance;
//prev=distance2;
check7=0;
        }
        counter11++;

    }
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
                                    "Distance:",
                                    AUTO_STRING_LENGTH,
                                    0,
                                    56,
                                    OPAQUE_TEXT);
    Graphics_drawString(&g_sContext,
                                        "Speed:",
                                        AUTO_STRING_LENGTH,
                                        70,
                                        56,
                                        OPAQUE_TEXT);
    xscreen=0;
    yscreen=0;  // just use origin, first write is a background
}

// TA0CCR0 Interrupt Handler
void TA0_0_Handler(){
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_0);


    current=MAP_Timer32_getValue(TIMER32_0_BASE);

    check8=1;
      P3->OUT |= BIT6;

      while(check8)
    {more_current=MAP_Timer32_getValue(TIMER32_0_BASE);
    if(more_current-current > 150)
    {check8=0;}
    }

      P3->OUT &= ~BIT6;
     // more_current=MAP_Timer32_getValue(TIMER32_0_BASE);
    counter4++;


}

/* Timer_A UpMode Configuration Parameter */
const Timer_A_UpModeConfig upConfig =
{
 TIMER_A_CLOCKSOURCE_SMCLK, // SMCLK Clock Source
 TIMER_A_CLOCKSOURCE_DIVIDER_1, // 10MHz
8000000, //
 TIMER_A_TAIE_INTERRUPT_DISABLE, // Disable Timer interrupt
 TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE , // Disable CCR0 interrupt
 TIMER_A_DO_CLEAR // Clear value
};
const Timer_A_CompareModeConfig ccr0_Config ={
TIMER_A_CAPTURECOMPARE_REGISTER_0,
TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,
TIMER_A_OUTPUTMODE_OUTBITVALUE,
8000000
};

void init_timer(){ // initialization and start of timer
 /* Configuring Timer_A1 for Up Mode */
 MAP_Timer_A_configureUpMode(TIMER_A0_BASE, &upConfig);
 // configure channels
 MAP_Timer_A_initCompare(TIMER_A0_BASE, &ccr0_Config);

 MAP_Timer_A_registerInterrupt(TIMER_A0_BASE,TIMER_A_CCR0_INTERRUPT,TA0_0_Handler);

//GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN7,GPIO_PRIMARY_MODULE_FUNCTION);
MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE); // start TA0 in up

}


void PORT2_IRQHandler(){

// Good practice to check which of the 8 pins caused the interrupt.
// In general, we would take different actions for interrupts on different
// input pins, and this handler would be called for any of them.
// In this program, there should be only the one interrupt.
if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P2) & GPIO_PIN2){
MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN2); // reset interrupt
}


start= MAP_Timer32_getValue(TIMER32_0_BASE);




//check7=1;

}


void PORT1_IRQHandler(){

// Good practice to check which of the 8 pins caused the interrupt.
// In general, we would take different actions for interrupts on different
// input pins, and this handler would be called for any of them.
// In this program, there should be only the one interrupt.
if (MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1) & GPIO_PIN0){
MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN0); // reset interrupt
}

finish= MAP_Timer32_getValue(TIMER32_0_BASE);

if(start > finish )
                        {distance=start-finish;
                        distance1=((distance*17000)/10000000)-1;

                         check7=1;

                        }

}

void init_button(){
// All GPIO's are already inputs if we are coming in after a reset
MAP_GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2,GPIO_PIN2);
MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P2,GPIO_PIN2,GPIO_LOW_TO_HIGH_TRANSITION);
MAP_GPIO_clearInterruptFlag(GPIO_PORT_P2,GPIO_PIN2);
MAP_GPIO_registerInterrupt(GPIO_PORT_P2,PORT2_IRQHandler); //registers handler

MAP_GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN2);
}


void init_button1(){
// All GPIO's are already inputs if we are coming in after a reset
MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN0);
MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P1,GPIO_PIN0,GPIO_HIGH_TO_LOW_TRANSITION);
MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1,GPIO_PIN0);
MAP_GPIO_registerInterrupt(GPIO_PORT_P1,PORT1_IRQHandler); //registers handler

MAP_GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN0);
}