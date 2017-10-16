/*
 * EC 450 HW 3
 * Dong Hyun Kim
 * U68030607
 * This stores an array of 'notes,' which are just different values of frequencies as shown in the commented list
 * 
 * There are two different buttons from the GPIO pins that toggle the song or change the speed of the song. 
 * This is done by the interrup handlers, which are constantly checking for changes in the state within the main function.  
 * Namely, Port 3 toggles the song, and Port 5 changes the speed of the song. 
 *
 * The main song is Bellacio, which will be played by default, and the other song is my all-time favorite, For Elise. 
 *
 *
 * As a side note, debouncing should be resolved! 
 *
 */



#include "msp.h"

#define BUTTON_PORT P5
#define BUTTON_BIT  BIT1

#define BUTTON_PORT2 P3
#define BUTTON_BIT2 BIT5

//variables
volatile float initialHalfPeriod = 1;

volatile unsigned int tempo = 100; //initial tempo for BellaCaio

volatile unsigned int tempoB = 50;

volatile unsigned int tempoA = 100;

volatile unsigned int tempoC = 80;  //original tempo for Elise

volatile unsigned int tempo2 = 80;  //original tempo for Elise

volatile unsigned int WDTcount = 100; //sets tempo

char flag = 0; //0 = play BellaCiao, 1 = play Elise

char staccato_flag=0;

char off=0; //0 = play, 1 = off



//songs
//0 = rest, -1 = staccato break between separate notes of the same pitch, otherwise pitch, each member of array is 1/8th note, repeat frequency for longer notes


//E3  = 3033
//Fs3 =  2703
//G3  = 2551
//Gs3 =  2408
//A3  = 2273
//As3 =  2145
//B3  = 2025
//C4  = 1911
//Cs4 =  1804
//Db4 =  1804
//D4  = 1703
//Ds4 =  1607
//Eb4 =  1607
//E4  = 1517
//F4  = 1432
//Fs4 =  1351
//Gb4 =  1351
//G4  = 1276
//Gs4 =  1204
//Ab4 =  1204
//A4  = 1136
//As4 =  1073
//Bb4 =  1073
//B4  = 1012
//C5  = 956
//Cs5 =  902
//Db5 =  902
//D5  = 851
//Ds5 =  804
//Eb5 =  804
//E5  = 758
//F5  = 716
//Fs5 =  676
//Gb5 =  676
//G5  = 638
//Gs5 =  602
//Ab5 =  602
//A5  = 568



//                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 17
float BellaCiao[100]={

                           3033, 2273, 2025, 1911, 2273, 2273, 2273, 3033, 2273, 2025, 1911, 2273, 2273, 2273, 3033,

                           2273, 2025, 1911, 1911, 2025, 2273, 1911, 1911, 2025, 2273, 1517, 1517, -1  , 1517, 1517, -1  , 1517, -1, -1, -1,
                           //미     //레    /미       /파                //파                                             //파     //미    //레    //파     /미
                           1517, 1703, 1517, 1432, -1  , 1432, 1432, 1432, -1  , -1  , 1432, 1517, 1703, 1432, 1517, 1517, 1517, -1 , -1,
                                              //시               /미                 /시               //도                 //라-
                           1517, 1703, 1911, 2025, 2025, 1517, 1517, 2025, 2025, 1911, 1911, 2273, 2273, 2273, 2273, -1,


                      };



float Elise[200]={
                          //미     /레b   /미      /레b   /미       /시      /레     /도      /라
                          1517, 1607, 1517, 1607, 1517, 2025, 1703, 1911, 2273, 2273, 2273, -1,
                          //도     /미       /라      /시                                       /미      /솔#    /시    /도
                          3822, 3033, 2273, 2025, 2025, 2025, -1  , 3033, 2408, 2025, 1911, 1911, 1911, 1911, -1,
                          //미     /레b   /미      /레b   /미       /시      /레     /도      /라
                          1517, 1607, 1517, 1607, 1517, 2025, 1703, 1911, 2273, 2273, 2273, -1,
                          //도     /미       /라      /시                                       /미      /도    /시    /라
                          3822, 3033, 2273, 2025, 2025, 2025, -1  , 3033, 1911, 2025, 2273, 2273, 2273

                 };




int increment=0;

// declare functions defined below
void mapports();         // connect TACCR0 to P2.7 using PMAP
void init_timer(void);   // routine to setup the timer
void init_button(void);  // routine to setup the button

void WDT_A_IRQHandler(void)
{
    if(!off)
    {
        if(flag==0) //play BellaCiao
        {
            P2-> OUT |= BIT4; //turn on green LED
            P5-> OUT &= ~BIT6; //turn off blue LED

            if(increment<1200) //not end of the song
                {
                    if(WDTcount==0)
                        {
                            if(BellaCiao[increment]>0)
                            {
                                TIMER_A0->CCTL[0]|=TIMER_A_CCTLN_OUTMOD_4;
                                if(staccato_flag) //if there was staccato,
                                {
                                    WDTcount=tempo-1;
                                    staccato_flag=0;
                                }
                                else
                                {
                                    WDTcount=tempo;
                                }
                                initialHalfPeriod = (BellaCiao[increment]);
                                TIMER_A0->CCR[0] = initialHalfPeriod-1;
                            }
                            else if(BellaCiao[increment]==0)
                            {
                                if(staccato_flag) //if there was staccato,
                                {
                                    WDTcount=tempo-1;
                                    staccato_flag=0;
                                }
                                else
                                {
                                    WDTcount=tempo;
                                }

                                //P2->OUT &= ~BIT7;
                                TIMER_A0->CCTL[0]&=~TIMER_A_CCTLN_OUTMOD_4;

                            }
                            else if(BellaCiao[increment]<0)
                            {
                                TIMER_A0->CCTL[0]&=~TIMER_A_CCTLN_OUTMOD_4;
                                WDTcount=1;
                                staccato_flag=1; //to account for lost time
                            }
                            increment++;

                        }
                    else
                        {
                            WDTcount--;
                        }

                }
        }
            else if(flag==1) //play Elise
            {

                //increment = 0;

                P5-> OUT |= BIT6; //turn on blue LED
                P2-> OUT &= ~BIT4; //turn off green LED
                    if(increment<1200) //not end of the song
                        {
                            if(WDTcount==0)
                                {
                                    if(Elise[increment]>0)
                                    {
                                        TIMER_A0->CCTL[0]|=TIMER_A_CCTLN_OUTMOD_4;
                                        if(staccato_flag) //if there was staccato,
                                        {
                                            WDTcount=tempo2-1;
                                            staccato_flag=0;
                                        }
                                        else
                                        {
                                            WDTcount=tempo2;
                                        }
                                        initialHalfPeriod = (Elise[increment]);
                                        TIMER_A0->CCR[0] = initialHalfPeriod-1;
                                    }
                                    else if(Elise[increment]==0)
                                    {
                                        if(staccato_flag) //if there was staccato,
                                        {
                                            WDTcount=tempo2-1;
                                            staccato_flag=0;
                                        }
                                        else
                                        {
                                            WDTcount=tempo2;
                                        }

                                        //P2->OUT &= ~BIT7;
                                        TIMER_A0->CCTL[0]&=~TIMER_A_CCTLN_OUTMOD_4;

                                    }
                                    else if(Elise[increment]<0)
                                    {
                                        TIMER_A0->CCTL[0]&=~TIMER_A_CCTLN_OUTMOD_4;
                                        WDTcount=1;
                                        staccato_flag=1; //to account for lost time
                                    }
                                    increment++;

                                }
                            else
                                {
                                    WDTcount--;
                                }

                        }

                    if(increment==53)     //play the song from beginning again
                        {
                            increment=0;
                        }


                }
    }


    if(increment==70)     //play the song from beginning again
        {
            increment=0;
        }



}

// ++++++++++++++++++++++++++
void main(){

    WDT_A->CTL = WDT_A_CTL_PW |             // 'password' to enable access
                    WDT_A_CTL_SSEL__SMCLK |         // clock source = SMCLK
                    WDT_A_CTL_TMSEL |               // this bit turns on interval mode
                    WDT_A_CTL_CNTCL |               // clear the internal WDT counter
                    WDT_A_CTL_IS_5;                 // specifies the divisor.  value 5 => 8K

    mapports();
    init_timer();  // initialize timer
    init_button(); // initialize the button

    P2->DIR |= BIT4; //Green LED (for "BellaCiao")
    P2->OUT |= BIT4; //Green LED is initially on

    P5->DIR |= BIT6; //Blue LED (for "Elise")
    P5->OUT &= ~BIT6; //Blue LED is off

    // setup NVIC and Interrupts
    SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;   // Specify that after an interrupt, the CPU wakes up

    __enable_interrupt();// unmask IRQ interrupts toallow the CPU to respond.
    // enable the interrupt for this program (this is done after unmasking CPU)
    // ISER = Insterrupt System Enable Register
    // A single 1 bit flag for each of the 64 theoretically possible interrupts
    // Notation: ISER[0] is for 0..31
    //           ISER[1] is for 31...63
    // Interrupt numbers are logical names in the include file.
    // Common numbers we have used in the early part of the course are:
    //     WDT_A_IRQn  =  3 (thus uses NVIC->ISER[0] to enable
    //     PORT5_IRQn  = 39 (uses NVIC->ISER[1])
    //     TA0_0_IRQn  =  8 (uses NVIC->ISER[0])
    //     TA0_N_IRQn  =  9 (uses NVIC->ISER[0])
    NVIC->ISER[1] = 1 << ((PORT5_IRQn) & 31); // enable P5 to send interrupt signals
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31); // enable P3 to send interrupt signals
    NVIC->ISER[0] = 1 << ((WDT_A_IRQn) & 31); // enable WDT to send interrupt signals

    while (1)
    {
        /* Go to LPM0 mode (Low power mode with CPU powered off */
        __sleep();        //
        __no_operation(); //  For debugger
    }

}

void mapports(){
    PMAP->KEYID=PMAP_KEYID_VAL; // unlock PMAP
    P2MAP->PMAP_REGISTER7=PMAP_TA0CCR0A;  // map TA0CCR0 to P2.7 as primary module
    PMAP->KEYID=0;              // relock PMAP until next hard reset
}


void init_timer(){              // initialization and start of timer
    TIMER_A0->CTL |= TIMER_A_CTL_CLR ;// reset clock
    TIMER_A0->CTL =  TIMER_A_CTL_TASSEL_2  // clock source = SMCLK
                    +TIMER_A_CTL_ID_0      // clock prescale=1
                    +TIMER_A_CTL_MC_1;     // Up Mode
    TIMER_A0->EX0 = TIMER_A_EX0_TAIDEX_2;  // additional divisor=3

    TIMER_A0->CCTL[0]=TIMER_A_CCTLN_OUTMOD_4; // compare mode, output mode 4 (toggle)
    TIMER_A0->CCR[0] = initialHalfPeriod-1; // in up mode TAR=0... TACCRO-1
    P2->SEL0|=BIT7; // connect timer output to pin (select alternate function for pin)
    P2->DIR |=BIT7; // output mode on P2.7 (direction output completes setting the function)
}


void init_button(){
// All GPIO's are already inputs if we are coming in after a reset
    BUTTON_PORT->OUT |= BUTTON_BIT; // pullup
    BUTTON_PORT->REN |= BUTTON_BIT; // enable resistor
    BUTTON_PORT->IES |= BUTTON_BIT; // set for 1->0 transition
    BUTTON_PORT->IFG &= ~BUTTON_BIT;// clear interrupt flag
    BUTTON_PORT->IE  |= BUTTON_BIT; // enable interrupt

    //BUTTON 2
    BUTTON_PORT2->OUT |= BUTTON_BIT2; // pullup
    BUTTON_PORT2->REN |= BUTTON_BIT2; // enable resistor
    BUTTON_PORT2->IES |= BUTTON_BIT2; // set for 1->0 transition
    BUTTON_PORT2->IFG &= ~BUTTON_BIT2;// clear interrupt flag
    BUTTON_PORT2->IE  |= BUTTON_BIT2; // enable interrupt

}

void PORT3_IRQHandler(){
    if (BUTTON_PORT2->IFG & BUTTON_BIT2){ // check that it is the button interrupt flag
        BUTTON_PORT2->IFG &= ~BUTTON_BIT2; // clear the flag to allow for another interrupt later.
        // This handler changes the state of the timer CCTL0 control register!
        // Toggle OUTMOD between
        //    mode 0: output = constant value determined by CCTL0 bit 2, and
        //    mode 4: toggle, which when repeated produces a square wave.
        if(!off)
        {
            P2-> OUT &= ~BIT4; //turn off green LED
            P5-> OUT &= ~BIT6; //turn off blue LED
            flag^=1;           //TOGGLE SONG////////////////////////////////////////////////////////////////////////////////////
        }

        //Pausing between the songs;
        TIMER_A0->CCTL[0]^=TIMER_A_CCTLN_OUTMOD_4;
        off = !off;
    }
}

//////////CHANGE TEMPO WHEN PORT 5 IS PRESSED ////////////////
void PORT5_IRQHandler(){
    if (BUTTON_PORT->IFG & BUTTON_BIT){ // check that it is the button interrupt flag
        BUTTON_PORT->IFG &= ~BUTTON_BIT; // clear the flag to allow for another interrupt later.
        // This handler changes the state of the timer CCTL0 control register!
        // Toggle OUTMOD between
        //    mode 0: output = constant value determined by CCTL0 bit 2, and
        //    mode 4: toggle, which when repeated produces a square wave.

            if(tempo==100) //changing tempo for BellaCiao
            {
                tempo=tempoB;
            }
            else if(tempo==50)
            {
                tempo=tempoA;
            }

            if(tempo2==80)  //changing tempo for Elise
            {
                tempo2=tempoB;
            }
            else if(tempo2==50)
            {
                tempo2=tempoC;
            }



      }
}
