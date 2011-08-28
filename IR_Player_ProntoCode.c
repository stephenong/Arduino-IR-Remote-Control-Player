/**
  ******************************************************************************
  * @file     IR_Player_ProntoCode.c 
  * @author   Stephen Ong
  * @version  V1.0.0
  * @date     08/01/2011
  * @brief    Infrared Remote Control Player. 
  *         
  ******************************************************************************
  * 
  * Infrared Remote Control Player by Stephen Ong (http://robotics.ong.id.au).
  * 
  * This infrared remote control player runs on the Arduino hardware platform.
  * It plays back IR code stored in Pronto Hex format. Useful for programming
  * learning universal remote control.
  *
  * @copy
  *
  * This work is licenced under the Creative Commons Attribution-NonCommercial 
  * 3.0 Unported License. To view a copy of this licence, visit 
  * http://creativecommons.org/licenses/by-nc/3.0/ or send a letter to Creative 
  * Commons, 171 Second Street, Suite 300, San Francisco, California 94105, USA.
  *
  * <a rel="license" href="http://creativecommons.org/licenses/by-nc/3.0/"><img alt="Creative Commons Licence" style="border-width:0" src="http://i.creativecommons.org/l/by-nc/3.0/88x31.png" /></a><br /><span xmlns:dct="http://purl.org/dc/terms/" href="http://purl.org/dc/dcmitype/Text" property="dct:title" rel="dct:type">Infrared Remote Control Player</span> by <a xmlns:cc="http://creativecommons.org/ns#" href="http://robotics.ong.id.au" property="cc:attributionName" rel="cc:attributionURL">Stephen Ong</a> is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc/3.0/">Creative Commons Attribution-NonCommercial 3.0 Unported License</a>.
  */ 


/* 
 * Changelog:
 * v1.0 - 2011/01/08 - First Release
 */


/* Includes ------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
 
/* Private define ------------------------------------------------------------*/
#define F_CPU           16000000        // System clock frequency
#define LED_PIN         _BV(1)          // OC1A -> PB1 = Arduino Digital Pin 9 
#define DEBUG_D13       _BV(5)          // GPIO -> PB5 = Arduino Digital Pin 13

#define PRONTO_IR_SOURCE            0   // Pronto code byte 0
#define PRONTO_FREQ_CODE            1   // Pronto code byte 1
#define PRONTO_SEQUENCE1_LENGTH     2   // Pronto code byte 2
#define PRONTO_SEQUENCE2_LENGTH     3   // Pronto code byte 3
#define PRONTO_CODE_START           4   // Pronto code byte 4

#define TOTAL_CYCLES 400000             // Turns off after this number of 
                                        // cycles. About 10 seconds
/* Hardware ------------------------------------------------------------------*/

//  Platform - Arduino Deumilanove - ATMEGA328p - 16MHz
//  
//  Connect IR LED anode to Arduino Digital Pin 9 (PB1) and connect its 
//  cathode to GND. 

/* Pronto Code Format---------------------------------------------------------*/

//  Reference:
//    http://www.remotecentral.com/features/irdisp1.htm
// 
//  Decoding:
//  1. First number is always 0 which means leared IR pattern.
//  2. Second number: Carrier freq in Hz. Freq = 1000000/(N * .241246) 
//  3. Third number: number of burst pair in non repeating burst pair #1
//  4. Fourth number: number of burst pair in repeating bust pair #2
//  5. Start of burst pairs. Burst pair #1 followed by burst pair #2
//
//  Eg Power off:
//  0000 0067 0000 000d 
//  0060 0018 
//  0030 0018 
//  0030 0018 
//  ...
//  ...
//  0018 03de
//
//  Word 1: 0x0000
//  Word 2: 0x0067 -> Carrier 40.244kHz
//  Word 3: 0x0000 -> 0 non repeating burst pair
//  word 4: 0x000d -> 13 repeating burst pair
//  Word 5, 6: Burst pair 2 -> 0x0060 0x0018 -> 96 cycles on, 24 cycles off
//  etc


/* Pronto Codes --------------------------------------------------------------*/

//Test Pronto Code Signal
uint16_t ProntoCodeTest[] = {
    0x00,                   //0 - recorded waveform
    0x6D,                   //38kKhz. 0x67=>40kHz, 0x6D=>38kHz
    0x01,                   //Sequence 1 on/off pair length
    0x03,                   //Sequence 2 on/off pair length
    0x05, 0x01,             //Sequence 1, pair 1
    0x02, 0x05,             //Sequence 2, pair 1
    0x04, 0x03,             //Sequence 2, pair 2
    0x06, 0x07};            //Sequence 2, pair 3


// Power Off - Sony LCD (2010) Television (Power Control) 
// http://files.remotecentral.com/library/3-2/sony/lcd_%282010%29_television/index.html
uint16_t ProntoCodeSonyOff[] = 
    {0x00, 0x67, 0x00, 0x0d,
     0x60, 0x18, 0x30, 0x18, 0x30, 0x18, 0x30, 0x18, 
     0x30, 0x18, 0x18, 0x18, 0x30, 0x18, 0x18, 0x18, 
     0x30, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
     0x18, 0x3de };

uint16_t *ProntoCode = ProntoCodeSonyOff;


/* Private functions ---------------------------------------------------------*/

/**************************************************************************//**
 * Turns IR LED on and off. This is done by connecting or disconnecting OCR1A
 * to physical pin. When connected, pin is pulsed at carrier frequency. When
 * disconnected pin works as GPIO, ie off.
 * @param on: LED is on
 * @retval None
 *****************************************************************************/

uint8_t gIRLED_Power;       //True when IR led is on

void IRLED_Power(uint8_t on) {
    if (on) {
        TCCR1A |= (1<<COM1A1) + (1<<COM1A0);
        gIRLED_Power = true;
        PORTB |= DEBUG_D13;  
    }
    else {
        TCCR1A &=  ( (~(1<<COM1A1)) & (~(1<<COM1A0)) );
        gIRLED_Power = false;
        PORTB &= ~DEBUG_D13;
    }
}

/**************************************************************************//**
 * Toggles IR LED state.
 * @retval None
 *****************************************************************************/
void IRLED_Toggle() {
    if(gIRLED_Power) {
        IRLED_Power(false);
    }
    else {
        IRLED_Power(true);
    }
}

/**************************************************************************//**
 * ISR of timver1 overflow. This sequences the pronto code
 * @retval None
 *****************************************************************************/
uint16_t gCycleCount;
uint32_t gTotalCycleCount;
uint8_t gCurrentSequenceIndex;

ISR(TIMER1_OVF_vect) {
    uint16_t sequenceIndexEnd;          // Index to the last element in the 
                                        // ProntoCode array
    uint16_t repeatSequenceIndexStart;  // Index to ProntoCode array to repeat

    gTotalCycleCount++;
    gCycleCount++;

    // End of this state, toggle led and move to the next state
    if (gCycleCount==ProntoCode[gCurrentSequenceIndex]) {
        IRLED_Toggle();
        gCycleCount = 0;
        gCurrentSequenceIndex++;
        sequenceIndexEnd = PRONTO_CODE_START +
                           (ProntoCode[PRONTO_SEQUENCE1_LENGTH]<<1) +
                           (ProntoCode[PRONTO_SEQUENCE2_LENGTH]<<1);
                                    
        repeatSequenceIndexStart = PRONTO_CODE_START +
                                   (ProntoCode[PRONTO_SEQUENCE1_LENGTH]<<1);

        // If index past last element in array, set index to repeat
        if (gCurrentSequenceIndex >= sequenceIndexEnd ) {        
            gCurrentSequenceIndex = repeatSequenceIndexStart;     

            if(gTotalCycleCount>TOTAL_CYCLES) {            // Finished
                TCCR1B &= ~(1<<CS10);                    // Stop Timer
                IRLED_Power(false);                        // Turn off LED
            }
        }
    }
}

/**************************************************************************//**
 * Main function. Setup gpio ports, timer1, then wait in infinite loop. The IR
 * code is played back for TOTAL_CYCLES cycles.
 * @retval none
 *****************************************************************************/
int main (void){
    PORTB = 0x00;                       // When LED_PIN disconnected from  
                                        //    counter, LED will be in off state
    DDRB = LED_PIN | DEBUG_D13;         // Set LED_PIN and DEBUG_D13 for output
    
                                        // SETUP TIMER 1    
    TCCR1A = 0x00;                      // WGM=0000 for instant update of OCR1A
    TCCR1B = 0x00;

    uint16_t top = ( (F_CPU/1000000.0) * 
                     ProntoCode[PRONTO_FREQ_CODE]  * 
                     0.241246 ) - 1;

    ICR1 = top;                         // Counter counts from 0 to top
    OCR1A = top>>1;                     // PWM set to 50%, set OCR1A to top/2

    
    TCCR1A = (1<<WGM11);                // WGM = 1110 => Fast PWM, TOP=ICR1, 
    TCCR1B = (1<<WGM13) | (1<<WGM12);   // TOV1 flag set on TOP, 
                                        // OCR1x updated at BOTTOM

    TCNT1 = 0x0000;                     // Clear counter
    TIFR1 = 0x00;                       // Clear any pending interrupt
    TIMSK1 = 1<<TOIE1;                   // Enable Overflow Interrupt

    IRLED_Power(true);                  // IR Led starts with the ON state,
                                        //   this connects OCR1A out to port pin
    
    gCurrentSequenceIndex = PRONTO_CODE_START;    
    gCycleCount = 0;

    
    TCCR1B |= (1<<CS10);                // Start timer running, Clock = CLKio/1

    sei();                              // Enable global interrupt

    while (true) {}                     // Background loop does nothing
}

//---------------------------END OF IR_Player_ProntoCode.c -------------------

