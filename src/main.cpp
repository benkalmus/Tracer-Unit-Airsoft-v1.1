#include <Arduino.h>
#include <inttypes.h>
#include <avr/sleep.h>

//#define     DEBUG_SERIAL
#define     DISABLE_SLEEP
#define     DISABLE_BATTERY_CHECK
#define     ALTERNATE_UV_PIN            

#define MAX_RPS         100.0f        //maximum rounds per second
#define ADC_INTERVAL_MS    500
#define POWER_SAVE_TIME_MS   1800000      //30mins

#define POWERSAVER_DDR  DDRB
#define POWERSAVER_PORT  PORTB
#define POWERSAVER_PIN  PB2
#define POWERSAVER_PINREAD  PINB
//#define POWERSAVER_OFF_CLICKS   4  //clicks required to toggle powersaver off
//#define POWERSAVER_OFF_TIME_MS    2000 //in what period of time should the user click the button to disable safety

#define UV_ON_TIME_US       880
#ifndef ALTERNATE_UV_PIN
#define LED_DDR     DDRA
#define LED_PORT    PORTA    //UV tracer led port
#define LED_PIN    PA0 //
#else                           //#######################
//                                The alternate pin assignment exists because the TX IR-diode may exist on the same port and as such limit the current draw necessary to switch MOSFET.
#define LED_DDR     DDRB            
#define LED_PORT    PORTB    //UV tracer led port
#define LED_PIN    PB1 //
#endif
//#define LED_INVERT 

#define LED_TX_DDR  DDRA
#define LED_TX_PORT    PORTA    //
#define LED_TX_PIN     PA3    //IR LED

#define LED_RX_DDR  DDRA        //RX DIODE INPUT
#define LED_RX_PORT    PORTA   //
#define LED_RX_READPIN     PINA    //
#define LED_RX_PIN     PA7    //
#define LED_RX_INVERT   

#define BATT_DDR    DDRA
#define BATT_PORT   PORTA
#define BATT_PIN    PA2
//#define BATT_MULTIPLIER 3.93f/830.0f*0.86     // 0.0040720482f 
#define BATT_MULTIPLIER     0.004010f
#define BATT_MIN_VOLTAGE    2600     //in milliVolts, 3.3V = 3300mV
#define BATT_MAX_VOLTAGE    4500     //4.5V , clearly erronous reading.


uint32_t timeADC = 0; 
uint32_t shotTime = 0;
uint16_t shotCounter = 0;
uint8_t checkBattery = 1;

uint16_t battVoltage = 0;
uint32_t maxFireRate = uint32_t(1000000.0f / MAX_RPS);
uint8_t detectedBB = 0;

//                  FUNCTION

void deepSleep()
{
#ifdef DEBUG_SERIAL
    Serial.println("Going to Sleep. Wake me up by pushing the toggle button");
#endif
    //PB2 BUTTON for wake up PCINT10
    GIMSK |= 1<<PCIE1;                     // Enable Pin Change Interrupts
    PCMSK1 |= 1<<PCINT10;                   // Use PB2 as interrupt pin
    ADCSRA &= ~(1<<ADEN);                   // ADC off
    //PCMSK0 &= ~(1<<PCINT7);             //disable RX DIODE interrupt
    //turn every LED off to save power
    LED_PORT &= ~(1 << LED_PIN);
    //LED_TX_PORT &= ~(1<< LED_TX_PIN);
    //sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK0 |= (1 << PCINT7);                //enable RX DIODE interrupt
    PCMSK1 &= ~(1<<PCINT10);                  // Turn off PB2 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    //turn LEDs back on
    //LED_PORT |= (1 << LED_PIN);
    LED_TX_PORT|= (1 << LED_TX_PIN);

    sei();                                  // Enable interrupts
    shotTime = micros();//update timers
    timeADC = millis(); 
}

//                SETUP

void setup()
{
    LED_RX_DDR &= ~(1 << LED_RX_PIN);       //input
    //pinMode(LED_RX_PIN, INPUT);
    //outputs:
    LED_DDR |= (1 << LED_PIN);      //output
    //TX led
    LED_TX_DDR |= (1<<LED_TX_PIN);
    /*  init ADC        */
    BATT_DDR &= ~(1 << BATT_PIN);//set pin to input
    POWERSAVER_DDR &= ~(1<<POWERSAVER_PIN);     //input
    //selecting voltage reference
    //ADMUX |= (1 << REFS1);     //internal 1.1v ref
    //ADMUX &= ~(1 << REFS0); 
    //voltage pin is PA2, so set according bits
    //ADMUX |= (1 << MUX1);     //pa2
    ADMUX |= (1 << MUX2);       //pa4
    //ADMUX &= ~((1 << MUX0) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4) | (1 << MUX5));
    //left adjust
    //ADCSRB |= (1<<ADLAR);      
    //free running mode: data collected automatically
    //single conversion mode: data collected when bit is written to adcsra
    //prescaler, division factor from system clock. This chip runs at 8mhz, to achieve rate of 128khz, divide by 64:
    ADCSRA |= (1 << ADPS1) | (1 << ADPS2);// | (1 << ADPS0);
    //DIDR0 |= 1 << ADC2D;        //disable digital input from pin 2, saves power
    DIDR0 |= 1 << ADC4D;        //disable digital input from pin 4, saves power
    ADCSRA |= 1 << ADEN;    //adc enable
    //ADCSRA |= 1 << ADIE;    //adc interrupts
    for (uint8_t i = 0; i < 10; i++)    //read several samples 
    {
        ADCSRA |= 1 << ADSC;    //begin conversion
        delay(50); //await conversion
        uint8_t low = ADCL;
        int16_t volt = (ADCH << 8) | low;
        volt = (float)volt * BATT_MULTIPLIER* 1000;
        if (i == 0) battVoltage = volt * 1000;
        else battVoltage = (volt * 0.95) + (battVoltage * 0.05);
    }
    //turn diodes ON
    LED_TX_PORT |= (1 << LED_TX_PIN);
    //pull  up
    POWERSAVER_PORT |= 1 << POWERSAVER_PIN;

#ifdef DEBUG_SERIAL
    Serial.begin(9600);
    delay(1500);
    Serial.println("Connected");
#endif

#ifdef LED_RX_INVERT
    if ( (LED_RX_READPIN & (1 << LED_RX_PIN)) != 0)    //something is blockin the tracer at startup
#else
    if ( (LED_RX_READPIN & (1 << LED_RX_PIN)) == 0)    //something is blockin the tracer at startup
#endif
    {
#ifdef DEBUG_SERIAL 
        Serial.println("Battery Safety has been switched off. ");
#endif
        //battery safety turn off
        checkBattery = false;
        for (uint8_t i = 0; i < 6; i++)    //flash leds to indicate safety off
        {
            LED_PORT |= (1 << LED_PIN);
            delay(1);
            LED_PORT &= ~(1 << LED_PIN);
            delay(400);
        }
    }
    //leds off
#ifdef LED_INVERT
    LED_PORT |= (1 << LED_PIN);
#else 
    LED_PORT &= ~(1 << LED_PIN);
#endif
    /*      init ISR      */
    GIMSK |= (1 << PCIE0);      //enable pin change interrupt on pins [0:7];
    PCMSK0 |= (1 << PCINT7);    //enable interrupt on RX DIODE
    sei();      //enable global interrupts
}


void loop() 
{    //read diode pin, sometimes the ISR is a little bit slow
#ifdef LED_RX_INVERT
    if ( ((LED_RX_READPIN &  (1<<LED_RX_PIN) ) != 0) || detectedBB)     //check if BB passed through
#else
    if ( ((LED_RX_READPIN &  (1<<LED_RX_PIN) ) == 0) || detectedBB)     //check if BB passed through
#endif
    //if ( detectedBB)     
    {
        //disable ISR
        //PCMSK0 &= ~(1 << PCINT7);
        uint32_t now = micros();
        //bench test, BB travelling at roughly 2.5-3.5fps, reaches led array after ~36mm, appropriate delay would be: 0.043s or 43ms
        //delayMicroseconds(48000);


        if (now - shotTime >= maxFireRate)
        {  
            // turn UV on
        #ifdef LED_INVERT
            LED_PORT &= ~(1 << LED_PIN);
        #else 
            LED_PORT |= (1 << LED_PIN);
        #endif
            shotTime = now;
#ifdef DEBUG_SERIAL
            Serial.println(now);
#endif
        }
        //how long to keep the leds on:
        //travelling at ~ 300fps over dist of 70mm, time = 0.07m / 91.4ms-1 = 766us
        delayMicroseconds(UV_ON_TIME_US);
        //delay(2000);
        //turn UV off
    #if defined(LED_INVERT)
        LED_PORT |= (1 << LED_PIN);
    #else 
        LED_PORT &= ~(1 << LED_PIN);
    #endif 
        //delay(500);

        //enable ISR
        //PCMSK0 |= (1 << PCINT7);
        //flag
        detectedBB = 0;
        shotCounter += 1;
#ifdef DEBUG_SERIAL 
        //Serial.print("detected BB, time: ");
        //Serial.println(shotTime);
        //Serial.print("\t Shot Counter: ");
        //Serial.println(shotCounter);
#endif
    }



    if (millis() - timeADC > ADC_INTERVAL_MS)
    //if (millis() - timeADC > 50)
    {
        timeADC = millis();
        int16_t volt = -1;
        if (checkBattery) 
        {
            //read ADC
            uint8_t low = ADCL;
            volt = (ADCH << 8) | low;
            float v = (float)volt * BATT_MULTIPLIER;
            volt = v * 1000;
            battVoltage = (volt * 0.95) + (battVoltage * 0.05);
            ADCSRA |= 1 << ADSC;    //start conversion for next loop
        }
#ifdef DEBUG_SERIAL
        Serial.print("battery: ");
        Serial.print(battVoltage);
        Serial.print("V\tcurrent: ");
        Serial.print(volt);
        Serial.print("V\tSec since last: ");
        Serial.println((timeADC-(shotTime/1000))/1000);
        //Serial.print("\t adcl: ");
        //Serial.println(ADCH);
        //Serial.println(volt);
        //scale is 0-1024
        /*uint8_t reading = LED_RX_READPIN & (1 << LED_RX_PIN);
        Serial.print("\t digital: ");
        Serial.println(reading);*/
#endif

        #ifndef DISABLE_BATTERY_CHECK
        //if battery level below min, power down mode.
        if (checkBattery && (battVoltage < BATT_MIN_VOLTAGE ) )//|| battVoltage > BATT_MAX_VOLTAGE)  )
        {
            //deep sleep
            deepSleep();
        }
        #endif

        //also check power save timing
        //timeADC is just millis but less instructions
        if (millis() - (shotTime/1000) >= POWER_SAVE_TIME_MS)
        {
            #ifndef DISABLE_SLEEP
            deepSleep();
            #endif
        }
    }
}

/* INTERRUPT 
   For detecting RX diode voltage fall.
*/
ISR(PCINT0_vect)
{
#ifdef LED_RX_INVERT
    if ((LED_RX_READPIN & (1 << LED_RX_PIN) ))       //pin high
#else
    if (!(LED_RX_READPIN & (1 << LED_RX_PIN) ))       //pin low
#endif
    {
        //set pin flag,
        detectedBB = 1;
    }
}

ISR(PCINT1_vect)
{
    //triggered when button pb2 is pushed.
}
