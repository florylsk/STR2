/**********************************************************
 *  INCLUDES
 *********************************************************/
#include <Wire.h>
#include "let_it_be_1bit.h"
/**********************************************************
 *  CONSTANTS
 *********************************************************/

// UNCOMMENT THIS LINE TO EXECUTE WITH RASPBERRY PI
//#define RASPBERRYPI 1

#define SLAVE_ADDR 0x8

#define SAMPLE_TIME 250
#define SWITCH_PIN  7
#define LED_PIN 13
#define BUF_SIZE 256
#define MUSIC_LEN 20480
#define SOUND_PIN 11
/**********************************************************
 *  GLOBALS
 *********************************************************/

unsigned char buffer[BUF_SIZE];
unsigned long timeOrig;
unsigned int RepMode=0; // 0=muted, 1 = sound
unsigned long SYSTEM_CLOCK = 16000000;
unsigned int preScaling=1;


/**********************************************************
 * Function: receiveEvent
 *********************************************************/
void receiveEvent(int num)
{
    static int count = BUF_SIZE/2;
    for (int j=0; j<num; j++) {
        buffer[count] = Wire.read();
        count = (count + 1) % BUF_SIZE;
    }
}

// --------------------------------------
// Handler function: requestEvent
// --------------------------------------
void requestEvent()
{
}

/**********************************************************
 * Function: play_bit
 *********************************************************/
void play_bit()
{
    static int bitwise = 1;
    static unsigned char data = 0;
    static int music_count = 0;

    bitwise = (bitwise * 2);

    //meter algoritmo aqui?
    if (bitwise > 128) {
        bitwise = 1;
#ifdef RASPBERRYPI
        data = buffer[music_count];
        music_count = (music_count + 1) % BUF_SIZE;
#else
        data = pgm_read_byte_near(music + music_count);
        music_count = (music_count + 1) % MUSIC_LEN;
#endif
    }
    if (RepMode==0) data=0;
    digitalWrite(SOUND_PIN, (data & bitwise));

}

/**********************************************************
 * Function: readSound
 *********************************************************/

void showSound(){
    int tmp = digitalRead(SWITCH_PIN);
    if (tmp==HIGH){
        if (RepMode==1)RepMode=0;
        else RepMode=1;
    }
    if (RepMode==0) digitalWrite(LED_PIN,HIGH);
    else digitalWrite(LED_PIN,LOW);
}


ISR(TIMER1_COMPA_vect) {
        play_bit();

}
/**********************************************************
 * Function: setup
 *********************************************************/
void setup ()
{
    // Initialize I2C communications as Slave
    Wire.begin(SLAVE_ADDR);
    // Function to run when data requested from master
    Wire.onRequest(requestEvent);
    // Function to run when data received from master
    Wire.onReceive(receiveEvent);
    // set I2C speed to 1 MHz
    Wire.setClock(1000000L);
    pinMode(SOUND_PIN, OUTPUT);
    pinMode(LED_PIN,OUTPUT);
    pinMode(SWITCH_PIN,INPUT);
    memset (buffer, 0, BUF_SIZE);
    timeOrig = micros();
    TCCR1B = _BV(WGM12) | _BV(CS10);
    TCCR1A = 0;
    TIMSK1 = _BV(OCIE1A);
    OCR1A=SYSTEM_CLOCK/(preScaling*4000);

}



/**********************************************************
 * Function: loop
 *********************************************************/
void loop ()
{
    unsigned long timeDiff;
    Serial.println(RepMode);
    receiveEvent(128);
    showSound();
    timeDiff = SAMPLE_TIME - (micros() - timeOrig);
    timeOrig = timeOrig + SAMPLE_TIME;
    delayMicroseconds(timeDiff);
}