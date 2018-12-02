//#include <TimerThree.h>
//#include <frequency_counter_TC.h> //meh

#define DEBUG
#ifdef DEBUG
//#define DPRINT(args...)  Serial.print(args)             //OR use the following syntax:
#define DPRINTSTIMER(t)    for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print("\t");Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}//StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSTIMER(t)    if(false)
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

//#define M0 44
//#define M1 45
//#define M2 10
//#define M3 5

#define M0 45
#define M1 44
#define M2 5
#define M3 10

/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK (NOT REALLY, HOBBYKING GOES STRAIGHT TO CONFIGURATION MENU. @pjago)
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 *
 * @author lobodol <grobodol@gmail.com>
 * @author pjago <pedro.m4rtins@gmail.com>
 */
// ---------------------------------------------------------------------------

#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo E[4];
char data;
// ---------------------------------------------------------------------------

volatile unsigned int count = 0;
volatile unsigned long prev = 0;
const unsigned long bounce = 100; //5000 us is the smaller period for a 12k rpm rotation
void speed_count () {
    unsigned long now = micros();
    if (now - prev > bounce) {
      count++;
      prev = now;
    }
}
volatile unsigned int w = 0;
void read_speed () {
    w = count;
    count -= w;
}

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(9600);

    pinMode(47, INPUT);
    
//    Timer3.initialize(0);
//    Timer3.attachInterrupt(read_speed, 1000000); // 1 sec
//    attachInterrupt(digitalPinToInterrupt(19), speed_count, FALLING);

    E[0].attach(M0, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    E[1].attach(M1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    E[2].attach(M2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    E[3].attach(M3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    
    displayInstructions();
}

short mask = 0xF;
void esc_write(unsigned int us) {
    for (int i = 0; i<4; i++) {
        if (mask & (1 << i)) {
            E[i].writeMicroseconds(us);
        }
    }
}

/**
 * Main function
 */
void loop() {
    int a, b;
    if (Serial.available()) {
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      esc_write(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      esc_write(MAX_PULSE_LENGTH);
            break;

            // 2
            case 50 : Serial.print("Running test in 3");
                      delay(1000);
                      Serial.print(" 2");
                      delay(1000);
                      Serial.println(" 1...");
                      delay(1000);
                      test();
            break;

            //3
            case 51: Serial.println("Reading from A0...");
                     a = analogRead(A0);
                     b = map(a,0,1023,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
                     esc_write(b);
                     Serial.print("Pulse length = ");
                     Serial.println(b);
            break;
            
            //4
            case 52: Serial.println("Enter pulse length in % (0-100):");
                     while (Serial.available() <= 0);
                     a = Serial.parseInt();
                     b = constrain(map(a,0,100,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH),MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
                     esc_write(b);
                     Serial.print("Pulse length = ");
                     Serial.println(b);
            break;

            //5
            case 53: Serial.println("Enter bitmask in hexadecimal:");
                     while (Serial.available() <= 0);
                     String str = Serial.readString();
                     mask = strtol(str.c_str(), NULL, 16);
                     Serial.print("Mask =");
                     if (mask) {
                         for (int i = 3; i>=0; i--) {
                             if (mask & (1 << i)) {
                                 Serial.print(" E");
                                 Serial.print(i);
                             }
                         }
                     }
                     else {
                        Serial.print(" None");
                     }
                     Serial.println();
        }
    }

//    DPRINTSTIMER(1000) {
//      int a = analogRead(A0);
//      int b = map(a,0,1023,MIN_PULSE_LENGTH,MAX_PULSE_LENGTH);
//      Serial.print("\tA0:");
//      Serial.print(b);
//      Serial.print("\tExtISR:");
//      Serial.println(w);
//    }
}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= MAX_PULSE_LENGTH; i += 100) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        esc_write(i);
        delay(5000);
    }
    Serial.println("STOP");
    esc_write(MIN_PULSE_LENGTH);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : Send max throttle");
    Serial.println("\t2 : Run test function");
    Serial.println("\t3 : Read throttle from A0");
    Serial.println("\t4 : Read throttle from Serial");
    Serial.println("\t5 : Choose selection bitmask\n");
}
