#include <eRCaGuy_Timer2_Counter.h>
#include "HC_SR04.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps00.h"
#include "TimerThree.h"
#include "Wire.h"
#include <avr/wdt.h>
#include "avr/pgmspace.h"
#include "avr/io.h"
#include <ServoTimer1.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

#define DEBUG
#ifdef DEBUG
#define DPRINTSFN(StrSize,Name,...) {char S[StrSize];Serial.print(Name);Serial.print(" "); Serial.print(dtostrf((float)__VA_ARGS__ ,S));}
#define DPRINTLN(...)      Serial.println(__VA_ARGS__)
#else
#define DPRINTSFN(...)     //blank line
#define DPRINTLN(...)      //blank line
#endif

// new
// replace standard delay with busy wait
// freaking micros will overflow after 70 minutes... what a charm
#define delay(d) mdelay(d)
void mdelay(long d)
{
  long to = timer2.get_count();
  while(long dt = (timer2.get_count() - to) < 2000*d);
}

void debug(char op, int code) {
  Serial.print(op);
  Serial.print('(');
  Serial.print(code);
  Serial.print("):");
}

//PINS
#define LED_PIN 13
#define ECHO 3
#define TRIG 4
//#define M0 44
//#define M1 45
//#define M2 10
//#define M3 5
//#define M0 45
//#define M1 44
//#define M2 5
//#define M3 10

#define M1 11
#define M3 12

// PARTS
//ServoTimer1 E0, E1, E2, E3;
ServoTimer1 E1, E3;
HC_SR04 hcsr04(TRIG, ECHO, digitalPinToInterrupt(ECHO));
MPU6050 mpu;

//CONST
const long T = 40000; // 0.5 us sampling time IMPORTANT! DON'T FORGET TO DELAY TO THIS VALUE
const long Ts = 0; // this is the reference delay between MPU interrupt and sampling
const float Tf = T / 2000000.0; //float T in seconds
const float SERVO_DELAY = 20000.0; // us (hobby protocol)
const float DMP_DELAY = 4800.0; //us (42 Hz DLPF_CFG)
const float FIXED_DELAY = SERVO_DELAY + DMP_DELAY;
const float UOFF = 1000;  // esc off actuation
const float URNG = 1000;  // esc range, from off to maximum throttle
const float UMIN = 1100;  // esc minimum actuation (>= UOFF)
const float UMAX = 1900;  // esc maximum actuation (<= UOFF + URNG)
const float GIRO_MAX = 500; // MPU gyro is configured as 2000°/s, 500°/s will jump of 10° per frame
const float JUMP_MAX = 1500; // 1.5 m/s will JUMP 30 cm per frame
const int TRIAL = 100; // try sensors this many times before loop

Quaternion dslerp (Quaternion a, Quaternion b, float t) {
  Quaternion c = b.getProduct(a.getConjugate()).getNormalized();
  c.w = cos(acos(c.w) * t);
  return c;
}

void gpid (float* g, float kp, float ti, float td) {
  float i = 0.5 * Tf / ti;
  float d = td / Tf;
  g[0] = kp * (i + d + 1);
  g[1] = kp * (i - 2 * d - 1);
  g[2] = kp * d;
}

void gpdf (float* g, float kp, float tf, float td) {
  float f = exp(-Tf / td);
  float d = td / Tf;
  g[0] = kp * (d + 1);
  g[1] = -kp * (d + f);
}

typedef struct {
  bool state = HIGH;
  float delta = 0.0;
  float value = 0.0;
  float dmax, ddmax;
  unsigned short count = 0, wait;
  void init (float zero, float threshold, int duration) {
    value = zero;
    wait = duration;
    dmax = abs(Tf * threshold);
    ddmax = abs(2 * Tf * dmax);
  }
  float next (float x) {
    float dx = x - value;
    if ((abs(dx) < dmax /*&& abs(dx - delta) < ddmax*/) || count > wait) {
      count = 0;
      delta = dx;
      value = x;
    }
    else if (count == wait) {
      debug('c', (int) abs(dx));
      count++;
      delta = 0;
      value = x;
    }
    else {
      count++;
      value += delta;
    }
    return state == HIGH ? value : x;
  }
} Denoise;

#define FN 100 // comb filter memory size
#define FQ 8.0 // comb filter quality
typedef struct {
  unsigned int k, p, q;
  float a, r[FN], y[FN]; //todo: think about memory usage; smart trick with overflow
  void init (float _r, float _y, float Q) {
    for (k = 0; k < FN; k++) {
      r[k] = _r;
      y[k] = _y;
    }
    k = q = 0;
    a = 1 / cos(PI / (2 * Q)) + cos(PI / (2 * Q));
    a = a + sqrt(pow(a, 2) - 4);
    a = (a - sqrt(pow(a, 2) - 4)) / 2;
  }
  float next (float _y, unsigned int n) {
    n = constrain(n, 2, FN);
    if (n > k + 1) p = FN + k - n + 1;
    else p = k - n + 1;
    n = n >> 1;
    if (n > k + 1) q = FN + k - n + 1;
    else q = k - n + 1;
    float _r = -a * r[q] + (_y + y[q]) * (1 + a) / 2;
    if (k == FN - 1) k = 0;
    else k++;
    y[k] = _y;
    r[k] = _r;
    return (_r + r[q]) / 2; //complementary filter?
  }
} Filter;

typedef struct {
  float gr[3]; //transfer gains
  float gy[3]; //transfer gains
  float ku = 1.0; //transfer gain
  float kr = 1.0; //dc fix
  float r, y, e, u; //instant
  float pr, py, pe; //previous
  float ppr, ppy, ppe; //pprevious
  char mode = 0; // 0 PI+D , 1 PID
  void init (float Kp, float ti, float td) {
    r = y = e = u = pr = py = pe = ppr = ppy = ppe = 0.0;
    gpid(gr, Kp, ti, 0.0);
    gpid(gy, -Kp, ti, td);
    mode = 0;
    ku = 1.0;
    kr = 1.0;
  }
  float next (float rn, float yn) {
    rn = kr * rn;
    ppy = py;
    ppr = pr;
    ppe = pe;
    py = y;
    pr = r;
    pe = e;
    y = yn; //whatever
    r = rn;
    e = r - y;
    if (!mode) {
      u = ku * u + gy[0] * (y - r) + gy[1] * (py - pr) + gy[2] * (ppy - ppr); //PID
    }
    else {
      u = ku * u + gy[0] * y + gy[1] * py + gy[2] * ppy + gr[0] * r + gr[1] * pr + gr[2] * ppr; //PI+D
    }
    return u;
  }
} PID;

typedef struct {
  float ku; //transfer gain
  float ge[3]; //transfer gain
  float r, y, e, u; //instant
  float pr, py, pe; //previous
  float ppr, ppy, ppe; //pprevious
  void init (float Kp, float tf, float td) {
    r = y = e = u = pr = py = pe = ppr = ppy = ppe = 0.0;
    gpdf(ge, -1.0, tf, 0.0);
    ku = ge[1];
    gpdf(ge, Kp, tf, td);
  }
  float next (float rn, float yn) {
    ppy = py;
    ppr = pr;
    ppe = pe;
    py = y;
    pr = r;
    pe = e;
    y = yn; //whatever
    r = rn;
    e = r - y;
    return u = u * ku + ge[0] * e + ge[1] * pe;
  }
} PDF;

typedef struct {
  unsigned int n, up, saw[2], psaw[2];
  float d, eps;
  void init (float _d, float _eps) {
    n = 0xFFFF;
    d = _d;
    eps = _eps;
    up = saw[0] = saw[1] = psaw[0] = psaw[1] = 1;
  }
  float next (float r, float y) {
    float e = r - y;
    if (e >= eps && !up) { //todo: combine if and else if
      n = psaw[0] + psaw[1];
      if (saw[0] == psaw[0]) saw[1] = 1;
      up = 1;
    }
    else if (e <= -eps && up) {
      n = psaw[0] + psaw[1];
      if (saw[1] == psaw[1]) saw[0] = 1;
      up = 0;
    }
    else psaw[up] = saw[up] = saw[up] + 1;
    return up ? d : -d;
  }
} Relay;

//TIMERS
long t = 0, dt = T, dts = 0;
volatile long lag = 0, ts = 0;

// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// -4232  -706  1729  173 -94 37
// -2915,  -3519, 1771,  77,  15,  -35 (freze all but pitch)
// -3141, -3674, 1730, 77, 15, -34 (freze all but pitch)
// -2970, -3658, 1746,  74,  16,  -33 (freze all but pitch)
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = { -2882,  -3635, 1723,  69,  15,  -28}; //@calibrate

// MPU control/status vars
uint8_t devStatus;                   // return status after each device operation (0 = success, !0 = error)
uint8_t packetSize;                  // expected DMP packet size (default is 42 bytes)
volatile uint8_t packetCount;        // current packet count
uint8_t fifoBuffer[1024];            // FIFO storage buffer (let's use all that mega memory)
volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high (or it should be)
void dmpDataReady() {
  ts = timer2.get_count();
  packetCount++;
  mpuInterrupt = true; //both interrupts set this to true
}

void dmpReset() {
  mpu.resetFIFO();
  mpuInterrupt = false;
  packetCount = 0;
}

//@tcc afirmar que é possível tentar sincronizar o período de um loop com o DMP,
//porém requer grupos de períodos menores e maiores que o período de amostragem,
//no final optou-se por permanecer com um perído fixo, e previsível.
//UPDATE: consegui matar os dois coelhos com uma fireball só
void timerISR () {
  //lag = constrain(dt - T, 0, T);
  lag = constrain(dt - T + dts - Ts, 0, T);
  //lag = constrain(T - timer2.get_count() + t - dts, 0, T) >> 3;
}

void MPU6050Connect() {
  static int MPUInitCntr = 0;
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};
    MPUInitCntr++;
    Serial.print(F("MPU connection Try #"));
    Serial.println(MPUInitCntr);
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(StatStr[devStatus]);
    Serial.println(F(")"));
    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  Serial.print("mpu.getInterruptDrive=  "); Serial.println(mpu.getInterruptDrive());
  attachInterrupt(0, dmpDataReady, RISING); //pin 2 on the Uno
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReset();
}

// orientation/motion vars
Quaternion dq, q;       // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float R, P, Y, H; // initial values
float Yaw, Pitch, Roll, Height = 0.0;
Denoise Dh, Dypr[3]; // initiated at setup
PID Cx, Cy, Cz, Ch; // to control x, y, z angles, and height
PDF Cxp, Cyp, Czp, Chp; // lead compensator
Filter Fx, Fy, Fz, Fh; // comb filters
Relay Rx, Ry, Rz, Rh; // symetric relay

//this blocks sometimes https://github.com/jrowberg/i2cdevlib/issues/252
//update: hopefully it won't block now
void GetDMP() {
  noInterrupts();
  dt = timer2.get_count() - t;
  t += dt;
  dts = t - ts;
  uint8_t pc = packetCount;
  if (!pc) {
    dmpReset();
    interrupts();
    debug('d', 0);
    digitalWrite(LED_PIN, LOW);
  }
  else {
    mpuInterrupt = false;
    interrupts();
    Quaternion qf;
    debug('p', pc);
    float pr = 1.0 / (pc - 1.0);
    dq = Quaternion(1.0, 0.0, 0.0, 0.0);
    packetCount--;
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    for (uint8_t *next = fifoBuffer + packetSize, i = 1; i < pc; i++, next += packetSize) {
      packetCount--;
      mpu.getFIFOBytes(next, packetSize);
      mpu.dmpGetQuaternion(&qf, next);
      dq = dslerp(q, qf, pr).getProduct(dq); // Riemannian (Geometric Mean)
      q = qf;
    }
    // make a prediction of what the next q will be, and slerp to account for lag
    q = dslerp(q, dq.getProduct(q), (FIXED_DELAY + (float) 0.5*dts) / (0.5*dt)).getProduct(q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Yaw = Dypr[0].next(ypr[0] * 180.0 / M_PI);
    Pitch = Dypr[1].next(ypr[1] * 180.0 / M_PI);
    Roll = Dypr[2].next(ypr[2] * 180.0 / M_PI);
    if (uint8_t leftover = mpu.getFIFOCount() % packetSize) {
      dmpReset();
      debug('d', leftover);
    }
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  }
}

//@tcc comentar que o ruído do HCS04 é bem diferente do que o MPU, no sentido que
//ele possui um alto SNR, e portanto a minha implementação de Denoise não funciona bem
//i.e. não adianta confiar na taxa de variação quando o valor anterior não é confiável
void GetHCSR04 () {
  Height = 0.95 * Height + 0.05 * Dh.next(hcsr04.getRange());
  hcsr04.start();
}

// DONT PANIC assigns only the LSB. The HSB is assigned at loop.
unsigned int PANIC = 0;
int DONT_PANIC () {
  PANIC = PANIC | (abs(Roll) > 75) << 3 | (abs(Pitch) > 75) << 2 | (Height > 300) << 1;
  if (PANIC) debug('e', PANIC);
  return PANIC;
}

// maps from 0-100 to 0-URNG, offsets, then saturates
// the ServoTimer1 Library min and max resolution is (2400 - 544)/4 (ServoTimer1.h and ServoTimer1.cpp)
// the ServoTimer1 us to ticks is (( clockCyclesPerMicrosecond()* _us) / 8) (ServoTimer1.cpp)
// during relay tests, the values where (1330, 1370 us) (1370, 1310 us)
// (1356-1324) = 32 , (1304-1376) = -72 => d(us) = 52
// (0.2754-0.2386) = 0.0368, (0.2161-0.2989) = -0.0828 => d(%) = 2.30 | m(%) = 5.98 (23.888) (new unit)
// (0.2457-0.2129) = 0.0328, (0.1928-0.2666) = -0.0738 => d(%) = 2.05 | m(%) = 5.33 (25.292) (actual)
// (0.2226-0.1774) = 0.0452, (0.1674-0.2326) = -0.0652 => d(%) = 2.0  | m(%) = 5.52 (25.292) (expected)

int sat_esc (float u) {
  if (PANIC) return UOFF;        //24.828 25.292     //5.160 6.331
  else return constrain((int) (pow(23.669 * sqrt(u) + 6.331, 2) + UOFF), UMIN, UMAX);
}

int sat_esc2 (float u) {
  if (PANIC) return UOFF;        //23.669 26.172     //6.331 6.331
  else return constrain((int) (pow(24.828 * sqrt(u) + 6.331, 2) + UOFF), UMIN, UMAX);
}

int cmd = 1;
unsigned long k = 0;

float ry;
void setup() {
  //configure Timer2
  timer2.setup(); //this MUST be done before the other Timer2_Counter functions work; Note: since this messes up PWM outputs on pins 3 & 11, as well as 
                  //interferes with the tone() library (http://arduino.cc/en/reference/tone), you can always revert Timer2 back to normal by calling 
                  //timer2.unsetup()
  
  //new
  wdt_disable();
  cbi (TIMSK0, TOIE0); // disable Timer0 !!! delay() is now not available
//  TIMSK0=0; // this will disable serial https://stackoverflow.com/questions/29312239/disable-timer-0-interrupt-on-arduino-without-breaking-serial

  // HARDWARE
  pinMode(LED_PIN, OUTPUT);
//  E0.attach(M0, UOFF, UMAX);
  E1.attach(M1, UOFF, UMAX);
//  E2.attach(M2, UOFF, UMAX);
  E3.attach(M3, UOFF, UMAX);

  // SOFTWARE
  // init controllers kp, ti, td
  //Cy.init(1.918565970092061e-04, 1.083382527619075, 0.270845631904769); //zigler nichols (horrível!)
  Cy.init(0.001600474762772, 2.966380993602277, 0.741595248400569); //astron(Gjwc, 10*exp(-1j*pi/1.5), wc, 0.25)
  Cyp.init(0.001600474762772, 0.048669134062632, 0.230423872469489); //pjago
  //  Cx.init(0.001600474762772, 2.966380993602277, 0.741595248400569); //astron(Gjwc, 10*exp(-1j*pi/1.5), wc, 0.25)
  //  Cx.init(0.001300/2, 1.0921, 0.0); //nichols(-3.4626, 4.7943, 'PI')
  //  Cx.init(0.001064, 1.9586, 0.4896); //astron(Gjw, 2*exp(-j*pi/2), w, 0.25); % relay 1 2
  //  Cx.init(0.002041, 1.9586, 0.4896); //astron(Gjw, abs(Gjw)*exp(-j*pi/2), w, 0.25); % relay 1 2
  //  Cx.init(0.002041, 1.0203, 0.5101); //astron(Gjw, abs(Gjw)*exp(-j*pi/2), w, 0.5) % relay 1 2
  //  Cx.init(0.002641, 0.2978, 0.2978); //astron(Gjw, 2*exp(-j*pi/1.5), w, 1.0) % relay 1 2
  //  Cx.init(0.0025, 0.5, 0.5); //astron(Gjw, 4.244*exp(-j*pi/1.9404), w, 1) % relay 1 2
  //  Cx.init(0.0025, 0.4, 0.5); //ajustes finais
  //  Cx.init(0.00042127016722639, 0.081434457024230, 0.583359199932275); // lshape(Gs, 3.2) %damping: 0.4
  //  Cx.init(0.00014012596198007, 0.081434457024230, 0.583359199932275); // lshape(Gs, 3.4) %damping: 0.7
  //  Cx.init(0.00010602393982948, 0.081434457024230, 0.583359199932275); // lshape(Gs, 3.55) %damping: 0.8
  //  Cx.init(0.00008491879449271, 0.081434457024230, 0.583359199932275); // lshape(Gs, 3.8) %damping: 0.9
  //  Cx.init(0.00006908770109760, 0.081434457024230, 0.583359199932275); // lshape(Gs) %damping: 1.0
  //  Cx.init(0.00005701826543864, 0.081434457024230, 0.583359199932275); // lshape(Gs, 6.1) %damping: 1.1
  //  Cxp.init(0.001600474762772, 0.048669134062632, 0.230423872469489); //pjago

  //Cx.init(2.5e-3, 0.5, 0.5); //astron(Gjw, 4.244*exp(-j*pi/1.9404), w, 1)
  Cx.init(1.401040111194481e-04, 0.081425178147269, 0.583430571761959); Cx.ku = exp(-2.5 * Tf); Cx.kr = 2.0 * 2.8675; //PID com pólo adiantado, frequência 2.5 rad/s
  //Cx.init(1.401040111194481e-04, 0.081425178147269, 0.583430571761959); //lshape(Gs, 3.4) %damping: 0.7 ITAE best eq 2°
  //Cx.init(1.812525511652470e-05, 0.011638991414821, 3.059129804465807); //pplace(G2, cs, 'PID') %ITAE best eq 3°

  Cz.init(0, 100, 0);
  Ch.init(0, 100, 0);
  // init relays
  Rx.init(0.01, 1);
  Ry.init(0.01, 0);
  Rz.init(0.01, 0);
  Rh.init(0.01, 0); //todo: offset

  // SENSORS
  Serial.begin(115200);
  while (!Serial);
  Serial.println("i2cSetup");
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz);
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.print("Wait.");
  delay(2000);
  Serial.print(".");
  delay(2000);
  Serial.print(".");
  delay(1000);
  Serial.println();
  dmpReset();
  Serial.println("Try MPU");
  float sR = 0.0, sP = 0.0, sY = 0.0;
  for (int i = 0; i < TRIAL; i++) {
    while (!mpuInterrupt);
    GetDMP();
    sY += Yaw;
    sP += Pitch;
    sR += Roll;
  }
  Serial.println();
  Serial.println("Try HCSR04");
  hcsr04.begin();
  float sH = 0.0;
//  for (int i = 0; i < TRIAL; i++) {
//    hcsr04.start();
//    while (!hcsr04.isFinished());
//    delay(10);
//    Height = (float) hcsr04.getRange();
//    sH += Height;
//  }
  Y = sY / TRIAL;
  P = sP / TRIAL;
  R = sR / TRIAL;
  H = 0.5;
  Serial.println("Initial Values: ");
  DPRINTSFN(15, "\tz:", Y, 6, 1);
  DPRINTSFN(15, "\ty:", Pitch, 6, 1);
  DPRINTSFN(15, "\tx:", Roll, 6, 1);
  DPRINTSFN(15, "\th:", H, 1, 6);
  DPRINTLN();

  // init denoisers (values are for T = 0.02s, giro sensitivity 250°/s)
  Dypr[0].init(Y, GIRO_MAX, 3);
  Dypr[1].init(P, GIRO_MAX, 3);
  Dypr[2].init(R, GIRO_MAX, 3);
  Dh.init(H, JUMP_MAX, 0);

  // init filters
  Fx.init(R, R, FQ);
  ry = 0.0;

  // MOTORS
  Serial.println("Will start ESCs (Be careful!)");
//  E0.writeMicroseconds(UOFF);
  E1.writeMicroseconds(UOFF);
//  E2.writeMicroseconds(UOFF);
  E3.writeMicroseconds(UOFF);
  Serial.println("Turn on battery then confirm: ");
  while (!Serial.available()){};
  cmd = Serial.read();
  cmd = cmd - 48;
  if (cmd >= 30 && cmd < 40) {
    Rx.init(0.01, cmd - 30);
    cmd = 3;
  }
//  Serial.println(cmd);
  t = timer2.get_count();
//  Serial.println(T);
//  Serial.println(timer2.get_count() - t);
//  Serial.println(timer2.get_count() - t);
//  Serial.println(timer2.get_count() - t);      
//  while (timer2.get_count() - t < T){};
//  Serial.println(timer2.get_count() - t - T);
  
  //TIMERS
  dmpReset();
  while (!mpuInterrupt);
  Timer3.restart();
  Timer3.attachInterrupt(timerISR, T);
  t = timer2.get_count();
}

float rk[10] = {0.0, 10.0, 0.0, -10.0, -10.0, 10.0, 10.0, -10.0, -10.0, 0.0};
void loop() {
  // READ
  while (timer2.get_count() - t < (T - lag));
  if (mpuInterrupt) GetDMP(); // reference time
  else PANIC |= 0x40;
//  if (hcsr04.isFinished()) GetHCSR04(); //@todo: interpolation
//  else PANIC |= 0x20;
//  PANIC &= 0xDD; //@relay disable hcsr04

  if (Serial.available() > 0) {
    cmd = Serial.read();
    cmd = cmd - 48;
    if (cmd >= 30 && cmd < 40) {
      Rx.init(0.01, cmd - 30);
      cmd = 3;
    }
  }

  DONT_PANIC(); //in large friendly letters

  k++;
  unsigned long ki = (k % 5001) / 500;
  float y = Pitch;
  if (cmd == 1) ry = rk[ki]; // filtro de 1° ordem
  if (cmd == 2) ry = 0.0;
  if (cmd == 3) ry = Fx.next(y, Rx.n);
  if (cmd == 4) ry = 10.0;

  // CTRL
  //float ux = Cx.next(ry, Roll); //x is aligned with the usb port
  float ux = 0.0;
  float uy = 0.0; //Cy.next(ry, Pitch);
  float uz = 0.0; //Cz.next(Y, Yaw);
  float uh = 0.8; //1.7804; //Ch.next(H, Height);

  if (cmd == 2) ux += Rx.next(ry, y);
  if (cmd == 3) ux += Rx.next(ry, y);
  if (cmd == 5) ux = 0.04;
  if (cmd == 6) ux = -0.04;
  if (cmd == 7) ux = 0.00;
  if (cmd == 0) {
    ux = 0.0;
    uh = 0.0;
    PANIC = 0;
  }
  else {
    PANIC = PANIC | !!PANIC;
  }
//  if (cmd == 9) {
//    ux = 0.0;
//    uh = 4.0;
//    PANIC &= 0xFE;
//  }

  //actuate
  float u0 = uh / 4 + uy / 2 + uz / 4;
  float u1 = uh / 4 - ux / 2 - uz / 4;
  float u2 = uh / 4 - uy / 2 + uz / 4;
  float u3 = uh / 4 + ux / 2 - uz / 4;
  //  E0.writeMicroseconds(sat_esc(u0));
  //  E1.writeMicroseconds(UOFF);
  //  E2.writeMicroseconds(sat_esc(u2));
  //  E3.writeMicroseconds(UOFF);
//  E0.writeMicroseconds(UOFF);
  E1.writeMicroseconds(sat_esc(u1));
//  E2.writeMicroseconds(UOFF);
  E3.writeMicroseconds(sat_esc2(u3));

  // PRINT (todo: send bytes instead? but this won't be here anyway)
  //profiler
  debug('n', Rx.n);
  debug('s', dts/2);
  debug('t', dt/2);
  DPRINTLN();
  //@tcc comentar que a comunicação serial é um overhead durante development,
  //e que se optou omitir o quaternion porque ao se processar 3 packets, //update: turns out we have time
  //ultrapassava-se em aproximadamente 10% um período de amostrogem.
  //update: slowing a bit seems to be the only way to prevent DMP hang
  //basically the code hangs, if we ask for packets when they're not ready
  //transform
  DPRINTSFN(15, "", q.w, 1, 6);
  DPRINTSFN(15, "", q.x, 1, 6);
  DPRINTSFN(15, "", q.y, 1, 6);
  DPRINTSFN(15, "", q.z, 1, 6);
  //  DPRINTLN();
  DPRINTSFN(15, "", Yaw, 4, 3);
  DPRINTSFN(15, "", Pitch, 4, 3);
  DPRINTSFN(15, "", Roll, 4, 3);
  DPRINTSFN(15, "", ry, 4, 3); // TEMP @relay
  //  DPRINTSFN(15, " h:", Height, 4, 3);
  //  DPRINTLN();
  //actuation
  DPRINTSFN(15, "", u0, 1, 6);
  DPRINTSFN(15, "", u1, 1, 6);
  DPRINTSFN(15, "", u2, 1, 6);
  DPRINTSFN(15, "", u3, 1, 6);
  DPRINTLN();

}
