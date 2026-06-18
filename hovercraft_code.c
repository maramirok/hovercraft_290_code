/*
  Hovercraft Control: "The Navigator" (PURE C VERSION)
  TURN UPGRADE v14: Left-turn ANTI-STALL fix. Full-lock vane (1750) was stalling on left turns
  -> thrust deflected backward -> craft slid back instead of rotating. Left turn now uses a
  gentler throw (SERVO_LEFT_TURN_US), floats higher (FAN_LIFT_TURN_LEFT raised), and no longer
  pins the vane through the turn (CTR_STEER_LEFT = 0). Scan still uses full throw to aim the
  sensor. INT0 ultrasonic + SCAN_SIDES stuck-state fix retained.
*/

#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// ---------- HARDWARE MAPPING (ATmega328P) ----------
#define FAN_LIFT_PIN   PD5
#define FAN_DIR_PIN    PD6
#define LED_PIN        PB5
#define SERVO_PIN      PB1

const int SERVO_CENTER_US = 3150;
const int SERVO_LEFT_US   = 1750;   // full-left throw (used by the SCAN to aim the sensor sideways)
const int SERVO_RIGHT_US  = 4500;

// Left TURN uses a gentler throw than the scan, so the vane vectors thrust SIDEWAYS (rotation)
// without STALLING. Full lock (1750) stalls -> thrust goes backward -> craft slides back.
const int SERVO_LEFT_TURN_US = 2000;  // RAISE toward 2700 if it still slides; LOWER toward 1900
                                      //   if it rotates too weakly to beat the counter-torque.

volatile int servoPulseWidth = SERVO_CENTER_US;
int targetPulse = SERVO_CENTER_US;
const int SERVO_SPEED = 150;

volatile uint32_t timer0_millis = 0;

#define MPU6050_ADDR 0x68
const int FAN_DIR_FULL = 255;
const int FAN_LIFT_90  = 230;   // normal cruise lift
const int FAN_STOP = 0;

// ===== turn tuning knobs =====
const int FAN_LIFT_TURN      = 50;   // deflate for RIGHT spin + scan. If it still slides: 30, then 0.
const int FAN_LIFT_TURN_LEFT = 60;   // raised from 20: float enough to PIVOT. Too low let it slide.
const uint32_t SCAN_SETTLE_MS = 200; // settle after sensor arrives at each side before measuring
const uint32_t SCAN_MAX_MS    = 1500;// HARD timeout per scan phase: never hang waiting to "arrive"

// --- two-stage wall trigger (breaks the early-turn vs ramming deadlock) ---
const int32_t SLOW_CM      = 35;     // notice the wall here -> slow approach (early warning)
const int32_t TURN_CM      = 18;     // commit to scan/turn here (close to the wall)
const int     FAN_DIR_SLOW = 150;    // reduced thrust during slow approach AND straighten

// --- post-turn straighten (settle heading before detecting again) ---
const uint32_t STRAIGHTEN_MAX_MS = 500; // safety timeout so it can't hang if heading won't fully settle

// --- spin thrust (LEVER 3) ---
const int FAN_DIR_TURN_RIGHT = 255;  // full thrust during right spin
const int FAN_DIR_TURN_LEFT  = 255;  // structural only -- left already at the 255 ceiling.

// --- RIGHT turn (torque assists): brake harder, lead more ---
const int32_t TURN_LEAD      = 5000;
const int     CTR_STEER_US   = 1800;
// --- LEFT turn (torque fights the spin) ---
const int32_t TURN_LEAD_LEFT = 500;
const int     CTR_STEER_LEFT = 0;    // was -800: stop pinning the vane at full throw through the turn.
// =============================

int32_t yaw = 0;
int32_t gyroErrorZ = 0;
uint32_t previousTime = 0, currentTime = 0;
uint32_t settleStart = 0;
uint32_t straightenStart = 0;
uint32_t scanStart = 0;

const int32_t Kp = 30;

// scan state
int scanPhase = 0;
uint8_t scanArmed = 0;
int32_t leftOpen = 0, rightOpen = 0;

// ================= Ultrasonic via INT0 (PD2), non-blocking timing =================
volatile uint32_t timer0_overflows = 0;
volatile uint8_t  echoState = 0;           // 0=idle, 1=waiting for rising, 2=timing high pulse
volatile uint32_t echoStartUs = 0;
volatile uint32_t echoWidthUs = 0;
volatile uint8_t  echoReady = 0;

// At 16MHz with /64 prescale: 1 tick = 4us, 256 ticks = 1024us per overflow.
static inline uint32_t micros_now(void) {
    uint32_t ov;
    uint8_t  t;
    uint8_t  oldSREG = SREG;
    cli();
    ov = timer0_overflows;
    t  = TCNT0;
    if ((TIFR0 & (1 << TOV0)) && t < 255) ov++;
    SREG = oldSREG;
    return (ov * 1024UL) + ((uint32_t)t * 4UL);
}

ISR(TIMER0_OVF_vect) {
    timer0_millis++;        // ticks every ~1.024ms with the /64 prescale
    timer0_overflows++;
}

// INT0 = PD2 = echo. Fires on every logic change (rising AND falling edge).
ISR(INT0_vect) {
    uint32_t now = micros_now();
    if (PIND & (1 << PD2)) {        // rising edge: echo started
        echoStartUs = now;
        echoState = 2;
    } else {                         // falling edge: echo ended
        if (echoState == 2) {
            echoWidthUs = now - echoStartUs;
            echoReady = 1;
            echoState = 0;
        }
    }
}

uint32_t get_millis() {
    uint32_t m;
    uint8_t oldSREG = SREG;
    cli();
    m = timer0_millis;
    SREG = oldSREG;
    return m;
}

void trig_pulse(void) {
    PORTB |= (1 << PB3);
    _delay_us(10);
    PORTB &= ~(1 << PB3);
}

// Fire a ping, then poll the ISR's "ready" flag (bounded). Echo TIMING is done in the ISR;
// this just collects the finished result. /58 -> cm scale identical to the old code.
int32_t us_sensor_get_distance(void) {
    echoReady = 0;
    echoState = 1;
    trig_pulse();

    uint16_t guard = 0;
    while (!echoReady && guard < 30000) {
        _delay_us(1);
        guard++;
    }
    if (!echoReady) { echoState = 0; return 0; }   // no echo -> open / out of range

    uint32_t w = echoWidthUs;
    int32_t d = (int32_t)(w / 58UL);
    return d;
}
// ==================================================================================

void UART_init(unsigned int ubrr) {
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;
  UCSR0B = (1 << TXEN0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}
void UART_TxChar(char ch) { while (!(UCSR0A & (1 << UDRE0))); UDR0 = ch; }

void I2C_init() { TWSR = 0; TWBR = 0x48; TWCR = (1 << TWEN); }
void I2C_start() { TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
void I2C_stop() { TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN); }
void I2C_write(uint8_t d){ TWDR=d; TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); }
uint8_t I2C_read_nack(){ TWCR=(1<<TWINT)|(1<<TWEN); while(!(TWCR&(1<<TWINT))); return TWDR; }

void MPU6050_write(uint8_t r, uint8_t d){
  I2C_start(); I2C_write(MPU6050_ADDR<<1); I2C_write(r); I2C_write(d); I2C_stop();
}
uint8_t MPU6050_read(uint8_t r){
  uint8_t d; I2C_start(); I2C_write(MPU6050_ADDR<<1); I2C_write(r);
  I2C_start(); I2C_write((MPU6050_ADDR<<1)|1); d = I2C_read_nack(); I2C_stop(); return d;
}
void MPU6050_init(){ MPU6050_write(0x6B,0x00); }

int16_t read_gyro_z_raw() {
  int16_t z_raw = (MPU6050_read(0x47) << 8) | MPU6050_read(0x48);
  return z_raw * -1;
}

void calculate_IMU_error() {
  int32_t raw_sum = 0;
  for(int i=0; i<200; i++) {
    raw_sum += read_gyro_z_raw();
    if(i % 10 == 0) PORTB ^= (1 << PB5);
    _delay_ms(10);
  }
  int32_t raw_avg = raw_sum / 200;
  gyroErrorZ = (raw_avg * 100) / 131;
  PORTB &= ~(1 << PB5);
}

// "How open is this side." 0 (no echo) = open, so map it to a large value.
// Median of 3 rejects single bad pings.
int32_t read_open(void) {
    int32_t a = us_sensor_get_distance(); _delay_ms(20);
    int32_t b = us_sensor_get_distance(); _delay_ms(20);
    int32_t c = us_sensor_get_distance();
    if (a == 0) a = 999;
    if (b == 0) b = 999;
    if (c == 0) c = 999;
    int32_t hi = (a > b) ? a : b; hi = (hi > c) ? hi : c;
    int32_t lo = (a < b) ? a : b; lo = (lo < c) ? lo : c;
    return a + b + c - hi - lo;   // median
}

ISR(TIMER1_COMPA_vect){ OCR1A = servoPulseWidth; }

// ---------- States ----------
enum SystemState {
  CRUISE       = 0,
  APPROACH     = 1,
  SCAN_SIDES   = 2,
  EXECUTE_TURN = 3,
  STRAIGHTEN   = 4
};
SystemState systemState = CRUISE;

int turnDirection = 1;     // +1 = right, -1 = left
int32_t targetYaw = 0;

int main(void) {
    I2C_init();
    MPU6050_init();

    DDRB |= (1 << PB3);   // trig output
    DDRD &= ~(1 << PD2);  // echo input (INT0)
    DDRB |= (1 << PB5);
    DDRD |= (1 << FAN_LIFT_PIN) | (1 << FAN_DIR_PIN);

    // INT0 (PD2) on ANY logic change: catches both echo edges.
    EICRA = (1 << ISC00);   // ISC01:ISC00 = 01 -> any edge
    EIMSK = (1 << INT0);

    // Timer0: Fast PWM for fans + overflow ISR for millis & us-clock (/64 prescale).
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << TOIE0);

    // Timer1: 16-bit Fast PWM for the servo (unchanged).
    DDRB |= (1<<DDB1);
    TCCR1A=0; TCCR1B=0; TCNT1=0;
    ICR1=20000;
    OCR1A=SERVO_CENTER_US;
    targetPulse = SERVO_CENTER_US;
    TCCR1A=(1<<COM1A1)|(1<<WGM11);
    TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
    TIMSK1|=(1<<OCIE1A);

    UART_init(103);
    sei();

    calculate_IMU_error();

    // LAUNCH SEQUENCE (lift first, then thrust)
    OCR0A = FAN_STOP; OCR0B = FAN_STOP; _delay_ms(3000);
    OCR0B = FAN_LIFT_90; OCR0A = FAN_STOP; _delay_ms(1500);   // pressurize
    OCR0A = FAN_DIR_FULL;                                     // launch

    previousTime = get_millis();
    currentTime = previousTime;

    while(1) {
        int16_t gz_raw = read_gyro_z_raw();
        int32_t gz_scaled = ((int32_t)gz_raw * 100) / 131;

        previousTime = currentTime;
        currentTime = get_millis();
        uint32_t delta_t = (currentTime >= previousTime) ? (currentTime - previousTime) : 0;

        yaw += ((gz_scaled - gyroErrorZ) * (int32_t)delta_t) / 1000;

        int32_t frontDist = us_sensor_get_distance();

        switch(systemState){

            case CRUISE: {
                OCR0A = FAN_DIR_FULL;
                OCR0B = FAN_LIFT_90;

                int32_t error = targetYaw - yaw;
                int correction = (int)((error * Kp) / 100);
                int newPulse = SERVO_CENTER_US + correction;
                if (newPulse < SERVO_LEFT_US)  newPulse = SERVO_LEFT_US;
                if (newPulse > SERVO_RIGHT_US) newPulse = SERVO_RIGHT_US;
                targetPulse = newPulse;

                static int32_t lastGoodDist = 999;
                if (frontDist > 0) lastGoodDist = frontDist;

                uint8_t wallNear =
                    (frontDist > 0 && frontDist < SLOW_CM) ||
                    (frontDist == 0 && lastGoodDist < SLOW_CM);

                if (abs(servoPulseWidth - SERVO_CENTER_US) < 200 && wallNear) {
                    lastGoodDist = 999;
                    systemState = APPROACH;
                }
                break;
            }

            case APPROACH: {
                OCR0A = FAN_DIR_SLOW;
                OCR0B = FAN_LIFT_90;

                int32_t error = targetYaw - yaw;
                int correction = (int)((error * Kp) / 100);
                int newPulse = SERVO_CENTER_US + correction;
                if (newPulse < SERVO_LEFT_US)  newPulse = SERVO_LEFT_US;
                if (newPulse > SERVO_RIGHT_US) newPulse = SERVO_RIGHT_US;
                targetPulse = newPulse;

                static int32_t lastGoodDist2 = 999;
                if (frontDist > 0) lastGoodDist2 = frontDist;

                uint8_t atWall =
                    (frontDist > 0 && frontDist < TURN_CM) ||
                    (frontDist == 0 && lastGoodDist2 < TURN_CM);

                if (atWall) {
                    lastGoodDist2 = 999;
                    scanPhase = 0;
                    scanArmed = 0;
                    scanStart = get_millis();
                    systemState = SCAN_SIDES;
                }
                break;
            }

            case SCAN_SIDES:
                OCR0A = FAN_STOP;
                OCR0B = FAN_LIFT_TURN;

                if (scanPhase == 0) {
                    targetPulse = SERVO_LEFT_US;          // full throw: aim sensor fully sideways
                    if (abs(servoPulseWidth - targetPulse) <= SERVO_SPEED ||
                        (get_millis() - scanStart) >= SCAN_MAX_MS) {
                        if (!scanArmed) { settleStart = get_millis(); scanArmed = 1; }
                        else if (get_millis() - settleStart >= SCAN_SETTLE_MS) {
                            leftOpen = read_open();
                            scanPhase = 1; scanArmed = 0;
                            scanStart = get_millis();
                        }
                    }
                } else { // scanPhase == 1
                    targetPulse = SERVO_RIGHT_US;         // full throw: aim sensor fully sideways
                    if (abs(servoPulseWidth - targetPulse) <= SERVO_SPEED ||
                        (get_millis() - scanStart) >= SCAN_MAX_MS) {
                        if (!scanArmed) { settleStart = get_millis(); scanArmed = 1; }
                        else if (get_millis() - settleStart >= SCAN_SETTLE_MS) {
                            rightOpen = read_open();

                            if (leftOpen > rightOpen) turnDirection = -1;  // LEFT
                            else                      turnDirection = 1;   // RIGHT (tie default)

                            targetYaw += (9000 * turnDirection);   // 9000 = 90.00 degrees
                            targetPulse = SERVO_CENTER_US;
                            scanArmed = 0;
                            systemState = EXECUTE_TURN;
                        }
                    }
                }
                break;

            case EXECUTE_TURN:
                OCR0A = (turnDirection == 1) ? FAN_DIR_TURN_RIGHT : FAN_DIR_TURN_LEFT;
                OCR0B = (turnDirection == 1) ? FAN_LIFT_TURN : FAN_LIFT_TURN_LEFT;

                if (turnDirection == 1) {                          // RIGHT: torque assists
                    if (yaw < targetYaw - TURN_LEAD)
                        targetPulse = SERVO_RIGHT_US;
                    else
                        targetPulse = SERVO_CENTER_US - CTR_STEER_US;

                    if (yaw >= targetYaw) {
                        OCR0B = FAN_LIFT_90;
                        straightenStart = get_millis();
                        systemState = STRAIGHTEN;
                    }
                } else {                                           // LEFT: torque fights us
                    // gentler throw so the vane vectors thrust sideways instead of stalling.
                    if (yaw > targetYaw + TURN_LEAD_LEFT)
                        targetPulse = SERVO_LEFT_TURN_US;
                    else
                        targetPulse = SERVO_CENTER_US + CTR_STEER_LEFT; // 0 -> center near the end

                    if (yaw <= targetYaw) {
                        OCR0B = FAN_LIFT_90;
                        straightenStart = get_millis();
                        systemState = STRAIGHTEN;
                    }
                }
                break;

            case STRAIGHTEN: {
                OCR0A = FAN_DIR_SLOW;
                OCR0B = FAN_LIFT_90;

                int32_t error = targetYaw - yaw;
                int correction = (int)((error * Kp) / 100);
                int newPulse = SERVO_CENTER_US + correction;
                if (newPulse < SERVO_LEFT_US)  newPulse = SERVO_LEFT_US;
                if (newPulse > SERVO_RIGHT_US) newPulse = SERVO_RIGHT_US;
                targetPulse = newPulse;

                if ((abs(servoPulseWidth - SERVO_CENTER_US) < 200) ||
                    (get_millis() - straightenStart >= STRAIGHTEN_MAX_MS)) {
                    systemState = CRUISE;
                }
                break;
            }
        }

        int diff = targetPulse - servoPulseWidth;
        if (abs(diff) > SERVO_SPEED) {
            if (diff > 0) servoPulseWidth += SERVO_SPEED;
            else          servoPulseWidth -= SERVO_SPEED;
        } else {
            servoPulseWidth = targetPulse;
        }
        OCR1A = servoPulseWidth;

        _delay_ms(40);
    }
    return 0;
}
