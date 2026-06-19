/*
  Hovercraft Control: "The Navigator" (PURE C VERSION)
  TURN UPGRADE v19:
    #1 PD TURN CONTROLLER -- replaces bang-bang+fixed-brake. Drives on remaining angle
       (targetYaw - yaw) and brakes on rotation RATE (gyro). The D term eases off as it
       spins toward target, so it approaches 90deg instead of overshooting.
    #2 STUCK-AT-WALL ESCAPE -- craft was pinning itself against a wall, rudder STRAIGHT,
       while the front reading failed to trigger (point-blank 0 / no change). Now if the
       front distance stops changing while close for too long, force the scan/turn.
  90deg symmetric turns, servo mirror, PI straight-line steering, INT0 ultrasonic retained.
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

// Logical-frame servo pulses. Servo physically rotated 180deg; output mirrored about center.
const int SERVO_CENTER_US = 3150;
const int SERVO_LEFT_US   = 1750;
const int SERVO_RIGHT_US  = 4500;

volatile int servoPulseWidth = SERVO_CENTER_US;
int targetPulse = SERVO_CENTER_US;
const int SERVO_SPEED = 150;

volatile uint32_t timer0_millis = 0;

#define MPU6050_ADDR 0x68
const int FAN_DIR_FULL = 255;
const int FAN_LIFT_90  = 230;
const int FAN_STOP = 0;

// ===== turn tuning knobs =====
const int FAN_LIFT_TURN      = 50;
const uint32_t SCAN_SETTLE_MS = 200;
const uint32_t SCAN_MAX_MS    = 1500;
const uint32_t TURN_MAX_MS    = 3000;  // escape hatch for the turn itself

// --- two-stage wall trigger ---
const int32_t SLOW_CM      = 45;
const int32_t TURN_CM      = 28;
const int     FAN_DIR_SLOW = 110;

// --- #2 STUCK-AT-WALL escape ---
const int32_t STUCK_CM      = 32;    // "close to a wall" band for the stuck-check
const int32_t STUCK_DELTA   = 3;     // if front distance changes less than this (cm)...
const uint32_t STUCK_MS     = 900;   // ...for this long while close, force the turn

// --- post-turn straighten ---
const uint32_t STRAIGHTEN_MAX_MS = 500;

// --- #1 PD TURN CONTROLLER gains ---
const int32_t Kp_turn = 18;    // drive strength on remaining angle. Higher = harder/faster turn.
const int32_t Kd_turn = 90;    // brake on rotation RATE. THIS kills overshoot. Raise if it still
                               //   overshoots; lower if it stalls/creeps before reaching target.
const int32_t TURN_DONE = 300; // within this many (x100) deg of target = turn complete (3.00 deg).

// --- steering gains (straight-line PI) ---
const int32_t Kp = 30;
const int32_t Ki          = 2;
const int32_t I_CLAMP     = 8000;
const int     I_OUT_CLAMP = 600;
// =============================

int32_t yaw = 0;
int32_t gyroErrorZ = 0;
uint32_t previousTime = 0, currentTime = 0;
uint32_t settleStart = 0;
uint32_t straightenStart = 0;
uint32_t scanStart = 0;
uint32_t turnStart = 0;

int32_t yawIntegral = 0;

// scan state
int scanPhase = 0;
uint8_t scanArmed = 0;
int32_t leftOpen = 0, rightOpen = 0;

// stuck-detect state
uint32_t stuckStart = 0;
int32_t  stuckRefDist = 0;
uint8_t  stuckArmed = 0;

// most-recent gyro rate, shared into EXECUTE_TURN for the D term
int32_t gz_scaled_global = 0;

// ================= Ultrasonic via INT0 (PD2), non-blocking timing =================
volatile uint32_t timer0_overflows = 0;
volatile uint8_t  echoState = 0;
volatile uint32_t echoStartUs = 0;
volatile uint32_t echoWidthUs = 0;
volatile uint8_t  echoReady = 0;

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
    timer0_millis++;
    timer0_overflows++;
}

ISR(INT0_vect) {
    uint32_t now = micros_now();
    if (PIND & (1 << PD2)) {
        echoStartUs = now;
        echoState = 2;
    } else {
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

int32_t us_sensor_get_distance(void) {
    echoReady = 0;
    echoState = 1;
    trig_pulse();

    uint16_t guard = 0;
    while (!echoReady && guard < 30000) {
        _delay_us(1);
        guard++;
    }
    if (!echoReady) { echoState = 0; return 0; }

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
  return z_raw * 1;
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

int32_t read_open(void) {
    int32_t a = us_sensor_get_distance(); _delay_ms(20);
    int32_t b = us_sensor_get_distance(); _delay_ms(20);
    int32_t c = us_sensor_get_distance();
    if (a == 0) a = 999;
    if (b == 0) b = 999;
    if (c == 0) c = 999;
    int32_t hi = (a > b) ? a : b; hi = (hi > c) ? hi : c;
    int32_t lo = (a < b) ? a : b; lo = (lo < c) ? lo : c;
    return a + b + c - hi - lo;
}

// Servo physically rotated 180deg -> mirror commanded pulse about center before output.
ISR(TIMER1_COMPA_vect){ OCR1A = 2 * SERVO_CENTER_US - servoPulseWidth; }

// ---------- States ----------
enum SystemState {
  CRUISE       = 0,
  APPROACH     = 1,
  SCAN_SIDES   = 2,
  EXECUTE_TURN = 3,
  STRAIGHTEN   = 4
};
SystemState systemState = CRUISE;

int turnDirection = 1;
int32_t targetYaw = 0;

int main(void) {
    I2C_init();
    MPU6050_init();

    DDRB |= (1 << PB3);
    DDRD &= ~(1 << PD2);
    DDRB |= (1 << PB5);
    DDRD |= (1 << FAN_LIFT_PIN) | (1 << FAN_DIR_PIN);

    EICRA = (1 << ISC00);
    EIMSK = (1 << INT0);

    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << TOIE0);

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

    OCR0A = FAN_STOP; OCR0B = FAN_STOP; _delay_ms(3000);
    OCR0B = FAN_LIFT_90; OCR0A = FAN_STOP; _delay_ms(1500);
    OCR0A = FAN_DIR_FULL;

    previousTime = get_millis();
    currentTime = previousTime;

    while(1) {
        int16_t gz_raw = read_gyro_z_raw();
        int32_t gz_scaled = ((int32_t)gz_raw * 100) / 131;
        gz_scaled_global = gz_scaled;          // share rate into the PD turn

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

                yawIntegral += (error * (int32_t)delta_t) / 1000;
                if (yawIntegral >  I_CLAMP) yawIntegral =  I_CLAMP;
                if (yawIntegral < -I_CLAMP) yawIntegral = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawIntegral * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int correction = pCorr + iCorr;
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
                    stuckArmed = 0;            // reset stuck-detect when entering approach normally
                    systemState = APPROACH;
                }
                break;
            }

            case APPROACH: {
                OCR0A = FAN_DIR_SLOW;
                OCR0B = FAN_LIFT_90;

                int32_t error = targetYaw - yaw;

                yawIntegral += (error * (int32_t)delta_t) / 1000;
                if (yawIntegral >  I_CLAMP) yawIntegral =  I_CLAMP;
                if (yawIntegral < -I_CLAMP) yawIntegral = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawIntegral * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int correction = pCorr + iCorr;
                int newPulse = SERVO_CENTER_US + correction;
                if (newPulse < SERVO_LEFT_US)  newPulse = SERVO_LEFT_US;
                if (newPulse > SERVO_RIGHT_US) newPulse = SERVO_RIGHT_US;
                targetPulse = newPulse;

                static int32_t lastGoodDist2 = 999;
                if (frontDist > 0) lastGoodDist2 = frontDist;

                uint8_t atWall =
                    (frontDist > 0 && frontDist < TURN_CM) ||
                    (frontDist == 0 && lastGoodDist2 < TURN_CM);

                // ---- #2 STUCK-AT-WALL escape ----
                // If close to a wall and the front distance isn't changing for STUCK_MS, the
                // craft is pinned (rudder straight, not progressing). Force the turn.
                uint8_t closeBand =
                    (frontDist > 0 && frontDist < STUCK_CM) ||
                    (frontDist == 0 && lastGoodDist2 < STUCK_CM);
                uint8_t forcedStuck = 0;
                if (closeBand) {
                    if (!stuckArmed) {
                        stuckArmed = 1;
                        stuckStart = get_millis();
                        stuckRefDist = (frontDist > 0) ? frontDist : lastGoodDist2;
                    } else {
                        int32_t cur = (frontDist > 0) ? frontDist : lastGoodDist2;
                        if (labs((long)(cur - stuckRefDist)) > STUCK_DELTA) {
                            // distance is changing -> making progress, re-arm the timer
                            stuckStart = get_millis();
                            stuckRefDist = cur;
                        } else if (get_millis() - stuckStart >= STUCK_MS) {
                            forcedStuck = 1;     // stuck too long -> force the turn
                        }
                    }
                } else {
                    stuckArmed = 0;
                }

                if (atWall || forcedStuck) {
                    lastGoodDist2 = 999;
                    stuckArmed = 0;
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
                    targetPulse = SERVO_LEFT_US;
                    if (abs(servoPulseWidth - targetPulse) <= SERVO_SPEED ||
                        (get_millis() - scanStart) >= SCAN_MAX_MS) {
                        if (!scanArmed) { settleStart = get_millis(); scanArmed = 1; }
                        else if (get_millis() - settleStart >= SCAN_SETTLE_MS) {
                            leftOpen = read_open();
                            scanPhase = 1; scanArmed = 0;
                            scanStart = get_millis();
                        }
                    }
                } else {
                    targetPulse = SERVO_RIGHT_US;
                    if (abs(servoPulseWidth - targetPulse) <= SERVO_SPEED ||
                        (get_millis() - scanStart) >= SCAN_MAX_MS) {
                        if (!scanArmed) { settleStart = get_millis(); scanArmed = 1; }
                        else if (get_millis() - settleStart >= SCAN_SETTLE_MS) {
                            rightOpen = read_open();

                            if (leftOpen > rightOpen) turnDirection = -1;
                            else                      turnDirection = 1;

                            targetYaw += (9000 * turnDirection);   // 9000 = 90.00 degrees
                            targetPulse = SERVO_CENTER_US;
                            scanArmed = 0;
                            turnStart = get_millis();
                            systemState = EXECUTE_TURN;
                        }
                    }
                }
                break;

            case EXECUTE_TURN: {
                // ---- #1 PD TURN CONTROLLER ----
                // Drive on remaining angle, brake on rotation rate. As yaw nears targetYaw the
                // P term shrinks; the D term (gyro rate) subtracts hard while spinning fast, so
                // it eases into the target instead of slamming full-throw then overshooting.
                OCR0A = FAN_DIR_FULL;
                OCR0B = FAN_LIFT_TURN;

                int32_t remaining = targetYaw - yaw;          // signed: + means need more CW(right)
                // PD output as a rudder offset from center.
                int32_t pTerm = (remaining * Kp_turn) / 100;
                int32_t dTerm = (gz_scaled_global * Kd_turn) / 100;
                int32_t turnCmd = pTerm - dTerm;              // brake against current spin rate

                int newPulse = SERVO_CENTER_US + (int)turnCmd;
                if (newPulse < SERVO_LEFT_US)  newPulse = SERVO_LEFT_US;
                if (newPulse > SERVO_RIGHT_US) newPulse = SERVO_RIGHT_US;
                targetPulse = newPulse;

                // done when within TURN_DONE of target (either direction), or timeout escape.
                if (labs((long)remaining) <= TURN_DONE ||
                    (get_millis() - turnStart >= TURN_MAX_MS)) {
                    OCR0B = FAN_LIFT_90;
                    yawIntegral = 0;
                    straightenStart = get_millis();
                    systemState = STRAIGHTEN;
                }
                break;
            }

            case STRAIGHTEN: {
                OCR0A = FAN_DIR_SLOW;
                OCR0B = FAN_LIFT_90;

                int32_t error = targetYaw - yaw;

                yawIntegral += (error * (int32_t)delta_t) / 1000;
                if (yawIntegral >  I_CLAMP) yawIntegral =  I_CLAMP;
                if (yawIntegral < -I_CLAMP) yawIntegral = -I_CLAMP;

                int pCorr = (int)((error * Kp) / 100);
                int iCorr = (int)((yawIntegral * Ki) / 100);
                if (iCorr >  I_OUT_CLAMP) iCorr =  I_OUT_CLAMP;
                if (iCorr < -I_OUT_CLAMP) iCorr = -I_OUT_CLAMP;

                int correction = pCorr + iCorr;
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
        OCR1A = 2 * SERVO_CENTER_US - servoPulseWidth;

        _delay_ms(40);
    }
    return 0;
}
