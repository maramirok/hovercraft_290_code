/*
  Hovercraft Control: "The Navigator" (PURE C VERSION)
  TURN UPGRADE v11: "Snap-Glance" & Skirt Drop Optimization
  
  COMPLIANCE:
  - No Arduino libraries used.
  - NO FLOATING POINT MATH (100x Speed Increase).
  - Left-turn tuning for counter-torque (Asymmetric Turning).
  - Two-stage approach retained (SLOW_CM -> TURN_CM).
  - Anti-Jam Timeout added (2 seconds).
  - SNAP-GLANCE: Blocking rapid-scan with physical skirt-drop braking.
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
const int SERVO_LEFT_US   = 1750;
const int SERVO_RIGHT_US  = 4500;

volatile int servoPulseWidth = SERVO_CENTER_US;
int targetPulse = SERVO_CENTER_US;
const int SERVO_SPEED = 150;

volatile uint32_t timer0_millis = 0;

#define MPU6050_ADDR 0x68
const int FAN_DIR_FULL = 255;
const int FAN_LIFT_90  = 230;   // normal cruise lift
const int FAN_STOP = 0;

// ===== turn tuning knobs =====
const int FAN_LIFT_TURN = 50;        // deflate level to drop the skirt and brake
const int FAN_DIR_SLOW = 150;        // reduced thrust during the slow approach

// --- two-stage wall trigger ---
const int32_t SLOW_CM      = 35;     // notice the wall here -> slow down
const int32_t TURN_CM      = 18;     // commit to brake and scan here

// --- RIGHT turn (torque assists): brake harder, lead more ---
const int32_t TURN_LEAD      = 5000;  // Stop driving 50 degrees early
const int     CTR_STEER_US   = 1800;  // Air brake left
// --- LEFT turn (torque FIGHTS the spin): drive almost to target ---
const int32_t TURN_LEAD_LEFT = 500;   // Stop driving 5 degrees early
const int     CTR_STEER_LEFT = 0;     // No air brake needed
// =============================

int32_t yaw = 0;
int32_t gyroErrorZ = 0;
uint32_t previousTime = 0, currentTime = 0;
uint32_t turnStartTime = 0; // Tracks when a turn or escape sequence began

const int32_t Kp = 30;

int32_t leftOpen = 0, rightOpen = 0;

ISR(TIMER0_OVF_vect) { timer0_millis++; }

uint32_t get_millis() {
   uint32_t m;
   uint8_t oldSREG = SREG;
   cli();
   m = timer0_millis;
   SREG = oldSREG;
   return m;
}

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
 return z_raw * -1; // Polarity flipped so Clockwise = Positive
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

void trig_pulse(void) {
   PORTB |= (1 << PB3);
   _delay_us(10);
   PORTB &= ~(1 << PB3);
}

int32_t us_sensor_get_distance(void) {
   DDRB |= (1 << PB3);
   DDRD &= ~(1 << PD2);
   int32_t largestDistance = 0;
   for (int i = 0; i < 1; i++) {
       trig_pulse();
       uint32_t timeout = 8000;
       uint32_t counter = 0;
       while (!(PIND & (1 << PD2))) { _delay_us(1); if (++counter > timeout) break; }
       if (counter > timeout) continue;
       counter = 0;
       while (PIND & (1 << PD2)) { _delay_us(1); if (++counter > timeout) break; }
       int32_t d = counter / 58;
       if (d > largestDistance) largestDistance = d;
       _delay_ms(2);
   }
   return largestDistance;
}

// Median of 3 rejects single bad pings. 0 (no echo) mapped to 999 (infinite hallway)
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

ISR(TIMER1_COMPA_vect){ OCR1A = servoPulseWidth; }

// ---------- States ----------
enum SystemState {
 CRUISE        = 0,
 APPROACH      = 1,   
 SCAN_SIDES    = 2,   
 EXECUTE_TURN  = 3,
 BLIND_FORWARD = 4    
};
SystemState systemState = CRUISE;

int turnDirection = 1;     // +1 = right, -1 = left
int32_t targetYaw = 0;

int main(void) {
   I2C_init();
   MPU6050_init();

   DDRB |= (1 << PB3);
   DDRD &= ~(1 << PD2);
   DDRB |= (1 << PB5);
   DDRD |= (1 << FAN_LIFT_PIN) | (1 << FAN_DIR_PIN);

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

   // LAUNCH SEQUENCE
   OCR0A = FAN_STOP; OCR0B = FAN_STOP; _delay_ms(3000);
   OCR0B = FAN_LIFT_90; OCR0A = FAN_STOP; _delay_ms(1500);   
   OCR0A = FAN_DIR_FULL;                                     

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
               OCR0A = FAN_DIR_SLOW;         // Cut thrust by ~40%
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
                   systemState = SCAN_SIDES;
               }
               break;
           }

           case SCAN_SIDES: {
               // ==========================================
               //      THE DROP-AND-SNAP GLANCE MODULE
               // ==========================================
               
               // 1. DROP AND BRAKE (Kill Thrust + Deflate Skirt)
               OCR0A = FAN_STOP;
               OCR0B = FAN_LIFT_TURN; 
               _delay_ms(200); // Wait for hull to hit the floor and kill momentum

               // 2. SNAP LEFT
               targetPulse = SERVO_LEFT_US;
               servoPulseWidth = SERVO_LEFT_US; 
               OCR1A = SERVO_LEFT_US; // Bypass smoothing, slam PWM directly
               _delay_ms(400); // Wait for physical servo to swing and vibrate
               
               leftOpen = read_open();

               // 3. SNAP RIGHT
               targetPulse = SERVO_RIGHT_US;
               servoPulseWidth = SERVO_RIGHT_US;
               OCR1A = SERVO_RIGHT_US;
               _delay_ms(600); // Takes slightly longer to swing a full 180 deg
               
               rightOpen = read_open();

               // 4. DECIDE PATH
               if (leftOpen > rightOpen) turnDirection = -1;  // LEFT
               else                      turnDirection = 1;   // RIGHT (tie default)

               // 5. PREPARE FOR BURST ESCAPE
               yaw = 0; // Reset heading for relative turn tracking
               targetYaw = (9000 * turnDirection);
               
               targetPulse = SERVO_CENTER_US;
               servoPulseWidth = SERVO_CENTER_US; // Snap rudder back to center for thrust vectoring
               OCR1A = SERVO_CENTER_US;
               
               // 6. FIX DELTA_T SPIKE (Because we used blocking delays while stopped)
               previousTime = get_millis();
               currentTime = previousTime;
               turnStartTime = currentTime; // Start Anti-Jam Timer
               
               systemState = EXECUTE_TURN;
               break;
           }

           case EXECUTE_TURN:
               OCR0A = FAN_DIR_FULL;
               OCR0B = FAN_LIFT_TURN;          // Stay deflated to grip floor during spin

               // --- ANTI-JAM TIMEOUT ---
               // If wedged for >2000 ms, abort turn and push forward
               if (currentTime - turnStartTime > 2000) {
                   yaw = 0;             
                   targetYaw = 0;       
                   targetPulse = SERVO_CENTER_US; 
                   turnStartTime = currentTime; // Reset clock for the blind push
                   systemState = BLIND_FORWARD; 
                   break;
               }
               // ------------------------

               if (turnDirection == 1) {                          // RIGHT: torque assists
                   if (yaw < targetYaw - TURN_LEAD)
                       targetPulse = SERVO_RIGHT_US;
                   else
                       targetPulse = SERVO_CENTER_US - CTR_STEER_US; // counter-steer brake

                   if (yaw >= targetYaw) {
                       OCR0B = FAN_LIFT_90;                       // Pop back up onto air cushion
                       targetPulse = SERVO_CENTER_US;
                       systemState = CRUISE;                      // Re-engage normal flight
                   }
               } else {                                           // LEFT: torque fights us
                   if (yaw > targetYaw + TURN_LEAD_LEFT)
                       targetPulse = SERVO_LEFT_US;
                   else
                       targetPulse = SERVO_CENTER_US + CTR_STEER_LEFT; 

                   if (yaw <= targetYaw) {
                       OCR0B = FAN_LIFT_90;                       
                       targetPulse = SERVO_CENTER_US;
                       systemState = CRUISE;
                   }
               }
               break;

           case BLIND_FORWARD:
               // Lock servo center, fully inflate, and blast forward to escape the corner wedge
               targetPulse = SERVO_CENTER_US;
               OCR0A = FAN_DIR_FULL;
               OCR0B = FAN_LIFT_90;

               // Push aggressively for 500 ms (approx 15 cm of travel) to clear the wall
               if (currentTime - turnStartTime > 500) {
                   yaw = 0; 
                   targetYaw = 0;
                   systemState = CRUISE;
               }
               break;
       }

       // --- SERVO SMOOTHING (For normal flight only) ---
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
