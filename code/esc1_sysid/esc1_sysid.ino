// esc1_sysid.ino — measure the reaction-wheel pendulum's dynamics with it HANGING DOWN (no stops needed).
//
// Two tests, both recorded at 5 ms for 7.5 s and dumped as CSV (sysid_log.py on the PC saves + fits them):
//   S  SWING : motor off, wheel free. Pull the bar ~20 deg aside, type S, let go.
//              The hanging swing gives lambda = sqrt(m g l / J) - the SAME number that sets how fast it
//              falls when upright - plus the pivot friction.
//   K  KICK  : pendulum hanging still. The motor fires a pattern of +-U volt pulses into the wheel; the
//              pendulum recoils. Gives the motor's torque per volt, its back-EMF braking, and the
//              wheel-to-pendulum inertia ratio.
// Other commands:  H  zero = hanging (let it hang dead still first; upright is then exactly 180 away)
//                  U<v> pulse voltage for K (default 2.0)   L  toggle live angle printout   I  re-calibrate
//                  V<v> spin the wheel at a steady v volts (V0 = stop) and print its speed - motor check
//
// The tilt sensor is the AS5048A PWM on J3 pad 2 (PA15), measured by TIM2 in hardware PWM-input mode
// (v2: the old interrupt timing picked up errors from the wheel encoder's interrupts). The wheel is the MT6701 on J8.
// Power: 12 V first, then USB. Serial 115200.

#include <SimpleFOC.h>

const int   POLE_PAIRS = 11;
const float SUPPLY_V   = 12.0f;
const int   ENC_PPR    = 1024;

BLDCMotor motor(POLE_PAIRS);
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
Encoder encoder(PB6, PB7, ENC_PPR);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }

// ---------- AS5048A PWM, timed in CPU cycles ----------
void tiltCaptureInit() {                 // TIM2 CH1 on PA15 (AF1), PWM-input mode, 170 MHz
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  (void)RCC->APB1ENR1;
  GPIOA->MODER = (GPIOA->MODER & ~(3u << 30)) | (2u << 30);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFu << 28)) | (1u << 28);
  GPIOA->PUPDR &= ~(3u << 30);
  TIM2->CR1 = 0; TIM2->PSC = 0; TIM2->ARR = 0xFFFFFFFF;
  TIM2->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (2u << TIM_CCMR1_CC2S_Pos) | (0xCu << TIM_CCMR1_IC1F_Pos);
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;
  TIM2->SMCR = (5u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos);
  TIM2->EGR = TIM_EGR_UG; TIM2->SR = 0; TIM2->CR1 = TIM_CR1_CEN;
}
volatile uint32_t frames = 0;
float rawDeg(bool &ok) {
  if (TIM2->SR & TIM_SR_CC1IF) frames++;
  uint32_t p = TIM2->CCR1, h = TIM2->CCR2;
  ok = (p > 34000 && p < 850000 && h < p);            // 0.2 .. 5 ms frame
  if (!ok) return 0;
  float counts = (float)h / p * 4119.0f - 16.0f;
  return constrain(counts, 0.0f, 4095.0f) * 360.0f / 4096.0f;
}
float hangRaw = 0;
float wrap180(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }

// ---------- recording ----------
const int N = 1500, DT_US = 5000;                    // 7.5 s
int16_t thBuf[N];  float psiBuf[N];  int16_t uBuf[N];  // theta 0.01 deg, wheel rad, volts in mV
int idx = -1; char mode = 0; uint32_t tNext = 0;
float U = 2.0f, u = 0;

// kick pattern: {duration ms, sign}. Doublets of different lengths, quiet gaps to watch the recoil ring.
const int16_t KICK[][2] = { {300, 0}, {200, 1}, {400, -1}, {200, 1}, {1200, 0},
                            {100, -1}, {100, 1}, {300, -1}, {300, 1}, {100, -1}, {4000, 0} };
float kickU(uint32_t ms) {
  for (auto &k : KICK) { if (ms < (uint32_t)k[0]) return k[1] * U; ms -= k[0]; }
  return 0;
}

void dump() {
  Serial.print("# test="); Serial.print(mode == 'S' ? "swing" : "kick");
  Serial.print(" hang_raw_deg="); Serial.print(hangRaw, 3);
  Serial.print(" U="); Serial.print(U, 2);
  Serial.print(" dir="); Serial.println((int)motor.sensor_direction);
  Serial.println("t_ms,theta_deg,psi_rad,u_V");
  for (int i = 0; i < N; i++) {
    Serial.print(i * 5); Serial.print(',');
    Serial.print(thBuf[i] * 0.01f, 2); Serial.print(',');
    Serial.print(psiBuf[i] - psiBuf[0], 4); Serial.print(',');
    Serial.println(uBuf[i] * 0.001f, 3);
  }
  Serial.println("# end");
}

void printStatus() {
  Serial.print("# motor status "); Serial.print((int)motor.motor_status);
  Serial.print(" (4 = ready, 14 = calibration FAILED)   dir "); Serial.print((int)motor.sensor_direction);
  Serial.print("   zero_elec "); Serial.println(motor.zero_electric_angle, 3);
}

void start(char m) {
  mode = m; idx = 0; tNext = micros();
  if (m == 'S') { u = 0; motor.disable(); Serial.println("# SWING: recording 7.5 s - let go now"); }
  else          { motor.enable(); Serial.println("# KICK: recording 7.5 s - hands off"); }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  tiltCaptureInit();

  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);
  driver.voltage_power_supply = SUPPLY_V;
  driver.voltage_limit = 6;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;   // target = Uq volts, nothing hidden in between
  motor.voltage_sensor_align = 2.0f;
  motor.voltage_limit = 6.0f;
  SimpleFOCDebug::enable(&Serial);                        // prints the calibration steps
  motor.init();
  motor.initFOC();
  printStatus();
  motor.disable();                                        // wheel free until a K test
  Serial.println("# ready. Let it hang still -> H.  S = swing test, K = kick test, U<v>, L, I");
}

bool live = false;
char line[16]; int ln = 0;

void loop() {
  motor.loopFOC();
  motor.move(u);

  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (ln < 15) line[ln++] = c; continue; }
    line[ln] = 0; if (!ln) continue; ln = 0;
    char cmd = toupper(line[0]);
    bool ok; float r = rawDeg(ok);
    if (idx >= 0) { Serial.println("# busy recording"); continue; }
    if (cmd == 'H' && ok) { hangRaw = r; Serial.print("# hanging zero at raw "); Serial.print(r, 3);
                            Serial.print(" deg -> upright = raw "); Serial.println(fmodf(r + 180, 360), 3); }
    else if (cmd == 'S' || cmd == 'K') start(cmd);
    else if (cmd == 'U') { U = constrain(atof(line + 1), 0.2f, 5.0f); Serial.print("# U = "); Serial.println(U, 2); }
    else if (cmd == 'L') live = !live;
    else if (cmd == 'I') { motor.enable(); motor.initFOC(); printStatus(); motor.disable(); }
    else if (cmd == 'V') { u = constrain(atof(line + 1), -5.0f, 5.0f);
                           if (u == 0) motor.disable(); else { motor.enable(); live = true; }
                           Serial.print("# steady Uq = "); Serial.println(u, 2); }
    else if (!ok) Serial.println("# no PWM from the AS5048A - 12 V on?");
  }

  if (idx >= 0 && (int32_t)(micros() - tNext) >= 0) {
    tNext += DT_US;
    bool ok; float r = rawDeg(ok);
    float th = wrap180(r - hangRaw);
    if (mode == 'K') u = kickU(idx * 5);
    thBuf[idx] = (int16_t)lroundf(th * 100);
    psiBuf[idx] = motor.sensor_direction * encoder.getAngle();   // + when Uq is +
    uBuf[idx] = (int16_t)lroundf(u * 1000);
    if (++idx >= N) { idx = -1; u = 0; motor.move(0); motor.disable(); dump(); }
  }

  static uint32_t t = 0;
  if (live && idx < 0 && millis() - t >= 200) {
    t = millis();
    bool ok; float r = rawDeg(ok);
    Serial.print("# raw "); Serial.print(r, 2); Serial.print("  from hanging "); Serial.print(wrap180(r - hangRaw), 2);
    Serial.print("  wheel "); Serial.print(motor.sensor_direction * encoder.getAngle(), 2);
    Serial.print(" rad  "); Serial.print(motor.sensor_direction * encoder.getVelocity(), 1);
    Serial.print(" rad/s  Uq "); Serial.println(u, 2);
  }
}
