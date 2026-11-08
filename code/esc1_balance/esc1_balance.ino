// esc1_balance.ino (v3.6, 7 Nov 2026) — FIRST SUSTAINED BALANCE: 126 s, tilt rms 0.27 deg — balance the reaction-wheel pendulum upright. B-G431B-ESC1, SimpleFOC 2.4.0, STM32 core 2.12.0.
//
//   tilt  : AS5048A PWM on J3 pad 2 (PA15), measured by TIM2 in PWM-input mode (since v3)
//   wheel : MT6701 ABZ on J8 (PB6/PB7), 4096 counts/rev; the motor runs in voltage-torque mode (target = Uq volts)
//   loop  : 1 kHz.  u = -g * (K0*theta + K1*theta_dot + K2*wheel_speed), clipped to +-6 V
//
// Model and gains come from the hanging tests (sysid.py, 7 Nov 2026):
//   lambda = 8.207 rad/s, I_w/J = r = 0.01173, wheel 8.8 rad/s per volt; LQR designed in discrete time at 1 ms.
// theta_dot comes from a MOMENTUM observer: p = (1+r)*theta_dot + r*wheel_speed is the total angular momentum
// about the pivot (per J). Only gravity changes it (p_dot = lambda^2*sin(theta) - c*theta_dot), so the observer never
// needs the motor constants the kick test couldn't separate. Tilt readings more than 1 deg off the observer's
// prediction are thrown away (the motor's noise spikes on the PWM wire); 20 in a row and it accepts them again.
//
// Sign convention: + tilt = counter-clockwise seen from the FRONT (wheel side). Upright, + = top leaning LEFT.
//
// HOW TO RUN IT (arch on, foam on the pads; 12 V first, then USB; Serial 115200, Newline):
//   1. The sketch calibrates the motor at start. Let the bar rest on a pad meanwhile.
//   2. Send  A  (or press the board's button). Status says "waiting".
//   3. Lift the bar slowly to upright and hold it lightly between two fingers. When it is within 1 deg of
//      balance for 0.3 s the controller takes over: you feel the wheel push. Let go.
//   4. Send  X  (or the button) to stop. It also stops by itself past 25 deg, if the wheel runs away, or if the
//      tilt sensor stops sending (20 ms without a reading).
//   Knob: gain multiplier 0.6x (left) .. 0.85x (middle) .. 1.2x (right). Start in the middle.
//
// v2, after the first run (balanced ~50 s but with the motor slammed at +-6 V 97% of the time):
//   - knob range lowered: with the real loop's few ms of sensor delay, gains above ~1.2x chatter bang-bang
//   - the balance point trims itself: the zero creeps opposite to the wheel speed (a steady wheel speed means the
//     zero is off), about 1 deg every few seconds; the live line shows the trim. Put the final value in UPRIGHT_RAW.
//   - stops if the motor sits at the voltage limit for 0.5 s (it has lost control authority)
//   - the observer restarts cleanly whenever the bar is more than 10 deg from upright and the motor is off
//   - the P dump also has the raw sensor tilt, to see noise spikes
//
// v3, after the second run (the raw tilt jumped by +-1-2 deg every sample whenever the wheel spun):
//   - the AS5048A PWM is now measured by TIMER HARDWARE (TIM2 in PWM-input mode on PA15), not by an interrupt
//     reading a clock. At 50 rad/s the wheel encoder fires ~33,000 interrupts a second; each one could delay the
//     tilt interrupt by a microsecond or two, and 1 us of the ~1 ms PWM frame is 0.35 deg. The timer latches both
//     edges itself, so that error is gone, and its input filter ignores glitches shorter than 0.75 us.
//   - G<x> sets a fixed gain from the keyboard (G0.85), G alone goes back to the knob (the knob read maximum
//     in both runs - check it moves "g" in the live line)
//   - "wheel too fast" needs 20 ms in a row, so a single encoder glitch can't trip it
// v3.1: T<deg> sets the trim directly; Z also clears the trim
// v3.2: defaults set from the bench: gain 0.85, balance point raw 17.476 (re-measure with Z after re-routing cables);
//       prints its version at start-up
// v3.3: the tilt reading went wrong for ~10 ms at a time whenever the wheel accelerated hard. Suspect: the 12 V
//       1.5 A brick sagging (a +-6 V kick from standstill draws ~2.5 A from it), browning out the sensor's 5 V.
//       - the supply voltage (VBUS, PA0) is measured every 1 ms: the live line shows its lowest value since the
//         last line ("vb"), and the P dump logs it
//       - U<volts> sets the motor voltage limit; default 3.5 V keeps the draw inside the brick's 1.5 A
//         (simulation: still balances, the wheel tops out near 31 rad/s instead of 53)
// v3.4: VBUS stayed at 12.0 V (the 3 A supply limit fixed the sag) but the dips stayed: every sudden step in the
//       motor voltage makes the tilt reading jump the opposite way for ~10 ms (about 0.6 deg per volt), whether
//       from the stand shaking the sensor or from the current. The controller then answered the dip with another
//       step: a self-sustaining chatter. The simulation reproduces it exactly when given that effect, and two
//       changes each cure it there:
//       - a slower observer (q_p 10 -> 3): it trusts the sensor less over ~10 ms, so the dips barely get in
//       - a slew limit: the motor voltage changes by at most 0.3 V per ms, so there are no sharp steps to cause them
//       Both are in, which balances in simulation with dips up to twice as big, 5 ms of delay, gain 0.7-1.2.
// v3.5: first clean run (no chatter, 0 rejects) - it stayed up but the wheel drifted to the 3.5 V speed limit
//       (-30 rad/s) and the controller had nothing left. Simulation: at 3.5 V a balance point 0.8 deg off already
//       does that; at 6 V up to 1.2 deg is fine and the self-trim takes it out. With the 3 A supply:
//       - motor voltage limit back to 6 V by default (U3.5 for the old setting)
//       - the live line printed every 100 ms stalled the loop for ~6 ms (the 2 ms log shows wheel-speed spikes of
//         -180 rad/s exactly every 100 ms). While ON it now prints a short line every 0.2 s that fits the serial
//         buffer, and a stall can no longer fake a wheel-speed spike
//       - "stuck at the voltage limit" now needs 1 s, not 0.5 s
// v3.6: the last blocker was mechanical: the motor and encoder cables all left the shaft on one side and held the
//       bar near upright, so leaning couldn't unload the wheel and its speed ran away. With the phase wires routed
//       to the left and the encoder wires to the right it balanced for 2 minutes (wheel within +-8 rad/s, motor
//       ~1.3 V rms) and recovered from one tap. Balance point = where the self-trim settled (31.123 - 1.30).
//   Other commands:  +/-  trim the balance point by 0.05 deg   T<deg> set the trim (T-2.2: what reads -2.2 becomes 0)
//                    Z  balance point = here (hold it balanced, motor off)
//                    L  live readout on/off   P  print the last 2 s (CSV) after a fall   I  re-calibrate the motor

#include <SimpleFOC.h>

// ---------------- model and gains ----------------
const float TS     = 0.001f;
const float LAM2   = 67.3548f;      // lambda^2 (1/s^2)
const float R_I    = 0.01173f;      // I_wheel / J
const float C_V    = 0.12f;         // pivot viscous friction (1/s)
const float K0 = -182.1729f, K1 = -21.1444f, K2 = -0.2273f;   // V/rad, V/(rad/s), V/(rad/s)
const float L0 = 0.101382f, L1 = 5.480116f;                      // observer gains (theta, p), q_p = 3 (v3.4)
const float SLEW = 0.3f;                                          // V per 1 ms control step (v3.4)
const float GATE   = 1.0f * DEG_TO_RAD;                         // reject tilt readings this far off prediction
float UPRIGHT_RAW  = 29.82f;        // deg: where the self-trim settled during the 2-minute balance (7 Nov, cables split L/R)
float U_MAX        = 6.0f;          // V. U<volts> to change (6 = what 12 V allows; needs a ~3 A supply)
const float VBUS_K = 3.3f / 4095.0f * (169.0f + 18.0f) / 18.0f;   // board divider 169k/18k -> volts
const float FALL   = 25.0f * DEG_TO_RAD;
const float W_MAX  = 75.0f;         // rad/s: above what 6 V can reach -> means something is wrong
const float ARM_WIN = 1.0f * DEG_TO_RAD;
const float KTRIM  = 2.0e-4f;       // balance-point trim rate: rad of zero shift per (rad/s of wheel) per second
float trim = 0;                     // rad, subtracted from every tilt reading

// ---------------- hardware ----------------
BLDCMotor motor(11);
BLDCDriver6PWM driver(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
Encoder encoder(PB6, PB7, 1024);
void doA() { encoder.handleA(); }
void doB() { encoder.handleB(); }
const int BUTTON = PC10, KNOB = PB12;

// AS5048A PWM -> TIM2 channel 1 (PA15, AF1) in PWM-input mode, clocked at 170 MHz:
//   CH1 captures on the rising edge = frame period (the counter is reset there), CH2 on the falling edge = high time.
void tiltCaptureInit() {
  RCC->AHB2ENR  |= RCC_AHB2ENR_GPIOAEN;
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  (void)RCC->APB1ENR1;
  GPIOA->MODER = (GPIOA->MODER & ~(3u << 30)) | (2u << 30);          // PA15 alternate function
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(0xFu << 28)) | (1u << 28);       // AF1 = TIM2_CH1
  GPIOA->PUPDR &= ~(3u << 30);
  TIM2->CR1 = 0; TIM2->PSC = 0; TIM2->ARR = 0xFFFFFFFF;
  TIM2->CCMR1 = (1u << TIM_CCMR1_CC1S_Pos) | (2u << TIM_CCMR1_CC2S_Pos)    // both channels from TI1
              | (0xCu << TIM_CCMR1_IC1F_Pos);                              // filter: 8 samples at 170/16 MHz = 0.75 us
  TIM2->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC2P;             // CH1 rising, CH2 falling
  TIM2->SMCR = (5u << TIM_SMCR_TS_Pos) | (4u << TIM_SMCR_SMS_Pos);        // trigger TI1FP1, reset mode
  TIM2->EGR = TIM_EGR_UG; TIM2->SR = 0; TIM2->CR1 = TIM_CR1_CEN;
}
volatile uint32_t frames = 0;
bool rawDeg(float &deg) {
  if (TIM2->SR & TIM_SR_CC1IF) frames++;            // a new frame since last look (reading CCR1 clears the flag)
  uint32_t p = TIM2->CCR1, h = TIM2->CCR2;
  if (!(p > 34000 && p < 850000 && h < p)) return false;
  float counts = (float)h / p * 4119.0f - 16.0f;
  deg = constrain(counts, 0.0f, 4095.0f) * 360.0f / 4096.0f;
  return true;
}
float wrapPi(float a) { while (a > PI) a -= TWO_PI; while (a < -PI) a += TWO_PI; return a; }

// ---------------- state ----------------
enum Mode { M_OFF, M_WAIT, M_ON } mode = M_OFF;   // (SimpleFOC already uses ON/OFF)
float thH = 0, pH = 0;          // observer: tilt from upright (rad), momentum (rad/s)
float wM = 0, psiPrev = 0, u = 0, g = 1.0f, thM = 0, thdH = 0;
int rej = 0; uint32_t rejTotal = 0, inWinSince = 0, tNext = 0, satSince = 0;
bool live = true, btnIdle = true;
float gFixed = 0.85f;               // > 0: gain set from the keyboard (the knob reads max), G alone = use the knob
int wFast = 0;
const char *why = "";

// 2 s log at 2 ms for after-fall analysis
const int NLOG = 1000;
int16_t lgTh[NLOG], lgTm[NLOG], lgW[NLOG], lgU[NLOG], lgV[NLOG]; int lgI = 0; bool lgFull = false;
float vbus = 12, vbMin = 99;

void stopMotor(const char *reason) {
  u = 0; motor.move(0); motor.disable();
  if (mode != M_OFF) {
    why = reason; Serial.print("# STOP: "); Serial.print(reason);
    Serial.print("   (trim so far "); Serial.print(trim * RAD_TO_DEG, 2);
    Serial.print(" deg -> balance point raw "); Serial.print(UPRIGHT_RAW + trim * RAD_TO_DEG, 3); Serial.println(")");
  }
  mode = M_OFF;
}

void printStatus() {
  Serial.print("# motor status "); Serial.print((int)motor.motor_status);
  Serial.print(" (4 = ready)  dir "); Serial.print((int)motor.sensor_direction);
  Serial.print("  balance point raw "); Serial.println(UPRIGHT_RAW, 3);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("# esc1_balance v3.6");
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  tiltCaptureInit();
  pinMode(BUTTON, INPUT_PULLUP);
  analogReadResolution(12);

  encoder.quadrature = Quadrature::ON;
  encoder.pullup = Pullup::USE_INTERN;
  encoder.init();
  encoder.enableInterrupts(doA, doB);
  motor.linkSensor(&encoder);
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.controller = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_sensor_align = 2.0f;
  motor.voltage_limit = 6.0f;              // hard ceiling; the controller clips to U_MAX itself
  motor.init();
  motor.initFOC();
  motor.disable();
  printStatus();
  delay(50);
  btnIdle = digitalRead(BUTTON);            // whatever level it rests at; a change = press
  psiPrev = motor.sensor_direction * encoder.getAngle();
  float d; if (rawDeg(d)) thH = wrapPi((d - UPRIGHT_RAW) * DEG_TO_RAD);
  pH = 0;
  Serial.println("# ready.  A = arm (then bring it upright), X = stop, G<x> gain, U<v> volt limit, L = live, P = last 2 s, +/- T<deg> trim, Z, I");
  tNext = micros();
}

void handleSerial() {
  static char line[16]; static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < 15) line[n++] = c; continue; }
    line[n] = 0; if (!n) continue; n = 0;
    char cmd = toupper(line[0]);
    if (cmd == 'A') { if (mode == M_OFF) { mode = M_WAIT; inWinSince = 0; Serial.println("# waiting: bring it upright"); } }
    else if (cmd == 'X') stopMotor("stop command");
    else if (cmd == 'L') live = !live;
    else if (cmd == 'U') { U_MAX = constrain(atof(line + 1), 1.0f, 6.0f); Serial.print("# motor voltage limit "); Serial.println(U_MAX, 1); }
    else if (cmd == 'G') { gFixed = line[1] ? constrain(atof(line + 1), 0.4f, 1.5f) : 0;
                           Serial.print("# gain "); if (gFixed > 0) Serial.println(gFixed, 2); else Serial.println("from the knob"); }
    else if (cmd == '+') { UPRIGHT_RAW += 0.05f; Serial.print("# balance point raw "); Serial.println(UPRIGHT_RAW, 3); }
    else if (cmd == '-') { UPRIGHT_RAW -= 0.05f; Serial.print("# balance point raw "); Serial.println(UPRIGHT_RAW, 3); }
    else if (cmd == 'Z' && mode == M_OFF) { float d; if (rawDeg(d)) { UPRIGHT_RAW = d; trim = 0; thH = 0; Serial.print("# balance point raw "); Serial.println(d, 3); } }
    else if (cmd == 'T') { trim = atof(line + 1) * DEG_TO_RAD;          // T-2.2: what now reads -2.2 becomes 0
                           Serial.print("# trim "); Serial.print(trim * RAD_TO_DEG, 2);
                           Serial.print(" deg -> balance point raw "); Serial.println(UPRIGHT_RAW + trim * RAD_TO_DEG, 3); }
    else if (cmd == 'I' && mode == M_OFF) { motor.enable(); motor.initFOC(); motor.disable(); printStatus(); }
    else if (cmd == 'P' && mode == M_OFF) {
      Serial.println("t_ms,theta_deg,theta_raw_deg,wheel_rad_s,u_V,vbus_V");
      int n0 = lgFull ? NLOG : lgI;
      for (int i = 0; i < n0; i++) {
        int j = lgFull ? (lgI + i) % NLOG : i;
        Serial.print(i * 2); Serial.print(','); Serial.print(lgTh[j] * 0.01f, 2); Serial.print(',');
        Serial.print(lgTm[j] * 0.01f, 2); Serial.print(',');
        Serial.print(lgW[j] * 0.1f, 1); Serial.print(','); Serial.print(lgU[j] * 0.001f, 3);
        Serial.print(','); Serial.println(lgV[j] * 0.01f, 2);
      }
      Serial.println("# end");
    }
  }
}

void control() {
  // ---- measurements ----
  static uint32_t tPrev = micros(), lastFrames = 0, lastFrameMs = 0;
  uint32_t now = micros();
  float dt = (now - tPrev) * 1e-6f; tPrev = now;
  if (dt < 0.0005f) dt = TS;
  if (dt > 0.02f) dt = 0.02f;
  float psi = motor.sensor_direction * encoder.getAngle();
  wM = (psi - psiPrev) / dt; psiPrev = psi;
  vbus = analogRead(A_VBUS) * VBUS_K; if (vbus < vbMin) vbMin = vbus;
  float d; bool ok = rawDeg(d);
  if (frames != lastFrames) { lastFrames = frames; lastFrameMs = millis(); }
  ok = ok && (millis() - lastFrameMs < 5);                          // a stale reading counts as no reading
  if (mode == M_ON && millis() - lastFrameMs > 20) stopMotor("tilt sensor lost - check the TILT plug");
  thM = ok ? wrapPi((d - UPRIGHT_RAW) * DEG_TO_RAD - trim) : thH;

  // far from upright with the motor off (resting on a pad, being lifted): just follow the sensor, so the
  // observer starts clean when it reaches upright
  if (mode != M_ON && ok && fabsf(thM) > 10.0f * DEG_TO_RAD) {
    thH = thM; pH = R_I * wM; thdH = 0; rej = 0;
  } else {
    // ---- momentum observer ----
    thdH = (pH - R_I * wM) / (1 + R_I);
    float thP = thH + TS * thdH;
    float pP  = pH + TS * (LAM2 * sinf(thP) - C_V * thdH);
    float e = wrapPi(thM - thP);
    if (!ok || (fabsf(e) > GATE && rej < 20)) { rej++; rejTotal++; e = 0; } else rej = 0;
    thH = thP + L0 * e;
    pH  = pP + L1 * e;
    thdH = (pH - R_I * wM) / (1 + R_I);
  }

  // ---- knob: gain 0.6x .. 1.2x (0.85x in the middle), unless fixed with G<x> ----
  static float knob = 2048;
  knob += 0.01f * (analogRead(KNOB) - knob);
  g = gFixed > 0 ? gFixed : 0.6f * powf(1.2f / 0.6f, knob / 4095.0f);

  // ---- modes ----
  if (mode == M_WAIT) {
    if (fabsf(thH) < ARM_WIN && fabsf(wM) < 5) {
      if (!inWinSince) inWinSince = millis();
      if (millis() - inWinSince > 300) {
        mode = M_ON; motor.enable(); lgI = 0; lgFull = false; rejTotal = 0; satSince = 0;
        Serial.println("# ON - let go");
      }
    } else inWinSince = 0;
  }
  if (mode == M_ON) {
    if (fabsf(thH) > FALL) stopMotor("fell past 25 deg");
    else if ((fabsf(wM) > W_MAX ? ++wFast : (wFast = 0)) > 20) stopMotor("wheel too fast");
    else {
      float uNew = -g * (K0 * thH + K1 * thdH + K2 * wM);
      uNew = constrain(uNew, -U_MAX, U_MAX);
      u += constrain(uNew - u, -SLEW, SLEW);
      if (fabsf(u) < U_MAX - 0.5f) { trim -= KTRIM * wM * TS; satSince = 0; }      // self-trim only when in control
      else if (!satSince) satSince = millis();
      else if (millis() - satSince > 1000) stopMotor("motor stuck at the voltage limit for 1 s");
    }
  } else u = 0;

  // ---- log every 2 ms ----
  static bool odd = false; odd = !odd;
  if (mode == M_ON && odd) {
    lgTh[lgI] = (int16_t)lroundf(thH * RAD_TO_DEG * 100); lgTm[lgI] = (int16_t)lroundf(thM * RAD_TO_DEG * 100); lgW[lgI] = (int16_t)lroundf(wM * 10); lgU[lgI] = (int16_t)lroundf(u * 1000); lgV[lgI] = (int16_t)lroundf(vbus * 100);
    if (++lgI >= NLOG) { lgI = 0; lgFull = true; }
  }
}

void loop() {
  motor.loopFOC();
  motor.move(u);
  if ((int32_t)(micros() - tNext) >= 0) {
    tNext += 1000;
    if ((int32_t)(micros() - tNext) > 5000) tNext = micros() + 1000;    // fell behind (e.g. long print): resync
    control();
  }
  handleSerial();

  // button: a change from its resting level toggles A / X (debounced)
  static uint32_t tBtn = 0; static bool last = true;
  bool pressed = (digitalRead(BUTTON) != btnIdle);
  if (pressed && !last && millis() - tBtn > 300) {
    tBtn = millis();
    if (mode == M_OFF) { mode = M_WAIT; inWinSince = 0; Serial.println("# waiting: bring it upright"); }
    else stopMotor("button");
  }
  last = pressed;

  // live readout, 10 Hz. Short line, printed only if the serial buffer has room, so it never stalls the loop
  static uint32_t tP = 0;
  if (live && mode == M_ON && millis() - tP >= 200 && Serial.availableForWrite() >= 50) {   // short, never blocks
    tP = millis();
    Serial.print("ON "); Serial.print(thH * RAD_TO_DEG, 2); Serial.print(" w "); Serial.print(wM, 0);
    Serial.print(" u "); Serial.print(u, 1); Serial.print(" tr "); Serial.println(trim * RAD_TO_DEG, 2);
  }
  else if (live && mode != M_ON && millis() - tP >= 100 && Serial.availableForWrite() >= 63) {
    tP = millis();
    Serial.print(mode == M_ON ? "ON " : mode == M_WAIT ? "WAIT " : "off ");
    Serial.print("tilt "); Serial.print(thH * RAD_TO_DEG, 2);
    Serial.print(" rate "); Serial.print(thdH * RAD_TO_DEG, 0);
    Serial.print(" whl "); Serial.print(wM, 1);
    Serial.print(" u "); Serial.print(u, 2);
    Serial.print(" g "); Serial.print(g, 2);
    Serial.print(" vb "); Serial.print(vbMin, 1); vbMin = 99;
    Serial.print(" trim "); Serial.print(trim * RAD_TO_DEG, 2);
    Serial.print(" rej "); Serial.println(rejTotal);
  }
}
