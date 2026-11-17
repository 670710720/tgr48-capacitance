// TGR-48 Capacitance meter (ESP32, method 3: fixed charge time + ADC, auto-range)
// Circuit: GPIO25 -> R1 (1 MOhm) -> Vc node -> GPIO34 (ADC1)
//          Vc node -> Cx -> GND
// Press BOOT (GPIO0) once -> measure once, print over Serial 9600, LED (GPIO2) on 200 ms.

const int PIN_CHARGE = 25, PIN_ADC = 34, PIN_BTN = 0, PIN_LED = 2;

// ---- Calibration (measured on our board) ----
const float R1 = 1.043e6;       // R1 measured with Fluke 117 (out of circuit)
const float VCC_MV = 3268;      // 3V3 rail measured with Fluke 117
const float C_STRAY_PF = 6.8;   // stray C, measured with no Cx (avg of 10)
const float GAIN = 0.9765;      // from 104Z reference = 146 nF (Fluke 117)

// ---- Auto-range settings ----
const uint32_t T_MIN_US = 20;               // shortest charge time
const uint32_t COARSE_TIMEOUT_US = 4000000; // coarse step waits at most 4 s
const float MAX_TAU_S = 5.0;                // tau above this (C > ~4.8 uF) = out of range

float lastTau = 0;   // tau of previous measurement (s), used for discharge time

void discharge(float tau) {
  digitalWrite(PIN_CHARGE, LOW);
  delay((uint32_t)(7.0 * tau * 1000.0) + 2);      // wait 7 tau
}

// Coarse step: time until Vc reaches 50% to estimate tau
float estimateTau() {
  discharge(lastTau);
  digitalWrite(PIN_CHARGE, HIGH);
  uint32_t t0 = micros(), t = 0;
  float mv = 0;
  while (true) {
    mv = analogReadMilliVolts(PIN_ADC);
    t = micros() - t0;
    if (mv >= VCC_MV * 0.5 || t >= COARSE_TIMEOUT_US) break;
  }
  digitalWrite(PIN_CHARGE, LOW);
  float frac = mv / VCC_MV;
  if (frac < 0.05) return -1;                       // too large, barely charged
  if (frac > 0.95) frac = 0.95;
  return (t * 1e-6) / -log(1.0 - frac);
}

// Fine step: method 3 at the chosen charge time
float measureAt(uint32_t tch_us, float tau) {
  discharge(tau);
  digitalWrite(PIN_CHARGE, HIGH);
  delayMicroseconds(tch_us);
  float mv = analogReadMilliVolts(PIN_ADC);
  digitalWrite(PIN_CHARGE, LOW);
  if (mv < VCC_MV * 0.05 || mv > VCC_MV * 0.95) return NAN;
  return -(tch_us * 1e-6) / (R1 * log(1.0 - mv / VCC_MV)) * 1e12;
}

float measurePF() {
  float tau = estimateTau();
  if (tau < 0 || tau > MAX_TAU_S) { lastTau = 1.0; return NAN; }
  uint32_t tch = (uint32_t)(0.693 * tau * 1e6);     // aim for Vc = 50%
  if (tch < T_MIN_US) tch = T_MIN_US;
  int n = constrain((int)(2.0 / (7.7 * tau)), 3, 32); // average at least 3 runs
  float sum = 0; int ok = 0;
  for (int i = 0; i < n; i++) {
    float c = measureAt(tch, tau);
    if (!isnan(c)) { sum += c; ok++; }
  }
  lastTau = tau;
  return ok ? (sum / ok - C_STRAY_PF) * GAIN : NAN;
}

uint32_t ledOffAt = 0; bool lastBtn = HIGH; uint32_t lastChange = 0;

void setup() {
  Serial.begin(9600);
  pinMode(PIN_CHARGE, OUTPUT); digitalWrite(PIN_CHARGE, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  bool b = digitalRead(PIN_BTN);
  if (b != lastBtn && millis() - lastChange > 50) {   // debounce
    lastChange = millis(); lastBtn = b;
    if (b == LOW) {                                    // one press = one measurement
      float pf = measurePF();
      if (isnan(pf)) Serial.println("\"TGR-48\"\nCapacitance = out of range");
      else Serial.printf("\"TGR-48\"\nCapacitance = %.1f pF\n", pf);
      digitalWrite(PIN_LED, HIGH); ledOffAt = millis() + 200;
    }
  }
  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }
}
