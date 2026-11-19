// TGR-48 Coffee bean moisture meter (ESP32 + LM393 comparator, inverting)
// Circuit:
//   GPIO25 -> R1 (1 MOhm) -> Vc node -> LM393 IN- ,  Vc node -> sensor plate A, plate B -> GND
//   Vref: 3.3V -> 10k (trimmer outer legs) -> node -> 2k -> GND, node -> LM393 IN+  (~0.55 V)
//   LM393 OUT -> GPIO34, 10k pull-up to 3.3V;  GPIO27 -> push button -> GND
// Press the button once -> measure once, print C and moisture over Serial 9600, LED 200 ms.
//   t = R1 * C * k,  k = -ln(1 - Vref/Vcc)  ->  C = t / (R1 * k)
//   Moisture (%) = M_SLOPE * C + M_INTERCEPT   (from the 7.4% and 14.4% reference beans)

#include "soc/gpio_reg.h"

const int PIN_CHARGE = 25, PIN_OUT = 34, PIN_BTN = 27, PIN_LED = 2;

// ---- Hardware values (do not change the circuit after calibration) ----
const float R1 = 1.043e6;
const float VREF_V = 0.550;
const float VCC_V = 3.300;

// ---- C calibration (fit from 5, 15, 20 pF; checked with 10 pF) ----
const float GAIN = 0.9396;
const float OFFSET_PF = 7.74;

// ---- Moisture calibration (fill in after step 4) ----
const float M_SLOPE = 0.665;
const float M_INTERCEPT = -4.60;

const float TIMEOUT_S = 2.0;

float k, cyclesPerSec, lastTau = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

inline bool outLow() { return !((REG_READ(GPIO_IN1_REG) >> (PIN_OUT - 32)) & 1); }

void discharge(float tau) {
  digitalWrite(PIN_CHARGE, LOW);
  delay((uint32_t)(7.0 * tau * 1000.0) + 2);
}

float crossTime(bool quiet) {
  if (outLow()) return -2;
  const uint32_t timeoutCycles = (uint32_t)(TIMEOUT_S * cyclesPerSec);
  uint32_t c0, c1; bool ok = true;
  if (quiet) portENTER_CRITICAL(&mux);
  digitalWrite(PIN_CHARGE, HIGH);
  c0 = ESP.getCycleCount();
  while (!outLow()) {
    if (ESP.getCycleCount() - c0 >= timeoutCycles) { ok = false; break; }
  }
  c1 = ESP.getCycleCount();
  if (quiet) portEXIT_CRITICAL(&mux);
  digitalWrite(PIN_CHARGE, LOW);
  if (!ok) return -1;
  return (c1 - c0) / cyclesPerSec;
}

float measurePF() {
  discharge(lastTau > 0 ? lastTau : 0.001);
  float t = crossTime(false);
  if (t == -2) { discharge(0.2); t = crossTime(false); }
  if (t < 0) { lastTau = 0.1; return NAN; }
  float tau = t / k;
  bool quiet = tau < 0.01;
  int n = constrain((int)(2.0 / (8.0 * tau)), 3, 200);
  float sum = 0; int ok = 0;
  for (int i = 0; i < n; i++) {
    discharge(tau);
    float ti = crossTime(quiet);
    if (ti > 0) { sum += ti; ok++; }
  }
  lastTau = tau;
  if (!ok) return NAN;
  float c = (sum / ok) / (R1 * k) * 1e12;
  return (c - OFFSET_PF) * GAIN;
}

uint32_t ledOffAt = 0; bool lastBtn = HIGH; uint32_t lastChange = 0;

void setup() {
  Serial.begin(9600);
  cyclesPerSec = getCpuFrequencyMhz() * 1e6;
  k = -log(1.0 - VREF_V / VCC_V);
  pinMode(PIN_CHARGE, OUTPUT); digitalWrite(PIN_CHARGE, LOW);
  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  bool b = digitalRead(PIN_BTN);
  if (b != lastBtn && millis() - lastChange > 50) {
    lastChange = millis(); lastBtn = b;
    if (b == LOW) {
      float pf = measurePF();
      Serial.print("\"TGR-48\"\r\n");
      if (isnan(pf)) {
        Serial.print("Capacitance = out of range\r\n\r\n");
      } else {
        Serial.print("Capacitance = "); Serial.print(pf, 2); Serial.print(" pF\r\n");
        Serial.print("Moisture = "); Serial.print(M_SLOPE * pf + M_INTERCEPT, 1); Serial.print(" %\r\n\r\n");
      }
      digitalWrite(PIN_LED, HIGH); ledOffAt = millis() + 200;
    }
  }
  if (ledOffAt && (int32_t)(millis() - ledOffAt) >= 0) { digitalWrite(PIN_LED, LOW); ledOffAt = 0; }
}