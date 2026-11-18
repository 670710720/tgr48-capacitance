// TGR-48 Capacitance meter (ESP32, method 2: ADC polling until 63.2%)
// Circuit:
//   GPIO25 -> R1 (1 MOhm) -> Vc node -> GPIO34 (ADC1)
//   Vc node -> Cx -> GND
//   GPIO27 -> push button -> GND
// Press the button once -> measure once, print over Serial 9600, LED (GPIO2) on 200 ms.
//
// How it works: set GPIO25 HIGH, read the ADC continuously, and record the time t
// when Vc reaches 63.2% of Vcc. At that point t = tau = R1 * Cx, so Cx = t / R1.
// Timing uses the CPU cycle counter (~4 ns resolution at 240 MHz).

const int PIN_CHARGE = 25, PIN_ADC = 34, PIN_BTN = 27, PIN_LED = 2;

// ---- Calibration (2-point, see steps) ----
const float R1 = 1.043e6;       // R1 measured with Fluke 117 (out of circuit)
const float VCC_MV = 3268;      // 3V3 rail measured with Fluke 117
const float C_OFFSET_PF = 0;    // from the 20 pF capacitor
const float GAIN = 1.0;         // from the 0.1 uF capacitor

// ---- Measurement settings ----
const float VTH_FRAC = 0.632;               // threshold = 63.2% of Vcc
const float TIMEOUT_S = 5.0;                // give up after 5 s (C > ~4.8 uF)

float lastTau = 0;    // tau of previous measurement (s), used for discharge time
float cyclesPerSec;   // CPU cycles per second

void discharge(float tau) {
  digitalWrite(PIN_CHARGE, LOW);
  delay((uint32_t)(7.0 * tau * 1000.0) + 2);      // wait 7 tau
}

/// One charge cycle. Returns the time (s) to reach the threshold, or -1 on timeout.
// If the first ADC sample is already above the threshold (small C), tau is computed
// from that single sample instead: tau = -t / ln(1 - Vc/Vcc).
// *firstSample is set true only if even that sample is too high to use (> 95%).
float timeToThreshold(bool *firstSample) {
  const float vth = VCC_MV * VTH_FRAC;
  const float k = -log(1.0 - VTH_FRAC);            // = 1.0 for 63.2%
  const uint32_t timeoutCycles = (uint32_t)(TIMEOUT_S * cyclesPerSec);
  *firstSample = false;
  analogReadMilliVolts(PIN_ADC);                   // dummy read: wake up the ADC first
  digitalWrite(PIN_CHARGE, HIGH);
  uint32_t c0 = ESP.getCycleCount();
  float tPrev = 0, vPrev = 0;
  bool first = true;
  while (true) {
    uint32_t a = ESP.getCycleCount();
    float v = analogReadMilliVolts(PIN_ADC);
    uint32_t b = ESP.getCycleCount();
    float t = (((a - c0) + (b - c0)) * 0.5f) / cyclesPerSec;   // middle of the read
    if (v >= vth) {
      digitalWrite(PIN_CHARGE, LOW);
      if (first) {                                 // small C: use this single sample
        if (v >= VCC_MV * 0.95) { *firstSample = true; return 0; }
        float tau = -t / log(1.0 - v / VCC_MV);
        return tau * k;                            // same unit as time-to-threshold
      }
      return tPrev + (vth - vPrev) * (t - tPrev) / (v - vPrev);  // linear interpolation
    }
    if (b - c0 >= timeoutCycles) { digitalWrite(PIN_CHARGE, LOW); return -1; }
    tPrev = t; vPrev = v; first = false;
  }
}

float measurePF() {
  bool firstSample = true;
  float t = -1;
  for (int tries = 0; tries < 5; tries++) {        // retry if the first ADC sample was too late
    discharge(lastTau);
    t = timeToThreshold(&firstSample);
    if (t < 0 || !firstSample) break;
    lastTau = 0;
  }
  if (t < 0) { lastTau = 1.0; return NAN; }        // too large (timeout)
  if (firstSample) { lastTau = 0; return NAN; }    // too small (failed 5 times)

  const float k = -log(1.0 - VTH_FRAC);            // = 1.0 for 63.2%
  float tau = t / k;
  int n = constrain((int)(2.0 / (8.0 * tau)), 3, 200); // small C: up to 200 runs
  float sum = 0; int ok = 0;
  for (int i = 0; i < n; i++) {
    discharge(tau);
    float ti = timeToThreshold(&firstSample);
    if (ti > 0 && !firstSample) { sum += ti; ok++; }
  }
  lastTau = tau;
  if (!ok) return NAN;
  float c = (sum / ok) / (R1 * k) * 1e12;          // Cx = t / R1  (pF)
  return (c - C_OFFSET_PF) * GAIN;
}

uint32_t ledOffAt = 0; bool lastBtn = HIGH; uint32_t lastChange = 0;

void setup() {
  Serial.begin(9600);
  cyclesPerSec = getCpuFrequencyMhz() * 1e6;
  pinMode(PIN_CHARGE, OUTPUT); digitalWrite(PIN_CHARGE, LOW);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  analogSetAttenuation(ADC_11db);
}

void loop() {
  bool b = digitalRead(PIN_BTN);
  if (b != lastBtn && millis() - lastChange > 50) {   // software debounce 50 ms
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