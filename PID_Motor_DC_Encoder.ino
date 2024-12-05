// Deklarasi pin untuk L298N 
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// Deklarasi pin untuk sensor enkoder
const int encoderA = 2;
const int encoderB = 3;

volatile int pulseCount = 0;  // Variabel untuk menghitung pulsa
unsigned long lastTime = 0;
int rpm = 0;

// Variabel PID
float Kp = 0.5, Ki = 0.02, Kd = 0.8;  // Tuning PID
float setpoint = 200;  // Setpoint RPM
float integral = 0, previousError = 0;
float maxIntegral = 500;  // Batas untuk anti-windup

// Variabel kontrol motor
bool direction = true;  // true: searah jarum jam, false: berlawanan jarum jam
bool motorRunning = true;  // true: motor aktif, false: motor berhenti

// Variabel tambahan untuk perhitungan step response (untuk keperluan GUI)
unsigned long stepResponseTime = 0;
float steadyStateError = 0.0;
float overshoot = 0.0;
float riseTime = 0.0;
float settlingTime = 0.0;

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), countPulseA, RISING);

  Serial.begin(9600);
  Serial.println("Mulai mengontrol motor DC...");
}

void countPulseA() {
  pulseCount++;
}

float calculatePID(float currentRPM) {
  float error = setpoint - currentRPM;

  // Integral dengan anti-windup
  integral += error;
  integral = constrain(integral, -maxIntegral, maxIntegral);

  float derivative = error - previousError;
  float output = Kp * error + Ki * integral + Kd * derivative;

  previousError = error;
  return constrain(output, 0, 255);  // Batasi nilai PWM antara 0 dan 255
}

void controlMotor(float pwmValue) {
  analogWrite(enA, pwmValue);
  if (direction) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);  // Searah jarum jam
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);  // Berlawanan jarum jam
  }
}

void stopMotor() {
  analogWrite(enA, 0);  // Berhenti motor
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  // Pastikan motor benar-benar mati
  motorRunning = false;  // Nonaktifkan loop PID
}

void loop() {
  unsigned long currentTime = millis();

  // Hitung RPM setiap 100ms untuk kontrol lebih sering
  if (motorRunning && currentTime - lastTime >= 100) {
    rpm = (pulseCount * 600) / 100;  // Hitung RPM
    pulseCount = 0;
    lastTime = currentTime;

    // Kirim data RPM ke serial untuk debugging dan analisis step response
    Serial.print("SETPOINT:");
    Serial.print(setpoint);
    Serial.print(", RPM:");
    Serial.println(rpm);

    // Untuk analisis step response di GUI, kirim informasi tambahan
    Serial.print("STEP_RESPONSE_TIME:");
    Serial.print(currentTime);
    Serial.print(", STEADY_STATE_ERROR:");
    steadyStateError = abs(setpoint - rpm);  // Steady-state error
    Serial.print(steadyStateError);
    Serial.print(", OVERSHOOT:");
    overshoot = ((rpm - setpoint) / setpoint) * 100;  // Hitung overshoot dalam persentase
    Serial.print(overshoot);
    Serial.print(", RISE_TIME:");
    if (rpm > (setpoint * 0.1) && riseTime == 0) {
      riseTime = currentTime;  // Deteksi rise time ketika RPM mencapai 10% setpoint
    }
    Serial.print(riseTime);
    Serial.print(", SETTLING_TIME:");
    // Settling time perkirakan jika RPM sudah stabil dalam 5% dari setpoint
    if (abs(rpm - setpoint) < (setpoint * 0.05)) {
      settlingTime = currentTime;
    }
    Serial.println(settlingTime);

    // Hitung dan kendalikan dengan PID
    float pwm = calculatePID(rpm);
    controlMotor(pwm);
  }

  // Periksa perintah dari Python (GUI)
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    // Mengubah nilai PID jika perintah diterima
    if (command.startsWith("Kp:")) {
      Kp = command.substring(3).toFloat();
    } else if (command.startsWith("Ki:")) {
      Ki = command.substring(3).toFloat();
    } else if (command.startsWith("Kd:")) {
      Kd = command.substring(3).toFloat();
    } else if (command.startsWith("SETPOINT:")) {
      setpoint = command.substring(9).toFloat();
    } else if (command == "STOP") {
      stopMotor();
    } else if (command == "CW") {
      direction = true;  // Searah jarum jam
      motorRunning = true;  // Aktifkan motor kembali
    } else if (command == "CCW") {
      direction = false;  // Berlawanan jarum jam
      motorRunning = true;  // Aktifkan motor kembali
    }
  }
}