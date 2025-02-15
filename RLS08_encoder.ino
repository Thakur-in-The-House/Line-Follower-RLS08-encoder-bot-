#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Motor Driver Pins
#define PWMA 5
#define AIN1 6  // A01 (Left Motor M1)
#define AIN2 7  // A02 (Left Motor M2)
#define PWMB 9
#define BIN1 11 // B01 (Right Motor M1)
#define BIN2 10 // B02 (Right Motor M2)
#define STBY 13

// Robot Configuration
bool isBlackLine = 1;             // 1 for black line, 0 for white line
unsigned int lineThickness = 30;  // Line thickness in mm (10-35 works best)
unsigned int numSensors = 8;      // Number of sensors

// PID Control
int P, D, I, previousError, PIDvalue;
double error;
int lsp, rsp;
int lfSpeed = 150;
int currentSpeed = 60;
int sensorWeight[8] = {10, 4, 2, 1, -1, -2, -4, -10};  // Sensor weights for error calculation
int activeSensors;
float Kp = 0.8;
float Kd = 0.1;
float Ki = 0.00;

// Sensor Data
int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8], sensorArray[8];

void setup() {
    // Increase ADC speed for faster sensor readings
    sbi(ADCSRA, ADPS2);
    cbi(ADCSRA, ADPS1);
    cbi(ADCSRA, ADPS0);

    Serial.begin(9600);

    // Motor Driver Setup
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, HIGH);  // Enable motor driver
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);

    // Perform 360-degree calibration at startup
    calibrate();
    delay(1000);  // Wait for calibration to complete
}

void loop() {
    // Follow the black line
    readLine();
    if (currentSpeed < lfSpeed) currentSpeed++;  // Ramp up speed

    if (onLine == 1) {
        linefollow();  // Follow the line using PID control
    } else {
        // If the line is lost, turn to find it
        if (error > 0) {
            motorRight(-100);
            motorLeft(lfSpeed);
        } else {
            motorRight(lfSpeed);
            motorLeft(-100);
        }
    }
}

// Motor Control Functions
void motorLeft(int ispeed) {
    ispeed = constrain(ispeed, -255, 255);
    if (ispeed < 0) {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    } else if (ispeed > 0) {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    } else {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, LOW);
    }
    analogWrite(PWMA, abs(ispeed));
}

void motorRight(int ispeed) {
    ispeed = constrain(ispeed, -255, 255);
    if (ispeed < 0) {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    } else if (ispeed > 0) {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    } else {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, LOW);
    }
    analogWrite(PWMB, abs(ispeed));
}

// Line Following Function
void linefollow() {
    error = 0;
    activeSensors = 0;

    // Calculate error based on sensor weights
    for (int i = 0; i < 8; i++) {
        error += sensorWeight[i] * sensorArray[i] * sensorValue[i];
        activeSensors += sensorArray[i];
    }
    error = error / activeSensors;

    // PID Calculation
    P = error;
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    // Adjust motor speeds based on PID value
    lsp = currentSpeed - PIDvalue;
    rsp = currentSpeed + PIDvalue;

    lsp = constrain(lsp, 0, 255);
    rsp = constrain(rsp, 0, 255);

    motorRight(lsp);
    motorLeft(rsp);
}

// Read Sensor Values
void readLine() {
    onLine = 0;
    for (int i = 0; i < 8; i++) {
        sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], isBlackLine ? 0 : 1000, isBlackLine ? 1000 : 0);
        sensorValue[i] = constrain(sensorValue[i], 0, 1000);
        sensorArray[i] = sensorValue[i] > 500;
        if (sensorArray[i]) onLine = 1;
    }
}

// Calibrate Sensors
void calibrate() {
    // Rotate 360 degrees to calibrate sensors
    motorRight(100);
    motorLeft(-100);
    delay(1000);  // Adjust delay for 360-degree rotation
    motorRight(0);
    motorLeft(0);

    // Read sensor values during rotation
    for (int i = 0; i < 8; i++) {
        minValues[i] = analogRead(i);
        maxValues[i] = analogRead(i);
    }

    for (int i = 0; i < 22100; i++) {
        motorRight(100);
        motorLeft(-100);

        for (int j = 0; j < 8; j++) {
            minValues[j] = min(minValues[j], analogRead(j));
            maxValues[j] = max(maxValues[j], analogRead(j));
        }
    }

    // Calculate thresholds
    for (int i = 0; i < 8; i++) {
        threshold[i] = (minValues[i] + maxValues[i]) / 2;
        Serial.print(threshold[i]);
        Serial.print(" ");
    }
    Serial.println();

    motorRight(0);
    motorLeft(0);
}
