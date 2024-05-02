#define ENCA_M1 27 // Encoder pin A for Motor 1
#define ENCB_M1 26 // Encoder pin B for Motor 1
#define PWM_PIN_M1 15 // Motor 1 PWM pin
#define IN1_PIN_M1 2 // Motor 1 direction pin 1
#define IN2_PIN_M1 0 // Motor 1 direction pin 2

#define ENCA_M2 19 // Encoder pin A for Motor 2
#define ENCB_M2 21 // Encoder pin B for Motor 2
#define PWM_PIN_M2 17 // Motor 2 PWM pin
#define IN1_PIN_M2 4 // Motor 2 direction pin 1
#define IN2_PIN_M2 16 // Motor 2 direction pin 2

#define IR_SENSOR_1_PIN 33 // IR sensor 1 pin
#define IR_SENSOR_2_PIN 32 // IR sensor 2 pin
#define IR_SENSOR_3_PIN 23 // IR sensor 3 pin
#define IR_SENSOR_4_PIN 22 // IR sensor 4 pin

#define M1 1 // Motor 1 identifier
#define M2 2 // Motor 2 identifier

volatile int encoderPosM1 = 0; // Encoder position for Motor 1
volatile int encoderPosM2 = 0; // Encoder position for Motor 2


void setup() {
  Serial.begin(9600);
  
  // Set IR sensor pins as inputs
  pinMode(IR_SENSOR_1_PIN, INPUT);
  pinMode(IR_SENSOR_2_PIN, INPUT);
  pinMode(IR_SENSOR_3_PIN, INPUT);
  pinMode(IR_SENSOR_4_PIN, INPUT);

  // Set motor 1 control pins as outputs
  pinMode(PWM_PIN_M1, OUTPUT);
  pinMode(IN1_PIN_M1, OUTPUT);
  pinMode(IN2_PIN_M1, OUTPUT);

  // Set motor 2 control pins as outputs
  pinMode(PWM_PIN_M2, OUTPUT);
  pinMode(IN1_PIN_M2, OUTPUT);
  pinMode(IN2_PIN_M2, OUTPUT);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_M1), updateEncoderM1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCA_M2), updateEncoderM2, CHANGE);
}

void loop() {
  // Read IR sensor values
  bool ir1 = !digitalRead(IR_SENSOR_1_PIN);
  bool ir2 = !digitalRead(IR_SENSOR_2_PIN);
  bool ir3 = !digitalRead(IR_SENSOR_3_PIN);
  bool ir4 = !digitalRead(IR_SENSOR_4_PIN);


  // Check if both IR1 and IR4 detect an obstacle
  bool walllr = ir1 && ir4;
  bool walll = ir1 && !ir4;
  bool wallf = ir2 || ir3;
  bool wallr = ir4 && !ir1;
  bool wallnlr = !ir4 && !ir1;

  // Adjust motor 1 based on obstacle detection
    if (wallf) {
      if(walllr){
        stopMotor(M1);
        stopMotor(M2);
        Serial.println("Wall in all sides");
      }if(walll){ 
        runMotor(M1, 60);
        runMotor(M2, 255);
        Serial.println("Wall on left and front");
      }if(wallr){ 
        runMotor(M1, 255);
        runMotor(M2, 60);
        Serial.println("Wall on right and front");
      }if(wallnlr){
        runMotor(M1, 60);
        runMotor(M2, 255);
        Serial.println("Wall on front only");
      }
    } else if (walllr || wallnlr) {
      runMotor(M1, 255);
      runMotor(M2, 255);
      Serial.println("Wall on neither side or on both sides");
    } else if (walll) {
      runMotor(M1, 60);
      runMotor(M2, 255);
      Serial.println("Wall on left");
    } else if (wallr) {
      runMotor(M1, 255);
      runMotor(M2, 60);
      Serial.println("Wall on right");
    }
}

void runMotor(int motor, int pwm) {
  if (motor == M1) {
    digitalWrite(IN1_PIN_M1, HIGH);
    digitalWrite(IN2_PIN_M1, LOW);
    analogWrite(PWM_PIN_M1, pwm); // Adjust PWM value as needed for motor speed
  } else if (motor == M2) {
    digitalWrite(IN1_PIN_M2, HIGH);
    digitalWrite(IN2_PIN_M2, LOW);
    analogWrite(PWM_PIN_M2, pwm); // Adjust PWM value as needed for motor speed
  }
}

void stopMotor(int motor) {
  if (motor == M1) {
    digitalWrite(IN1_PIN_M1, LOW);
    digitalWrite(IN2_PIN_M1, LOW);
    analogWrite(PWM_PIN_M1, 0);
  }else if (motor == M2) {
    digitalWrite(IN1_PIN_M2, LOW);
    digitalWrite(IN2_PIN_M2, LOW);
    analogWrite(PWM_PIN_M2, 0);
  }
}

void updateEncoderM1() {
  // This function updates the encoder position for Motor 1
  int encA = digitalRead(ENCA_M1);
  int encB = digitalRead(ENCB_M1);

  encoderPosM1 += (encA == encB) ? 1 : -1;
}

void updateEncoderM2() {
  // This function updates the encoder position for Motor 2
  int encA = digitalRead(ENCA_M2);
  int encB = digitalRead(ENCB_M2);

  encoderPosM2 += (encA == encB) ? 1 : -1;
}