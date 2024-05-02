#include <stdio.h>
#include <stdbool.h>

#include <stdlib.h>
#include <math.h>
#include <string.h>
// #include "API.h"

//Defining directions
#define NORTH 0
#define EAST 1
#define WEST 2
#define SOUTH 3

int orient = NORTH;

//Defining maximum size of the maze
#define MAZE_MAX_SIZE 8

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

#define IR_SENSOR_1_PIN 32 // IR sensor 1 pin
#define IR_SENSOR_2_PIN 33 // IR sensor 2 pin
#define IR_SENSOR_3_PIN 22 // IR sensor 3 pin
#define IR_SENSOR_4_PIN 23 // IR sensor 4 pin

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

void read_readings() {
    bool ir1 = !digitalRead(IR_SENSOR_1_PIN);
    bool ir2 = !digitalRead(IR_SENSOR_2_PIN);
    bool ir3 = !digitalRead(IR_SENSOR_3_PIN);
    bool ir4 = !digitalRead(IR_SENSOR_4_PIN);

    Serial.print(ir1); Serial.print(" ");
    Serial.print(ir2); Serial.print(" ");
    Serial.print(ir3); Serial.print(" ");
    Serial.print(ir4); Serial.println();

    // Check if both IR1 and IR4 detect an obstacle
    bool walllr = ir1 && ir4;
    bool walll = ir1 && !ir4;
    bool wallf = ir2 || ir3;
    bool wallr = ir4 && !ir1;
    bool wallnlr = !ir4 && !ir1;
}

#define rotationSpeed 30
#define motionSpeed 150

unsigned long motorStartTime = 0;
unsigned long motorDuration = 0;

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

void runMotor(int motor, int pwm, int dir) {
    if (motor == M1) {
        if (dir == 1) {
            digitalWrite(IN1_PIN_M1, HIGH);
            digitalWrite(IN2_PIN_M1, LOW);

        } else {
            digitalWrite(IN1_PIN_M1, LOW);
            digitalWrite(IN2_PIN_M1, HIGH);
        }

        analogWrite(PWM_PIN_M1, pwm);
    } else if (motor == M2) {
        if (dir == 1) {
            digitalWrite(IN1_PIN_M2, HIGH);
            digitalWrite(IN2_PIN_M2, LOW);

        } else {
            digitalWrite(IN1_PIN_M2, LOW);
            digitalWrite(IN2_PIN_M2, HIGH);
        }

        analogWrite(PWM_PIN_M2, pwm);
    }
}

void turnLeft() {
    runMotor(M1, rotationSpeed, -1);
    runMotor(M2, rotationSpeed, 1);
}

void turnRight() {
    runMotor(M1, rotationSpeed, 1);
    runMotor(M2, rotationSpeed, -1);
}

void moveForward() {
    runMotor(M1, motionSpeed, 1);
    runMotor(M2, motionSpeed, 1);
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


#define rows 8
#define cols 8

bool visited[rows][cols];
int dx[] = {-1, 0, 1, 0};
int dy[] = {0, -1, 0, 1};
char orientation = 'U';

bool isValid(int nextRow, int nextCol, int row, int col) {

  bool ir1 = !digitalRead(IR_SENSOR_1_PIN);
  bool ir2 = !digitalRead(IR_SENSOR_2_PIN);
  bool ir3 = !digitalRead(IR_SENSOR_3_PIN);
  bool ir4 = !digitalRead(IR_SENSOR_4_PIN);

  Serial.print(ir1); Serial.print(" ");
  Serial.print(ir2); Serial.print(" ");
  Serial.print(ir3); Serial.print(" ");
  Serial.print(ir4); Serial.println();

  // Check if both IR1 and IR4 detect an obstacle
  bool walllr = ir1 && ir4;
  bool walll = ir1 && !ir4;
  bool wallf = ir2 || ir3;
  bool wallr = ir4 && !ir1;
  bool wallnlr = !ir4 && !ir1;

  if (nextRow >= 0 && nextRow < rows && nextCol >= 0 && nextCol < cols) {
    // Go Left
    if (nextCol == col - 1) {
      if (orientation == 'U') {
        bool cringe = true;
        turnLeft();
        if (wallf) {
          cringe = false;
        } 
        turnRight();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'D') {
        bool cringe = true;
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'L') {
        bool cringe = true;
        if (wallf) {
          cringe = false;
        } 
        if (cringe) {
          return false;
        }
      } else if (orientation == 'R') {
        bool cringe = true;
        turnRight();
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        turnLeft();
        if (cringe) {
          return false;
        }
      }
      return true;
    }
    // Go Right
    if (nextCol == col + 1) {
      if (orientation == 'U') {
        bool cringe = true;
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'D') {
        bool cringe = true;
        turnLeft();
        if (wallf) {
          cringe = false;
        } 
        turnRight();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'L') {
        bool cringe = true;
        turnRight();
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'R') {
        bool cringe = true;
        if (wallf) {
          cringe = false;
        } 
        if (cringe) {
          return false;
        }
      }
      return true;
    }
    // Go Up
    if (nextRow == row - 1) {
      if (orientation == 'U') {
        bool cringe = true;
        if (wallf) {
          cringe = false;
        } 
        if (cringe) {
          return false;
        }
      } else if (orientation == 'D') {
        bool cringe = true;
        turnRight();
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'L') {
        bool cringe = true;
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'R') {
        bool cringe = true;
        turnLeft();
        if (wallf) {
          cringe = false;
        } 
        turnRight();
        if (cringe) {
          return false;
        }
      }
      return true;
    }
    // Go Down
    if (nextRow == row + 1) {
      if (orientation == 'U') {
        bool cringe = true;
        turnRight();
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        turnLeft();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'D') {
        bool cringe = true;
        if (wallf) {
          cringe = false;
        } 
        if (cringe) {
          return false;
        }
      } else if (orientation == 'L') {
        bool cringe = true;
        turnLeft();
        if (wallf) {
          cringe = false;
        } 
        turnRight();
        if (cringe) {
          return false;
        }
      } else if (orientation == 'R') {
        bool cringe = true;
        turnRight();
        if (wallf) {
          cringe = false;
        } 
        turnLeft();
        if (cringe) {
          return false;
        }
      }
      return true;
    }
  } else {
    return false;
  }
}

void dfs(int row, int col) {
  visited[row][col] = 1;
  for (int k = 0; k < 4; ++k) {
    int nextRow = row + dx[k];
    int nextCol = col + dy[k];
    if (isValid(nextRow, nextCol, row, col) && !visited[nextRow][nextCol]) {
      dfs(nextRow, nextCol);
      // Go Left
      if (nextCol == col + 1) {
        if (orientation == 'U') {
          turnLeft();
          moveForward();
        } else if (orientation == 'D') {
          turnRight();
          moveForward();
        } else if (orientation == 'L') {
          moveForward();
        } else if (orientation == 'R') {
          turnLeft();
          turnLeft();
          moveForward();
        }
        orientation = 'L';
      }
      // Go Right
      if (nextCol == col - 1) {
        if (orientation == 'U') {
          turnRight();
          moveForward();
        } else if (orientation == 'D') {
          turnLeft();
          moveForward();
        } else if (orientation == 'L') {
          turnRight();
          turnRight();
          moveForward();
        } else if (orientation == 'R') {
          moveForward();
        }
        orientation = 'R';
      }
      // Go Up
      if (nextRow == row + 1) {
        if (orientation == 'U') {
          moveForward();
        } else if (orientation == 'D') {
          turnRight();
          turnRight();
          moveForward();
        } else if (orientation == 'L') {
          turnRight();
          moveForward();
        } else if (orientation == 'R') {
          turnLeft();
          moveForward();
        }
        orientation = 'U';
      }
      // Go Down
      if (nextRow == row - 1) {
        if (orientation == 'U') {
          turnLeft();
          turnLeft();
          moveForward();
        } else if (orientation == 'D') {
          moveForward();
        } else if (orientation == 'L') {
          turnLeft();
          moveForward();
        } else if (orientation == 'R') {
          turnRight();
          moveForward();
        }
        orientation = 'D';
      }
    }
  }
}

void startMaze() {
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      visited[row][col] = false;
    }
  }
  dfs(0, 0);
}

void loop() {
  startMaze();
  return;
}
