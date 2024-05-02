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

#define SIZE 8

#define R 3
#define C 5
#define NORTH 0
#define SOUTH 2
#define WEST 1
#define EAST 3

struct cell {
    int x, y;
};

struct CircularQueue {
    cell queue[SIZE*SIZE]{};
    int head = 0, tail = 0;

    bool empty() const {
        return head == tail;
    }

    void push(cell c) {
        queue[tail] = c;
        tail = (tail+1) % (SIZE*SIZE);
    }

    cell pop() {
        cell c = queue[head];
        head = (head + 1 + SIZE*SIZE) % (SIZE*SIZE);
        return c;
    }
};

int flood[SIZE][SIZE]{};
int wall[SIZE][SIZE]{};

cell position;
int current;

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

    // Adjust motor 1 based on obstacle detection
    if (wallf) {
        if(walllr){
            stopMotor(M1);
            stopMotor(M2);
            Serial.println("Wall in all sides");
        }if(walll){
            turn_right();
            Serial.println("Wall on left and front");
        }if(wallr){
            turn_left();
            Serial.println("Wall on right and front");
        }if(wallnlr){
            turn_left();
            Serial.println("Wall on front only");
        }
    } else if (walllr || wallnlr) {
        move_forward();
        Serial.println("Wall on neither side or on both sides");
    } else if (walll) {
        turn_right();
        Serial.println("Wall on left");
    } else if (wallr) {
        turn_left();
        Serial.println("Wall on right");
    }
}

void turn_left() {
    runMotor(M1, 60, -1);
    runMotor(M2, 60, 1);
}

void turn_right() {
    runMotor(M1, 60, 1);
    runMotor(M2, 60, -1);
}

void move_forward() {
    runMotor(M1, 255, 1);
    runMotor(M2, 255, 1);
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

cell get_next(cell c, int direction) {
  switch (direction) {
      case NORTH: return {c.x, c.y + 1};
      case SOUTH: return {c.x, c.y - 1};
      case WEST: return {c.x + 1, c.y};
      case EAST: return {c.x - 1, c.y};
      default: return {-1, -1};
  }
}

cell make_move(int direction) {
  int a_clk = (direction - current + 4) % 4;
  int clk = (current - direction + 4) % 4;

  if (a_clk < clk) {
      while (a_clk--)
          turn_left();

  } else {
      while (clk--)
          turn_right();
  }

  move_forward();
  current = direction;
  position = get_next(position, direction);
}

bool is_valid(cell c, int k) {
    cell d = get_next(c, k);
    return (0 <= d.x && d.x < SIZE) &&
          (0 <= d.y && d.y < SIZE) && !(wall[c.x][c.y] >> k & 1);
}

// adds a wall to c in direction of k
void add_wall(cell c, int k) {
    wall[c.x][c.y] |= 1 << k;
    if (is_valid(c, k)) {
        cell d = get_next(c, k);
        wall[d.x][d.y] |= 1 << ((k + 2) % 4);
    }
}

// adds all walls for current cell
void update_walls() {
    if (front_wall()) add_wall(position, current);
    if (left_wall()) add_wall(position, current);
    if (right_wall()) add_wall(position, current);
}

void update_flood() {
    for (auto &i: flood)
        for (int &j: i)
            j = 10000;

    flood[R][C] = 0;

    CircularQueue q;
    q.push({R, C});

    while (!q.empty()) {
        cell c = q.pop();
        for (int k = 0; k < 4; k++) {
            if (is_valid(c, k)) {
                cell d = get_next(c, k);
                if (flood[d.x][d.y] > flood[c.x][c.y] + 1) {
                    flood[d.x][d.y] = flood[c.x][c.y] + 1;
                    q.push(d);
                }
            }
        }
    }
}

void solver(cell c) {
    while (flood[c.x][c.y] != 0) {
        update_walls();
        update_flood();

        int min_flood = 10000, direction = 0;
        for (int k = 0; k < 4; k++) {
            if (is_valid(c, k)) {
                cell d = get_next(c, k);
                if (flood[d.x][d.y] < min_flood) {
                    min_flood = flood[d.x][d.y];
                    direction = k;
                }
            }
        }

        c = get_next(c, direction);
    }
}
