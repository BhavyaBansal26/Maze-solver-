
#include <SparkFun_TB6612.h>  // This libaray is for sparkfun motor driver
#include <QTRSensors.h>       // It is for qtr sensors

// Initialization of the motors
#define AIN1 2
#define BIN1 5
#define AIN2 3
#define BIN2 4
#define PWMA 9
#define PWMB 10
#define STBY 6


// Initialization of the controls
#define sw1 12
#define sw2 13
#define led A0

// Initialization of sensors
#define NUM_SENSORS 8
unsigned int sensorValue[8];
#define threshold 700

// User Defined Parameters
int chr = 1;  // 1 for LHS and 2 for RHS

#define MaxSpeed 150
#define BaseSpeed 100
#define TurnSpeed1 120
#define TurnSpeed2 120
#define CallibrationMode 1  // 1 for Circle and 2 for Pendulum
#define CallibrationSpeed 160

int lastError = 0;
float kp = 0.07;  // It fully depends on the bot system
float kd = 0.3;   // Please follow the method provided in instructables to get your values
int last_pos = 2000;

// shorthest path parameters
int num = 0;
char path[100];
int path_length = 0;

// Creating the instance of class for motor and senors
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
QTRSensors qtr;

void setup() {
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){ // QTR8A sensor setup
                                       A7,A6,A5,A4,A3,A2,A1,A0 },
                    NUM_SENSORS);

  pinMode(sw1, INPUT);
  pinMode(sw2, INPUT);
  pinMode(led, OUTPUT);  // Control pin setup
  pinMode(13, OUTPUT);
  pinMode(obs, INPUT);

  Serial.begin(9600);
  //  while (!Serial)
  //  {
  //    ;                           //Serial communication setup for bluetooth
  //  }
  //
  calibration();
  track_path();
  // select_mode();
  // delay(900);
  // wait_for_s1();
  // delay(800);
}



/*void select_mode() {

  while (1) {
    int s1 = digitalRead(sw1);
    int s2 = digitalRead(sw2);
    if (s1 == LOW) {
      chr = 1;
      break;  // Here we have to choose the Rule which will follow by the bot
              // S1 switch for LHS and s2 switch for RHS
              // In LHS left, straight, right will be priority
    }         // In RHS right, straight, left will be priority
    if (s2 == LOW) {
      chr = 2;
      break;
    }
  }
  Serial.println(chr);
}*/

void loop() {
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);  // For graduly increasing speed so that bot dose not get direct high speed at the start
  forward(motor1, motor2, 100);
  delay(40);
  brake(motor1, motor2);
  delay(2000);
  maze();
}
void calibration() {

  if (CallibrationMode == 1) {
    for (int i = 0; i <= 200; i++) {
      left(motor1, motor2, CallibrationSpeed);
      qtr.calibrate();
      delay(10);
    }
  }
}

bool w(int pin) {
  return sensorValue[pin] < 700;
}

bool is_left() {
  if (w(0) && w(1)) {
    return true;
  }
  return false;
}

bool is_right() {
  if (w(7) && w(6)) {
    return true;
  }
  return false;
}

bool all_black() {
  if (!w(0) && !w(1) && !w(2) && !w(3) && !w(4) && !w(5) && !w(6) && !w(7)) {
    return true;
  }
  return false;
}

bool all_white() {
  if (w(0) && w(1) && w(2) && w(3) && w(4) && w(5) && w(6) && w(7)) {
    return true;
  }
  return false;
}

bool intersection_found() {

  if (is_left()) {
    left(motor1, motor2, 100);
    delay(10);
    qtr.readLineWhite(sensorValue);
    if (is_left()) {
      return true;
    }
  } else if (is_right()) {
    right(motor1, motor2, 100);
    delay(10);
    qtr.readLineWhite(sensorValue);
    if (is_right()) {
      return true;
    }
  } else if (all_black()) {
    delay(5);
    qtr.readLineWhite(sensorValue);
    if (all_black()) {
      return true;
    }
  }
  return false;
}
void print_position() {

  Serial.println("Found an Intersection...");
  Serial.print(sensorValue[4]);
  Serial.print("\t");
  Serial.print(sensorValue[3]);
  Serial.print("\t");
  Serial.print(sensorValue[2]);
  Serial.print("\t");
  Serial.print(sensorValue[1]);
  Serial.print("\t");
  Serial.println(sensorValue[0]);

  Serial.print(w(4));
  Serial.print(w(3));
  Serial.print(w(2));
  Serial.print(w(1));
  Serial.println(w(0));
  return;
}

void follow_segment() {

  //  while(1){
  //    qtr.readLineWhite(sensorValue);
  //    Serial.print(w(0));
  //    Serial.print(w(1));
  //    Serial.print(w(2));
  //    Serial.print(w(3));
  //    Serial.println(w(4));
  //    delay(5);
  //
  //  }

  while (1) {

    digitalWrite(led, HIGH);
    // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int position = qtr.readLineWhite(sensorValue);  // Getting the present position of the bot
    int error = 2000 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    //    int motorSpeed = kp * error;

    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed)
      rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed)
      leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);

    if (intersection_found()) {
      return;
    }
    delay(5);
  }
}
void maze() {

  while (1) {
    // First main loop body were a goal is to store all the dead end or we say dry run path
    // After reaching at end we use break to get out of this

    follow_segment();  // It will follow the path until it dosn't get any intersection

    digitalWrite(led, LOW);
    //    brake(motor1, motor2); // After getting intersetion fist thing that we have to do
    // Drive straight a bit. This helps us in case we entered the
    // intersection at an angle

    //    delay(300);
    //        forward(motor1, motor2, 50);
    //        delay(10);
    //     brake(motor1, motor2);

    // These variables record whether the robot has seen a line to the
    // straight ahead, and right, whil examining the current
    // intersection.
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    // Now read the sensors and check the intersection type.
    //  qtr.readLineWhite(sensorValue);
    print_position();
    if (w(4)) {
      found_left = 1;
    }
    if (w(0)) {
      found_right = 1;
    }

    // Drive straight a bit more - this is enough to line up our
    // wheels with the intersection.

    forward(motor1, motor2, 60);
    delay(300);
    brake(motor1, motor2);
    qtr.readLineWhite(sensorValue);
    if (w(1) || w(2) || w(3)) {
      found_straight = 1;
    }

    // Check for the ending spot.
    if (all_white()) {
      //    Yeh break ko uncomment karna hai

      forward(motor1, motor2, 60);
      delay(300);
      brake(motor1, motor2);

      if (all_white())
        break;
    }

    // According to rule we have to decided which turn has to be taken
    if (chr == 1)
      dir = select_turnL(found_left, found_straight, found_right);
    else if (chr == 2)
      dir = select_turnR(found_right, found_straight, found_left);
    Serial.println(dir);

    // Take a turn according to that
    turn(dir);

    // Store the intersection in the path variable.
    path[path_length] = dir;
    path_length++;

    // Simplify the path for the final run
    simplify_path();
  }
  // Now the second half is the short run which is obtained after sorting the dry run

  brake(motor1, motor2);
  //  delay(40000);
  // Move straight a bit on end point and glow a led
  forward(motor1, motor2, 80);
  brake(motor1, motor2);
  for (int w = 0; w < path_length; w++) {
    Serial.print(path[w]);
    Serial.print(' ');
  }
  digitalWrite(led, LOW);
  delay(4000);
  digitalWrite(led, HIGH);
  delay(800);
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  forward(motor1, motor2, 100);
  delay(40);

  // Final Run
  int k;
  for (k = 0; k < path_length; k++) {
    follow_segment();
    forward(motor1, motor2, 50);  // After reaching a intercetion follow the shortest path turn
    delay(50);
    forward(motor1, motor2, 60);
    delay(200);
    brake(motor1, motor2);
    delay(5);
    turn(path[k]);
  }
  follow_segment();
  brake(motor1, motor2);
  forward(motor1, motor2, 80);
  delay(200);
  brake(motor1, motor2);

  if (all_white()) {
    forward(motor1, motor2, 140);
    delay(200);
    brake(motor1, motor2);
    qtr.readLineWhite(sensorValue);
    if (all_white()) {
      digitalWrite(led, LOW);
    }
  }
  // Go back to a starting of the main loop
}
char select_turnL(char found_left, char found_straight, char found_right) {
  // Make a decision about how to turn. The following code
  // Implements a left-hand-on-the-wall strategy, where we always preffer left turn
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}
char select_turnR(char found_right, char found_straight, char found_left) {
  // Make a decision about how to turn. The following code
  // Implements a right-hand-on-the-wall strategy, Where we always preffer right turn
  if (found_right)
    return 'R';
  else if (found_straight)
    return 'S';
  else if (found_left)
    return 'L';
  else
    return 'B';
}
void turn(char dir) {
  int line_position;
  // According to dir perform a turn
  switch (dir) {
    case 'L':
      left(motor1, motor2, TurnSpeed1);
      qtr.readLineWhite(sensorValue);
      while (w(4)) {
        line_position = qtr.readLineWhite(sensorValue);  // Move left until left extreme sensor goes out of the white line comes on black
      }
      left(motor1, motor2, TurnSpeed2);
      qtr.readLineWhite(sensorValue);
      while (!w(4)) {
        line_position = qtr.readLineWhite(sensorValue);  // Again move left until it get's on white line
      }
      follow_segment1();  // Fast pid after turn to align bot quickly on line
      brake(motor1, motor2);
      break;
    case 'R':
      right(motor1, motor2, TurnSpeed1);
      qtr.readLineWhite(sensorValue);
      while (w(0)) {
        line_position = qtr.readLineWhite(sensorValue);  // Move right until left extreme sensor goes out of the white line comes on black
      }
      right(motor1, motor2, TurnSpeed2);
      qtr.readLineWhite(sensorValue);
      while (!w(0)) {
        line_position = qtr.readLineWhite(sensorValue);  // Again move right until it get's on white line
      }
      follow_segment1();  // Fast pid after turn to align a bot quickly on line
      brake(motor1, motor2);
      break;
    case 'B':
      right(motor1, motor2, TurnSpeed1);
      qtr.readLineWhite(sensorValue);
      while (w(0)) {
        line_position = qtr.readLineWhite(sensorValue);  // Take u turn using right turn
      }
      right(motor1, motor2, TurnSpeed2);
      qtr.readLineWhite(sensorValue);
      while (!w(0)) {
        line_position = qtr.readLineWhite(sensorValue);
      }

      // Next Three lines are only for testing purpose

      follow_segment1();  // Fast pid after turn to align a bot quickly on line
      brake(motor1, motor2);
      break;

      follow_segment3();
      brake(motor1, motor2);
      delay(50);
      follow_segment2();  // Back moving pid
      forward(motor1, motor2, 40);
      delay(40);
      break;
  }
}
void follow_segment1() {
  // Fast pid after turn to get quickly align a bot on line
  int Kp = 1;
  int Kd = 10;
  for (int j = 0; j < 60; j++) {
    int position = qtr.readLineWhite(sensorValue);  // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error = 2000 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed)
      rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed)
      leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(2);
  }
}
void track_path() {
  // Fast pid after turn to get quickly align a bot on line
  int Kp = 1;
  int Kd = 10;
  for (int j = 0; j < 60; j++) {
    int position = qtr.readLineWhite(sensorValue);  // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error = 2000 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = 0 - motorSpeed;
    int leftMotorSpeed = 0 + motorSpeed;
    if (rightMotorSpeed > MaxSpeed)
      rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed)
      leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(2);
  }
}
void simplify_path() {
  // Path simplification. The strategy is that whenever we encounter a
  // sequence xBx, we can simplify it by cutting out the dead end.

  if (path_length < 3 || path[path_length - 2] != 'B')  // simplify the path only if the second-to-last turn was a 'B'
    return;
  int total_angle = 0;
  int m;
  // Get the angle as a number between 0 and 360 degrees.
  for (m = 1; m <= 3; m++) {
    switch (path[path_length - m]) {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
  // Replace all of those turns with a single one.
  total_angle = total_angle % 360;
  switch (total_angle) {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}
void follow_segment2() {
  // Back moving pid
  int Kp = 1;
  int Kd = 50;
  int baseSpeed = 90;
  int maxSpeed = 90;
  for (int j = 0; j < 70; j++) {
    int position = qtr.readLineWhite(sensorValue);  // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error = 2000 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = baseSpeed + motorSpeed;
    int leftMotorSpeed = baseSpeed - motorSpeed;
    if (rightMotorSpeed > maxSpeed)
      rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed)
      leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    motor1.drive(-rightMotorSpeed);
    motor2.drive(-leftMotorSpeed);
    delay(1);
  }
}

void follow_segment3() {
  int Kp = 1;
  int Kd = 10;
  for (int j = 0; j < 10; j++) {
    int position = qtr.readLineWhite(sensorValue);  // FOR BLACK LINE FOLLOWER JUST REPLACE White WITH Black
    int error = 2000 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    if (rightMotorSpeed > MaxSpeed)
      rightMotorSpeed = MaxSpeed;
    if (leftMotorSpeed > MaxSpeed)
      leftMotorSpeed = MaxSpeed;
    if (rightMotorSpeed < 0)
      rightMotorSpeed = 0;
    if (leftMotorSpeed < 0)
      leftMotorSpeed = 0;
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}
