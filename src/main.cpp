#define Motion

#define Debug
//#define Production

#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "common.h"

#include <debug.h>
#include "servo_motors.h"
#include "communications.h"
#include "stdint.h"
#include "MedianFilterLib.h"
#include "PID_v1.h"

#define myID 5 //Robot ID
#define TEAMCOL 3 //Team Cylinder Color
#define SPEEDLIMIT 100 //Speed Limit for the motors as a percent
#define HOMEX 220 //Home location
#define HOMEY 220 //Home location
#define BUFFLENGTH 5//Buffer Length for Median Filter
#define IRDISTBUFFLENGTH 20//Buffer Length for Median Filter
#define IRSENSORPIN 36  // IR Sensor Pin
#define SWEEPANGLE 90 // Sweep angle for the servo
#define NUMBALLS 3 // Number of balls in the field
#define pinSW 39 //limit switch pin
#define IRSENSORENABLE 16 // IR Sensor Enable Pin

#define GRID_WIDTH 10
#define GRID_HEIGHT 10
#define MAX_NODES 100



/* -------------------------------------------------------------------------- */
/*                                  Functions                                 */
/* -------------------------------------------------------------------------- */
int updateRobotPosition();
void setupLimitInterupt();
void IRAM_ATTR onSwitchInterupt();
void setupEncoderInterupt();
void IRAM_ATTR onEncoderTimer();
void setupPIDInterupt();
void IRAM_ATTR onPIDTimer();
void setupMotor();
void DCMotorCalibration();

int goToLocation(float dx, float dy);
void hunting();
void defending();
void capturing();
void goToHome();

double* transformRobotPos(uint16_t X, uint16_t Y);
void open(Servo* clawservo);
void close(Servo* clawservo);
void DCMotorCalibration();
void calibrateIRSensor();
BallPosition* updateClosestCylinder();


/* -------------------------------------------------------------------------- */
/*                                  Variables                                 */
/* -------------------------------------------------------------------------- */
typedef unsigned short uShort;
static RobotPose myPose;
BallPosition currentBallPoss[ 20 ];
Servo clawServo;


uShort ballNum; //number of balls/cylinders in the field
uShort x = 0;  //x location of robot in pixels
uShort y = 0;  //y location of robot in pixels
float theta = 0; //rotation of robot in radians
float d_x = 0, d_y = 0; //x and y location of the target
float omega_1, omega_2, trueOmega_1, trueOmega_2;

float error_x = 0;
float error_y = 0;
float error_d = 0;
float error_theta = 0;

float prev_error_d = 0;
float prev_error_theta = 0;

const float Kp1 = 0.6;//1
const float Kd1 = 1;//1.1

const float Kp2 = 9.9;//12
const float Kd2 = 2.55;//1.5


static uShort IRSensorValue = 0; // IR Sensor Value

const uShort PWMfreq = 5000; // PWM frequency
const uShort PWMresolution = 12; // PWM resolution
const int maxDutyCycle = (int)(pow(2, PWMresolution) - 1);; // Max duty cycle for the PWM
int pidSampleTime = 10; // milliseconds

const int motCH1 = 4, motCH2 = 5; // Motor channels
const char enCHApin = 14, enCHBpin = 27;
const char motIN1pin = 13, motIN2pin = 12; // Motor IN pins
volatile long position = 0; // Encoder position
volatile bool lastEncA = 0, lastEncB = 0;
volatile bool newEncA = 0, newEncB = 0;
volatile bool error;
volatile bool isSwitchDown = false;

double input = 0, output = 0, setpoint = 60, angle = 0;
double kp = 135, ki = 16, kd = 0.68;//1.5;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

hw_timer_t* encoderTimer = NULL;
hw_timer_t* PIDtimer = NULL;
hw_timer_t* switchTimer = NULL;

MedianFilter<uShort>robotXFilter(BUFFLENGTH);
MedianFilter<uShort>robotYFilter(BUFFLENGTH);
MedianFilter<float>robotThetaFilter(BUFFLENGTH);
MedianFilter<uShort>IRSensorMagnitudeFilter(IRDISTBUFFLENGTH);

typedef signed char Byte;
struct Node {
  Byte x;
  Byte y;
  uShort g;        // Cost from start to current node
  uShort h;        // Heuristic (estimated cost from current to goal)
  uShort f;        // Total cost (g + h)
  Byte parent;  // Index of parent node in the closed list
};

Byte grid[ GRID_HEIGHT ][ GRID_WIDTH ];

// Open and closed lists
Node openList[ MAX_NODES ];
Node closedList[ MAX_NODES ];
Byte openCount = 0;
Byte closedCount = 0;

// Path storage
Byte pathX[ MAX_NODES ];
Byte pathY[ MAX_NODES ];
Byte pathLength = 0;

void setupGrid();
bool findPath(Byte startX, Byte startY, Byte goalX, Byte goalY);
Byte findNodeWithLowestF();
bool isNodeInList(Node* list, Byte count, Byte x, Byte y, Byte* index);
void reconstructPath(Byte goalX, Byte goalY);
int calculateHeuristic(Byte x1, Byte y1, Byte x2, Byte y2);
void printPath();

// MARK: SETUP
/* -------------------------------------------------------------------------- */
/*                                    Setup                                   */
/* -------------------------------------------------------------------------- */
void setup() {
  //---------General Setup---------
  Serial.begin(115200);
  setupServos();

#ifdef Production
  setupCommunications();
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_printf(...)
#else
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.print(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#define D_printf(...)  Serial.printf(__VA_ARGS__)
#endif


#ifdef Motion
  setupMotor();
#endif

/* ------------------------------ Setup Sensors ----------------------------- */
//---------Drive Servo Setup---------
  servo3.attach(SERVO2_PIN, 1300, 1700);
  servo4.attach(SERVO1_PIN, 1300, 1700);
  clawServo.attach(SERVO3_PIN);


  //---------DC Motor Setup---------
  pinMode(enCHApin, INPUT);
  pinMode(enCHBpin, INPUT);
  pinMode(pinSW, INPUT);
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(pidSampleTime);
  myPID.SetOutputLimits(-4095, 4095);

  //---------Timer Interupts---------
  setupEncoderInterupt();
  setupPIDInterupt();
  setupLimitInterupt();

  //---------IR Sensor Setup---------
  pinMode(IRSENSORENABLE, OUTPUT);

  //---------Path Setup---------
  /*
  setupGrid();
  if (findPath(0, 0, 9, 9)) {
    Serial.println("Path found!");
    printPath();
  } else {
    Serial.println("No path found!");
  }
    */
  delay(1000);
}

/* -------------------------------------------------------------------------- */
/*                                  Main Loop                                 */
/* -------------------------------------------------------------------------- */
//MARK: LOOP

void loop() {
  //hunting();
  //capturing();
  //calibrateIRSensor();
  DCMotorCalibration();


}


//-------------------STATES----------------------------------

void hunting() {
  updateRobotPosition();
  BallPosition* closestBall = updateClosestCylinder();
  d_x = closestBall->x;
  d_y = closestBall->y;
  goToLocation(d_x, d_y);
  return;


}
void defending() {
  //fuck this shit
  //Center backs dont win the ballon d'Or
}
void capturing() {
  goToLocation(HOMEX, HOMEY);
}
// MARK: PATHFINDING
/* ------------------------------- Pathfinding ------------------------------ */
void setupGrid() {
  // Clear grid
  for (Byte y = 0; y < GRID_HEIGHT; y++) {
    for (Byte x = 0; x < GRID_WIDTH; x++) {
      grid[ y ][ x ] = 0;  // Open space
    }
  }

  // Add some obstacles (1 = obstacle)
  grid[ 1 ][ 2 ] = 1;
  grid[ 2 ][ 2 ] = 1;
  grid[ 3 ][ 2 ] = 1;
  grid[ 3 ][ 3 ] = 1;
  grid[ 3 ][ 4 ] = 1;
  grid[ 6 ][ 5 ] = 1;
  grid[ 6 ][ 6 ] = 1;
  grid[ 6 ][ 7 ] = 1;
  grid[ 7 ][ 7 ] = 1;
  grid[ 8 ][ 7 ] = 1;
}

// Calculate Manhattan distance heuristic
int calculateHeuristic(Byte x1, Byte y1, Byte x2, Byte y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Find the node with the lowest F score in the open list
Byte findNodeWithLowestF() {
  Byte lowestIndex = 0;
  int lowestF = openList[ 0 ].f;

  for (Byte i = 1; i < openCount; i++) {
    if (openList[ i ].f < lowestF) {
      lowestF = openList[ i ].f;
      lowestIndex = i;
    }
  }

  return lowestIndex;
}

// Check if a node exists in a list and return its index
bool isNodeInList(Node* list, Byte count, Byte x, Byte y, Byte* index) {
  for (Byte i = 0; i < count; i++) {
    if (list[ i ].x == x && list[ i ].y == y) {
      *index = i;
      return true;
    }
  }
  *index = 0;
  return false;
}

// Reconstruct the path from start to goal
void reconstructPath(Byte goalX, Byte goalY) {
  pathLength = 0;

  // Find the goal node in the closed list
  Byte index = 0;
  Byte currentIndex = 0;

  if (isNodeInList(closedList, closedCount, goalX, goalY, &currentIndex)) {
    while (true) {
      // Add current node to path
      pathX[ pathLength ] = closedList[ currentIndex ].x;
      pathY[ pathLength ] = closedList[ currentIndex ].y;
      pathLength++;

      // Check if we reached the start node (parent index is itself)
      if (closedList[ currentIndex ].parent == currentIndex) {
        break;
      }

      // Move to parent node
      currentIndex = closedList[ currentIndex ].parent;
    }
  }
}

// Main A* algorithm function
bool findPath(Byte startX, Byte startY, Byte goalX, Byte goalY) {
  // Clear the lists
  openCount = 0;
  closedCount = 0;

  // Add start node to open list
  openList[ openCount ].x = startX;
  openList[ openCount ].y = startY;
  openList[ openCount ].g = 0;
  openList[ openCount ].h = calculateHeuristic(startX, startY, goalX, goalY);
  openList[ openCount ].f = openList[ openCount ].g + openList[ openCount ].h;
  openList[ openCount ].parent = 0;  // Parent is itself for start node
  openCount++;

  while (openCount > 0) {
    // Find node with lowest F score
    Byte currentIndex = findNodeWithLowestF();
    Byte currentX = openList[ currentIndex ].x;
    Byte currentY = openList[ currentIndex ].y;

    // If goal reached, reconstruct path and return success
    if (currentX == goalX && currentY == goalY) {
      // Move the goal node to closed list first
      closedList[ closedCount ] = openList[ currentIndex ];
      closedCount++;

      // Remove goal node from open list
      openList[ currentIndex ] = openList[ openCount - 1 ];
      openCount--;

      reconstructPath(goalX, goalY);
      return true;
    }

    // Move current node from open list to closed list
    closedList[ closedCount ] = openList[ currentIndex ];
    Byte closedCurrentIndex = closedCount;
    closedCount++;

    // Remove current node from open list
    openList[ currentIndex ] = openList[ openCount - 1 ];
    openCount--;

    // Check all adjacent nodes (4-directional movement)
    const Byte dx[ 4 ] = { 0, 1, 0, -1 };  // Right, Down, Left, Up
    const Byte dy[ 4 ] = { 1, 0, -1, 0 };

    for (Byte i = 0; i < 4; i++) {
      Byte newX = currentX + dx[ i ];
      Byte newY = currentY + dy[ i ];

      // Check if new position is valid
      if (newX >= GRID_WIDTH || newY >= GRID_HEIGHT) {
        continue;  // Outside grid bounds
      }

      // Check if position is walkable
      if (grid[ newY ][ newX ] == 1) {
        continue;  // Obstacle
      }

      // Check if node is in closed list
      Byte tempIndex;
      if (isNodeInList(closedList, closedCount, newX, newY, &tempIndex)) {
        continue;  // Skip if already in closed list
      }

      // Calculate G score for this path
      int newG = closedList[ closedCurrentIndex ].g + 1;  // Assuming cost of 1 to move

      // Check if node is already in open list
      bool inOpenList = isNodeInList(openList, openCount, newX, newY, &tempIndex);

      if (!inOpenList || newG < openList[ tempIndex ].g) {
        // This path is better, update or add node
        if (!inOpenList) {
          // Make sure we don't exceed the array size
          if (openCount >= MAX_NODES) {
            continue;  // Skip if we reached the maximum number of nodes
          }

          tempIndex = openCount;
          openCount++;
        }

        // Update or add node to open list
        openList[ tempIndex ].x = newX;
        openList[ tempIndex ].y = newY;
        openList[ tempIndex ].g = newG;
        openList[ tempIndex ].h = calculateHeuristic(newX, newY, goalX, goalY);
        openList[ tempIndex ].f = openList[ tempIndex ].g + openList[ tempIndex ].h;
        openList[ tempIndex ].parent = closedCurrentIndex;
      }
    }
  }

  // No path found
  return false;
}

// Print the found path
void printPath() {
  Serial.print("Path length: ");
  Serial.println(pathLength);

  // Print in reverse order (from goal to start)
  for (int i = pathLength - 1; i >= 0; i--) {
    Serial.print("(");
    Serial.print(pathX[ i ]);
    Serial.print(",");
    Serial.print(pathY[ i ]);
    Serial.println(")");
  }

  // Print grid with path
  Serial.println("Grid with path (S=start, G=goal, #=obstacle, .=path, _=open):");

  for (Byte y = 0; y < GRID_HEIGHT; y++) {
    for (Byte x = 0; x < GRID_WIDTH; x++) {
      bool isOnPath = false;

      // Check if this position is on the path
      for (Byte i = 0; i < pathLength; i++) {
        if (pathX[ i ] == x && pathY[ i ] == y) {
          isOnPath = true;
          break;
        }
      }

      // Print appropriate character
      if (x == 0 && y == 0) {
        Serial.print("S");  // Start
      } else if (x == 9 && y == 9) {
        Serial.print("G");  // Goal
      } else if (grid[ y ][ x ] == 1) {
        Serial.print("#");  // Obstacle
      } else if (isOnPath) {
        Serial.print(".");  // Path
      } else {
        Serial.print("_");  // Open space
      }
    }
    Serial.println();
  }
}

int updateRobotPosition() {
  myPose = getRobotPose(myID);
  if (error_d > 0 && error_d < 200) {
    open(&clawServo);
  } else {
    close(&clawServo);
  }

  if (myPose.valid) {
    x = robotXFilter.AddValue(myPose.x);
    y = robotYFilter.AddValue(myPose.y);
    theta = robotThetaFilter.AddValue(myPose.theta);
    return 1;
  } else {
    Serial.println("Invalid Pose");
    return 0;
  }
}
// MARK: Go To Location

int goToLocation(float dx, float dy) {

  bool atLocation = false;

  while (!atLocation) {
    error_x = dx - x;
    error_y = dy - y;
    error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
    error_theta = (atan2(error_y, error_x) - (theta / 1000.0f)) * (180.0f / (PI)) - 90.0f;

    if (error_theta < -180) {
      error_theta = error_theta + 360;
    } else if (error_theta > 180) {
      error_theta = error_theta - 360;
    }

    //PID controller for left and right motor
    omega_1 = 0.2 * (Kp1 * error_d - Kp2 * error_theta + Kd1 * (error_d - prev_error_d) - Kd2 * (error_theta - prev_error_theta));
    omega_2 = 0.2 * (Kp1 * error_d + Kp2 * error_theta + Kd1 * (error_d - prev_error_d) + Kd2 * (error_theta - prev_error_theta));

    //limit PID controller
    omega_1 = constrain(omega_1, -SPEEDLIMIT, SPEEDLIMIT);
    omega_2 = constrain(omega_2, -SPEEDLIMIT, SPEEDLIMIT);

    trueOmega_1 = map(omega_1, -100, 100, 1300, 1700);
    trueOmega_2 = map(-omega_2, -100, 100, 1300, 1700);

    //write the microseconds to the servos
    servo3.writeMicroseconds(trueOmega_1);
    servo4.writeMicroseconds(trueOmega_2);

    D_print(d_x);
    D_print(" ");
    D_print(d_y);
    D_print("   ");
    D_print(x);
    D_print(" ");
    D_print(y);
    D_print("   ");
    D_print(omega_1);
    D_print(" ");
    D_print(omega_2);
    D_print("   ");
    D_print(error_d);
    D_print("   ");
    D_println(error_theta);

    if (error_d < 150) {
      servo3.writeMicroseconds(1500);
      servo4.writeMicroseconds(1500);
      atLocation = true;
      return 1;
    }
    prev_error_d = error_d;
    prev_error_theta = error_theta;
  }
  return 0;
}

BallPosition* updateClosestCylinder() {
  static BallPosition closestBall;

  bool inReach = false;
  int ballnum = 0;

  while (inReach == false) {
    double minDist = INFINITY;

    getBallPositions(currentBallPoss);
    updateRobotPosition();
    for (int i = 0; i < NUMBALLS; i++) {
      if (currentBallPoss[ i ].hue == TEAMCOL) {
        int dx = currentBallPoss[ i ].x - x;
        int dy = currentBallPoss[ i ].y - y;
        int dist = sqrt(dx * dx + dy * dy);
        if (dist < minDist) {
          minDist = dist;
          closestBall = currentBallPoss[ i ];
          ballnum = i;
        }
      }
    }

  }
  return &closestBall;
}

void huntingClosestBall() {
  BallPosition* closestBall = updateClosestCylinder();
  d_x = closestBall->x;
  d_y = closestBall->y;
  goToLocation(d_x, d_y);


}

void DCMotorCalibration() {

  D_print("Setpoint: ");    D_print(setpoint); D_print("  ");
  D_print("Measured : ");   D_print(input);    D_print("  ");
  D_print("PWM Output: ");  D_print(output);   D_print("  ");
  D_print("Angle: ");      D_print(angle);    D_print("  ");
  D_println("");

  delay(100);
}

void calibrateIRSensor() {
  digitalWrite(IRSENSORENABLE, HIGH);
  IRSensorValue = analogRead(IRSENSORPIN);
  double val = IRSensorMagnitudeFilter.AddValue(IRSensorValue);
  Serial.println(IRSensorValue);
  delay(50);
  //Eqtn = 10209*x^(-0.738)
}
uint16_t getIRSensorDistance() {
  digitalWrite(IRSENSORENABLE, HIGH);
  IRSensorValue = IRSensorMagnitudeFilter.AddValue(analogRead(IRSENSORPIN));
  return (uint16_t)(10209 * pow(IRSensorValue, -0.738));

  //Eqtn = 10209*x^(-0.738)

}
void setupMotor() {
  pinMode(enCHApin, INPUT_PULLDOWN);
  pinMode(enCHBpin, INPUT_PULLDOWN);
  ledcSetup(motCH1, PWMfreq, PWMresolution);
  ledcSetup(motCH2, PWMfreq, PWMresolution);
  ledcAttachPin(motIN1pin, motCH1);
  ledcAttachPin(motIN2pin, motCH2);
  ledcWrite(motCH1, 0);
  ledcWrite(motCH2, 0);
}

void setupEncoderInterupt() {
  encoderTimer = timerBegin(0, 80, true);  // timer 0, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(encoderTimer, &onEncoderTimer, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(encoderTimer, 10, true);  // 10 microsecond timer interrupt
  timerAlarmEnable(encoderTimer);
}
void IRAM_ATTR onEncoderTimer() {
  newEncA = digitalRead(enCHApin); //read encoder value
  newEncB = digitalRead(enCHBpin); //read encoder value
  position += (newEncA ^ lastEncB) - (lastEncA ^ newEncB); // determine new position from encoder readings
  if ((lastEncA ^ newEncA) & (lastEncB ^ newEncB)) {
    error = true;
  }
  lastEncA = newEncA;
  lastEncB = newEncB;

}

void setupPIDInterupt() {

  PIDtimer = timerBegin(1, 80, true);  // timer 1, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(PIDtimer, &onPIDTimer, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(PIDtimer, 10000, true);  // 10 millisecond timer interrupt
  timerAlarmEnable(PIDtimer);

}

void IRAM_ATTR onPIDTimer() {

  input = position / (2940.0 / 360.0);
  setpoint = (input < -60) || (input > 60) ? -setpoint : setpoint;
  myPID.Compute();
  if (output > 0) { // drive motor based off pid output
    ledcWrite(motCH1, abs(output));
    ledcWrite(motCH2, 0);
  } else {
    ledcWrite(motCH2, abs(output));
    ledcWrite(motCH1, 0);
  }

}

void setupLimitInterupt() {
  switchTimer = timerBegin(2, 80, true);  // timer 0, prescalewr of 80 give 1 microsecond tiks
  timerAttachInterrupt(switchTimer, &onSwitchInterupt, true); // connect interrupt function to hardware with pointer
  timerAlarmWrite(switchTimer, 10000, true);  // 10 microsecond timer interrupt
  timerAlarmEnable(switchTimer);

}
void IRAM_ATTR onSwitchInterupt() {
  isSwitchDown = digitalRead(pinSW);
  if (isSwitchDown == true) {
    Serial.println("Sum Behind u Dipshit");
  }
}

//Open the claw
void open(Servo* clawservo) {
  clawservo->write(20);  //Figure out OPEN AND CLOSE VALUES*****
  delay(15);
  return;
}

//Close the claw
void close(Servo* clawservo) {
  clawservo->write(85);
  delay(15);
  return;
}