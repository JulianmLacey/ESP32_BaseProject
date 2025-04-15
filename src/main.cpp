#include <debug.h>
#include <Arduino.h> // This is needed for the ESP32 to work with base Arduino libraries
#include "debug.h"
#include "common.h"
#include "dc_motors.h"
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
#define IRSENSORPIN 36  // IR Sensor Pin
#define IRSENSORENABLE 10 // IR Sensor Enable Pin
#define NUMBALLS 3 // Number of balls in the field
#define pinSW 21 //limit switch pin
//----------------------------------FUNCS----------------------------------

void hunting();
void defending();
void capturing();
void goToHome();
int goToLoction();
double* transformRobotPos(uint16_t X, uint16_t Y);
void open(Servo* clawservo);
void close(Servo* clawservo);

//----------------------------------VARS----------------------------------
/*
RobotPose: rotation, and location of the robot.
  .x = x location in pixels
  .y = y location in pixels
  .theta = rotation in radians
  .valid = true if the robot is in a valid location
*/
int wallCertaintyMatrix[ 5 ][ 5 ] = { {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}, { 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0 } }; // Wall Certainty Matrix

 // Example obstacles
RobotPose myPose;
BallPosition currentBallPoss[ 20 ];

/*
clawServo: servo object for the robot claw
  .attach() = attaches the servo to the pin
  .write() = sets the position of the servo
*/
Servo clawServo;


int ballNum; //number of balls/cylinders in the field
double x = 0;  //x location of robot in pixels
double y = 0;  //y location of robot in pixels
double theta = 0; //rotation of robot in radians
/*
omega1 & omega2: speed of the left and right motors from -100 - 0 - 100 (100 = full speed forward, -100 = full speed backward)
  omega1 = speed of the left motor
  omega2 = speed of the right motor
  (to help understand the robot's movement)


  trueOmega1 & trueOmega2: speed of the left and right motors mapped to microseconds(1300 - 1700) for the motorshield
*/
double omega_1, omega_2, trueOmega_1, trueOmega_2;

/*
  Error x&y: the difference between the robot's location and the target location
  Error d: the distance between the robot and the target location
  Error theta: the difference between the robot's rotation and the vector between the robot and the target location
*/
double error_x = 0;
double error_y = 0;
double error_d = 0;
double error_theta = 0;


/*
  prev_error_d & prev_error_theta: the previous error values for the PID controller
  Kp1, Kd1, Kp2, Kd2: the PID controller constants
*/
double prev_error_d = 0;
double prev_error_theta = 0;

const double Kp1 = 1;//1
const double Kd1 = 2.7;//1.1

const double Kp2 = 19;//12
const double Kd2 = 1.95;//1.5


static uint8_t IRSensorValue = 0; // IR Sensor Value

/*
Median Filters for sensor values
*/
MedianFilter<double>robotXFilter(BUFFLENGTH);
MedianFilter<double>robotYFilter(BUFFLENGTH);
MedianFilter<double>robotThetaFilter(BUFFLENGTH);
MedianFilter<double>IRSensorMagnitudeFilter(BUFFLENGTH);
/*
hw_timer_t* switchTimer = NULL;
*/


//-------------------SETUP----------------------------------


//void setupSwitchInterupt() {
  //pinMode(pinSW, INPUT);
  //attachInterrupt(digitalPinToInterrupt(pinSW), onSwitchInterupt, FALLING);
//}
//-------------------STATES----------------------------------

void hunting() {}
void defending() {}
void capturing() {}

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

double* transformRobotPos(uint16_t X, uint16_t Y) {
  static double robotPos[ 2 ];
  //static double* robotPos = pos[1];

  double m1 = 1.335, m2 = 1.33, b1 = 384.475, b2 = 328.15;

  robotPos[ 0 ] = (m1 * X) - b1;
  robotPos[ 1 ] = (m2 * Y) - b2;

  return robotPos;
}

void goToHome() {
  error_d =
    prev_error_d = 0;
  prev_error_theta = 0;
  bool isHome = false;

  while (isHome == false) {
    RobotPose myPose = getRobotPose(myID);
    if (myPose.valid == true) {
      double* robotPos = transformRobotPos(myPose.x, myPose.y);
      x = robotXFilter.AddValue(robotPos[ 0 ]);
      y = robotYFilter.AddValue(robotPos[ 1 ]);

      theta = robotThetaFilter.AddValue(myPose.theta);


    } else {
      Serial.println("Invalid Pose");
     // return;
    }

    double d_x = HOMEX; //home x location
    double d_y = HOMEY; //home y location
    error_x = d_x - x;
    error_y = d_y - y;
    error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
    error_theta = (atan2(error_y, error_x) - (theta / 1000)) * (180 / (PI)) - 90;

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
      delay(1000);
      open(&clawServo);
      isHome = true;
      while (1) {}
      //return 1;
    }
    prev_error_d = error_d;
    prev_error_theta = error_theta;

  }
  //return 0;
}

int goToLoction() {
  open(&clawServo);
  BallPosition closestBall;
  RobotPose myPose = getRobotPose(myID);
  bool inReach = false;
  int ballnum = 0;
  while (inReach == false) {
    double minDist = INFINITY;

    getBallPositions(currentBallPoss);
    myPose = getRobotPose(myID);
    for (int i = 0; i < 3; i++) {
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

    //if valid pose, update x, y, and theta to new location
    if (myPose.valid == true) {
      double* robotPos = transformRobotPos(myPose.x, myPose.y);
      x = robotPos[ 0 ];
      y = robotPos[ 1 ];

      theta = myPose.theta;
    } else {
      Serial.println("Invalid Pose");
      return 1;
    }

    double d_x = static_cast<double>(closestBall.x); //target x location
    double d_y = static_cast<double>(closestBall.y); //target y location

    //calculate difference between robot and target location
    error_x = d_x - x;
    error_y = d_y - y;

    //calculate distance between robot and target location
    error_d = sqrt(abs(error_x * error_x) + abs(error_y * error_y));
    error_theta = (atan2(error_y, error_x) - (theta / 1000)) * (180 / (PI)) - 90;

    //direction of rotation for the robot to spin
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


    //map omega1 and omega2 to microseconds for the motorshield
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

  //if close enough to the target location, stop the motors
    if (error_d < 130) {
      servo3.writeMicroseconds(1500);
      servo4.writeMicroseconds(1500);
      delay(100);
      close(&clawServo);
      inReach = true;
      //while (1) {}
    }
    prev_error_d = error_d;
    prev_error_theta = error_theta;
  }
  return 0;
}

struct tileNode {

  int x, y, gCost, hCost;
  int wallCertainty;
  bool explored;
  tileNode* parent;

  tileNode() : x(0), y(0), gCost(0), hCost(0), wallCertainty(0), explored(false), parent(nullptr) {}

  tileNode(double dx, double dy) : x(0), y(0), gCost(0), hCost(0), wallCertainty(0), explored(false), parent(nullptr) {
    //Euclidean distance heuristic
    //double Dx = dx - x;
    //double Dy = dy - y;
    //hCost = sqrt(abs(Dx * Dx) + abs(Dy * Dy));

    //Manhattan distance heuristic
    hCost = abs(dx - x) + abs(dy - y);//
  }
  tileNode* getParent() { return this->parent; }
  tileNode(int x, int y, int gCost, int hCost, int wallCertainty, bool explored, tileNode* parent)
    : x(x), y(y), gCost(gCost), hCost(hCost), wallCertainty(wallCertainty), explored(explored), parent(parent) {}

  int fCost() const { return gCost + hCost; };
};




const int gridWidth = 5;
static tileNode path[ gridWidth ][ gridWidth ];
tileNode* unExplored[ gridWidth * gridWidth ];
bool isExplored[ gridWidth ][ gridWidth ];
int unExploredCount = 0;


void printCHar(int c) {
  if (c == 1) {
    Serial.print("#");
  } else if (c == 2) {
    Serial.print("X");
  } else if (c == 3) {
    Serial.print("?");
  } else
    Serial.print("o");

}

void printMap(tileNode* pathEnd) {
  Serial.println("PrintingMap");
  while (pathEnd != nullptr) {
    wallCertaintyMatrix[ pathEnd->x ][ pathEnd->y ] = 2;
    if (pathEnd->parent == nullptr) {
      wallCertaintyMatrix[ pathEnd->x ][ pathEnd->y ] = 3;
    }
    pathEnd = pathEnd->getParent();
  }
  for (int i = 0; i < gridWidth; i++) {
    for (int j = 0; j < gridWidth; j++) {
      printCHar(wallCertaintyMatrix[ i ][ j ]);
    }
    Serial.println();
  }
}

void initializeGrid() {
  Serial.println("InitializingGrid");
  for (int i = 0; i < gridWidth; i++) {
    for (int j = 0; j < gridWidth; j++) {
      path[ i ][ j ] = { x, y, 0, 0, wallCertaintyMatrix[ i ][ j ],false, nullptr };
      isExplored[ i ][ j ] = false;
    }
  }
}

int getHeuristic(tileNode* a, tileNode* b) {
  // Manhattan distance heuristic
  return abs(a->x - b->x) + abs(a->y - b->y);
}

void addToOpenList(tileNode* node) {
  Serial.println("Adding to Open List");
  unExplored[ unExploredCount++ ] = node;
  Serial.println(unExploredCount);
}


tileNode* getLowestFcostNode() {
  if (unExplored[ 0 ] == nullptr) {
    Serial.println("No unexplored nodes");
    return nullptr;
  }
  tileNode* lowestNode = unExplored[ 0 ];
  int lowestFcostIDX = 0;

  for (int i = 0; i < unExploredCount; i++) {
    if (unExplored[ i ]->fCost() < lowestNode->fCost()) {
      lowestFcostIDX = unExplored[ i ]->fCost();
      lowestNode = unExplored[ i ];
    }
  }
  unExplored[ lowestFcostIDX ] = unExplored[ --unExploredCount ];
  return lowestNode;

}

bool isInBounds(int x, int y) {
  return (x >= 0 && x < gridWidth && y >= 0 && y < gridWidth);
}

void reConstructPath(tileNode* node) {
  Serial.println("Reconstructing Path");
  tileNode* currentNode = node;
  while (currentNode != nullptr) {
    // Do something with the path
    currentNode = currentNode->getParent();
  }
}

void AStarPathFinding(int startX, int startY, int targetX, int targetY) {
  unExploredCount = 0;
  initializeGrid();

  static tileNode* startNode = &path[ startX ][ startY ];
  tileNode* targetNode = &path[ targetX ][ targetY ];

  addToOpenList(startNode);



  while (unExploredCount > 0) {
    Serial.println("Traversing");
    tileNode* currentNode = getLowestFcostNode();
    if (currentNode == targetNode) {

      reConstructPath(currentNode);
      printMap(currentNode);
      return;
    }

    const int DX[ 4 ] = { 0, -1, 1, 0 };
    const int DY[ 4 ] = { -1, 0, 0, 1 };


    for (int i = 0; i < 4; i++) {
      int nx = currentNode->x + DX[ i ];
      int ny = currentNode->y + DY[ i ];

      if (!isInBounds(nx, ny)) {
        Serial.println("Skipping");
        continue;
      }
      tileNode* neighbor = &path[ nx ][ ny ];
      if (neighbor->wallCertainty || isExplored[ nx ][ ny ]) continue;

      int tentativeG = currentNode->gCost + 1;
      bool inOpen = false;
      for (int j = 0; j < unExploredCount; j++) {
        if (unExplored[ j ] == neighbor) {
          inOpen = true;
          continue;
        }
      }

      if (!inOpen || tentativeG < neighbor->gCost) {
        neighbor->gCost = tentativeG;
        neighbor->hCost = getHeuristic(neighbor, targetNode);
        neighbor->parent = currentNode;
        if (!inOpen) addToOpenList(neighbor);
      }
    }
  }
  Serial.println("No path found");
}
void setup() {

    //---------General Setup---------
  Serial.begin(115200); //begin serial communication at 115200 baud rate
    /*
  setupServos();      //setup the servos
  setupCommunications();  //setup the communications with the camera

  //---------Drive Servo Setup---------
  servo3.attach(SERVO2_PIN, 1300, 1700);
  servo4.attach(SERVO1_PIN, 1300, 1700);
  clawServo.attach(SERVO3_PIN);


  //---------Limit Switch Setup---------
  //pinMode(IRSENSORENABLE, OUTPUT);
  //pinMode(pinSW, INPUT);


//---------Limit Switch Setup---------
*/
  delay(5);
  Serial.println("Setup Complete");

  delay(1000);
  //initializeGrid();
  AStarPathFinding(0, 0, 4, 4);


}
void loop() {
  //goToLoction();
  //goToHome();
  delay(1000000);
}