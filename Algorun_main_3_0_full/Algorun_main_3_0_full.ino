#include <Wire.h>
#include <VL53L0X.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <vector>
using namespace std;
HardwareSerial MySerial(1);
Preferences prefs;
const int LeftIN1 = 27, LeftIN2 = 26, RightIN1 = 33, RightIN2 = 25;                                                        //motor control pins
const int encLeftA = 34, encLeftB = 39, encRightA = 32, encRightB = 35;                                                    //encorder pins
const int TOF_XSHUT_FRONT = 19, TOF_XSHUT_LEFT = 5, TOF_XSHUT_RIGHT = 4, TOF_XSHUT_45_LEFT = 15, TOF_XSHUT_45_RIGHT = 18;  //vl53L TOF control pins
const int green = 12, red = 2, blue = 14;                                                                                  //LED pins
const int widthofTile = 18, disBetWheels = 75, midPoint = 90;                                                              //robot and maze constants
const int SlowDelay = 500, SlowDelayTurn = 500;                                                                            //delay constant for gradual speeding
const int targetClose_Limit = 180, sideThreshold_Curve = 40;                                                               //mm
const int buttonPin = 13, btn1Value = 4095, btn2Value = 1400, btn3Value = 600, tolerance = 100, readInterval = 10;         //btn constants
volatile long encoderCountLeft = 0, encoderCountRight = 0;                                                                 //encorder steps stored
float Kp = 5, Kd = 0.12, Ki = 0.08;                                                                                        //Encoder difference pid constants
float Kp_curve = 5, Kd_curve = 0.2, Ki_curve = 0.08;
float KpDF = 3, KdDF = 0.7;    // Front distance sensor target pid
float KpDEn = 2, KdDEn = 0.1;  //Target Encorder pid
float KdDEn_curve = 0.05;
int diaglSideSensor_Thr = 45, NondiagsideSensThr = 100;
float diagFSideSensorX = 0.25, diagSideSensorX = 2;
float NondiagsideSensX = 0.6;
float curveTurnSideThr = 60, curveTurnFSideThr = 200;
float curveturnX = 0.00305;
float factor = 0.8;
int frontDisTh_curve = 140;
bool run = 0, state1 = 0, state2 = 0, state3 = 0, prevPressed = false;  //initial robot state,debug state,btn states
bool tof_sensors_ready = false, gotileTicket = false, FwallTriggered = 0, send = 0;
unsigned long targetStep, lastReadTime = 0, st, lastMicrosPID, Count = 0;
int goCount = 0, curveTurncount = 0, turnCount = 0;  //delay for stopping,...
long lastEn = 0, timegap = 1000, lastmicros;         //us
bool hc12print = 0, serial = 0;                      //debug report
double velocity = 0;
int mode = -1;
int delay1 = 0;
int ANG = 70;                                                       //default mode
VL53L0X tof_front, tof_left, tof_right, tof_45_LEFT, tof_45_RIGHT;  //tof sensor object
int dis_Reading[5], buffsp[6] = { 140, 150, 120, 140, 160, 130 };
String print = "", Data = "";  //store data for debugging
SemaphoreHandle_t dataMutex;   //used to read/write varibles without interfering other core
void IRAM_ATTR isrEncLeft();
void IRAM_ATTR isrEncRight();
void setup_tof_sensors();
bool corner = 0;  //0=left ,1=right
bool run3 = 0, run4 = 0;
float theta = 0.0;                           // radians
long lastCountLeft = 0, lastCountRight = 0;  // Last encoder values
struct slowStopData {
  bool startofStopBool = false;
  unsigned long startofStop = 0;
  int waitDelay;
  unsigned long run_stime;
};
struct Cell {
  bool N, S, E, W;
  bool visited;
};
struct Move {
  char type;  // 'F' = forward, 'L' = left, 'R' = right
  int count;  // number of consecutive forwards
};
struct Diag {
  int start;
  int stop;
  char dir;  //1 for cw 0 for ccw
};
const int MAZE_SIZE = 8, WALL_THRESHOLD = 100;
Cell maze[MAZE_SIZE][MAZE_SIZE];
int current_x = 0, current_y = 0;
char current_direction = 'N';
bool exploration_complete = false, at_goal_flag = false;
struct Position {
  int x, y;
};
//Position goals[] = { { 7, 7 }, { 7, 8 }, { 8, 7 }, { 8, 8 } }, start_pos = { 0, 0 };//for 16x16 maze
Position goals[] = { { 3, 3 }, { 4, 3 }, { 3, 4 }, { 4, 4 } }, start_pos = { 0, 0 };  //for 8x8 maze
const int NUM_GOALS = 4;
int flood_distances[MAZE_SIZE][MAZE_SIZE], queue_x[MAZE_SIZE * MAZE_SIZE], queue_y[MAZE_SIZE * MAZE_SIZE];
int queue_front = 0, queue_rear = 0;
void setup() {
  Serial.begin(115200);
  MySerial.begin(115200, SERIAL_8N1, 16, 17);  //for HC12 | used for send debug data wireless
  setupPins();
  setup_tof_sensors();
  xTaskCreatePinnedToCore(core0, "core0", 4096, NULL, 1, NULL, 0);  // Run on core 0
  dataMutex = xSemaphoreCreateMutex();
  attachInterrupt(digitalPinToInterrupt(encLeftA), isrEncLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encRightA), isrEncRight, CHANGE);
  setupCompleted();  //LED indication if setup completed
  // prefs.begin("micromouse", false);  // RW mode
  // prefs.clear();
  // prefs.end();
}
void loop() {
  checkButtons();
  if (run) {
    delay(500);
    MySerial.print("Starting");
    runFloodfillMode(1);
    run = 0;
    MySerial.print("Ending");
  } else if (run3) {
    delay(500);
    MySerial.print("Starting");
    runFloodfillMode(3);
    run3 = 0;
    MySerial.print("Ending");
  } else if (run4) {
    delay(500);
    MySerial.print("Starting");
    runFloodfillMode(4);
    run4 = 0;
    MySerial.print("Ending");
  }
  if (MySerial.available()) {
    String incoming = MySerial.readStringUntil('\n');  // read line
    Serial.println(incoming);
    processHC12Data(incoming);
  }
}
void saveMazeAndFloodfill() {
  prefs.begin("micromouse", false);  // RW mode
  prefs.putBytes("maze", maze, sizeof(maze));
  prefs.putBytes("flood", flood_distances, sizeof(flood_distances));
  prefs.end();
}
bool loadMazeAndFloodfill() {
  prefs.begin("micromouse", true);  // RO mode
  size_t mazeSize = prefs.getBytesLength("maze");
  size_t floodSize = prefs.getBytesLength("flood");
  bool loaded = false;
  if (mazeSize == sizeof(maze) && floodSize == sizeof(flood_distances)) {
    prefs.getBytes("maze", maze, sizeof(maze));
    prefs.getBytes("flood", flood_distances, sizeof(flood_distances));
    loaded = true;
  }
  prefs.end();
  return loaded;
}
void generateOptimalPath(Move* path2, int& pathLength2) {
  Move path[256];  //temp parth, not final one
  pathLength2 = 0;
  int pathLength = 0;
  int x = current_x, y = current_y;
  char dir = current_direction;
  auto isAtGoalSim = [](int px, int py) -> bool {
    for (int i = 0; i < NUM_GOALS; i++) {
      if (px == goals[i].x && py == goals[i].y) {
        return true;
      }
    }
    return false;
  };
  while (!isAtGoalSim(x, y)) {
    int temp_x = current_x, temp_y = current_y;
    current_x = x;
    current_y = y;
    char best_dir = getBestDirection();
    current_x = temp_x;
    current_y = temp_y;
    int forwardCount = 0;
    while (best_dir == dir && !isAtGoalSim(x, y)) {
      forwardCount++;
      int dx, dy;
      getDirectionOffset(dir, &dx, &dy);
      x += dx;
      y += dy;
      if (!isAtGoalSim(x, y)) {
        current_x = x;
        current_y = y;
        best_dir = getBestDirection();
        current_x = temp_x;
        current_y = temp_y;
      }
    }
    if (forwardCount > 0) {
      path[pathLength].type = 'F';
      path[pathLength].count = forwardCount;
      pathLength++;
    }
    if (!isAtGoalSim(x, y)) {
      int turn_angle = directionToAngle(dir, best_dir);
      if (turn_angle != 0) {
        path[pathLength].type = (turn_angle > 0) ? 'R' : 'L';
        path[pathLength].count = 1;
        pathLength++;
        dir = best_dir;
      }
    }
  }
  vector<Diag> diag;
  // diag.push_back({ 0, 1 });
  int k = 0;
  if (path[0].type != 'F') {
    int angle = (path[k].type == 'R') ? 90 : -90;  //means inital one is turn, which will always follows by forward
    path2[k].type = path[k].type;
    path2[k].count = angle;  //turnangle
    k++;
    path2[k].type = path[k].type;
    path2[k].count = path[k].count;  //forward
    k++;
  } else {  //if current dir is equal to best dir
    path2[k].type = path[k].type;
    path2[k].count = path[k].count;  //forward
    k++;
  }
  for (int i = k; i < pathLength; i++) {  //for diagonal find
    char D;
    int temp = -1;
    if (path[i].type == 'R') D = 'R';
    else if (path[i].type == 'L') D = 'L';
    for (int j = i + 2; j < pathLength; j += 2) {
      if (D = 'R') {
        if (path[j].type == 'L') D = 'L';
        else {
          temp = j - 2;
          break;
        }
      } else if (D = 'L') {
        if (path[j].type == 'R') D = 'R';
        else {
          temp = j - 2;
          break;
        }
      }
    }
    if (temp != i) {
      if (D == 'L') diag.push_back({ i, temp, 'R' });
      else if (D == 'R') diag.push_back({ i, temp, 'L' });
    }
  }
  int dd = 0;
  for (int i = k; i < pathLength; i++) {
    if (i == diag[dd].start) {
      path2[k].type = 'D';
      int p;
      if (diag[dd].dir == 'L') p = -1;
      else if (diag[dd].dir == 'R') p = 1;
      int l = (diag[dd].stop - diag[dd].start + 1);
      path2[k].count = p * l;
      i = i + 2 * (l - 1);
      dd++;
    } else if (path[i].type == 'F' && path[i].count > 1) {
      path2[k].type = path[i].type;
      path2[k].count = path[i].count;
      k++;
      MySerial.println("GO");
      delay(10);
    } else {
      int angle;
      String kk;
      if (path[i].type == 'R') {
        angle = 90;
        if (path[i + 2].type == 'R' && path[i + 1].count == 1) {
          i += 2;
          angle = 180;
        }
        kk = "CTurn R" + String(angle);
        MySerial.println(kk);
        path2[k].type = 'C';
        path2[k].count = angle;
        k++;
        delay(10);
      } else if (path[i].type == 'L') {
        angle = -90;
        if (path[i + 2].type == 'L' && path[i + 1].count == 1) {
          angle = -180;
          i += 2;
        }
        kk = "CTurn L" + String(angle);
        MySerial.println(kk);
        path2[k].type = 'C';
        path2[k].count = angle;
        k++;
        delay(10);
      }
    }
  }
  String print2 = "";
  for (int i = 0; i < pathLength; i++) {
    print2 += String(i) + ":" + String(path[i].type) + " " + String(path[i].count) + "\n";
  }
  Serial.println(print2);
  MySerial.println(print2);
  pathLength2 = k;
}
void executeSpeedRun4() {
  Move path[256];
  int pathLength = 0;
  generateOptimalPath(path, pathLength);
  MySerial.println("hello");
  String print2 = "";
  for (int i = 0; i < pathLength; i++) {
    print2 += String(i) + ":" + String(path[i].type) + " " + String(path[i].count) + "\n";
  }
  int ddelay = 0;
  Serial.println(print2);
  MySerial.println(print2);
  for (int i = 0; i < pathLength; i++) {
    if (path[i].type == 'F' && path[i].count > 1) {
      GoTiles(path[i].count - 1.1, 160, 240, 10, 0, ddelay);  //go tiles
      MySerial.println("GO");
      delay(10);
    } else if (path[i].type == 'L' || path[i].type == 'R') {  //turn angle
      turnAngle(path[i].count, ddelay);
      GoTiles(1, 140, 240, 0, 0, ddelay);
      MySerial.println("Turn" + String(path[i].type));
      delay(10);
    } else if (path[i].type == 'C') {  //turn angle
      curveTurn(path[i].count - 17, 100);
      MySerial.println("CTurn:" + String(path[i].count));
      delay(10);
    } else if (path[i].type == 'D') {  //diagonal
      turnAngle(path[i].count / abs(path[i].count) * 45, ddelay);
      GoTiles(diagonaltiles_Calc(path[i].count), 30, 50, 0, 0, ddelay);
      turnAngle(path[i].count / abs(path[i].count) * 45, ddelay);
      MySerial.println("Diagonal");
      delay(10);
    }
  }
}
bool inTheMiddle = false;
int sideWallsArray[2] = { -1, -1 };
void GoTiles(float targetTiles, int wallStopThreshold, int wallNearThreshold, int mmExtra, bool diagonal, int waitDelay) {
  long targetStep = DistanceToStep(targetTiles * widthofTile * 10 + mmExtra), targetClose_Step = DistanceToStep(targetClose_Limit);
  long buffAvgEn = 0;
  gotileTicket = 1, lastmicros = micros(), lastEn = 0;
  float prevTargetEnc_Error = 0, prevFrontWall__Error = 0, prevErrorDiff = 0;
  float baseSpeed, centerAdj = 0, intg = 0;
  int FrontWall_Error = 0, TargetEnc_Error = DistanceToStep(360), dis_L, dis_R, dis_F = -1, dis_45L, dis_45R;
  unsigned long run_stime = millis();
  slowStopData S;
  inTheMiddle = 0, sideWallsArray[0] = -1, sideWallsArray[1] = -1;
  S.waitDelay = waitDelay, S.run_stime = run_stime;
  goCount++;
  long intialEncoderCountLeft = encoderCountLeft, intialEncoderCountRight = encoderCountRight;
  while (1) {
    checkButtons();
    print = "", Count++;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    dis_F = dis_Reading[0], dis_L = dis_Reading[1], dis_R = dis_Reading[2], dis_45L = dis_Reading[3], dis_45R = dis_Reading[4];
    xSemaphoreGive(dataMutex);
    long enL = encoderCountLeft - intialEncoderCountLeft, enR = encoderCountRight - intialEncoderCountRight;
    float avg_encoder_count = (enL + enR) / 2.00;
    FrontWall_Error = 1000;
    unsigned long now = micros();
    //print += String(now - lastMicrosPID) + " ";
    float dt = (now - lastMicrosPID) / 1e6;                                            // convert microseconds → seconds
    if (dt <= 0) dt = 0.00001;                                                         // avoid divide by zero
    if (dis_F < wallNearThreshold && !FwallTriggered && millis() - run_stime > 200) {  //if near front wall
      FrontWall_Error = dis_F - wallStopThreshold;
      targetStep = DistanceToStep(FrontWall_Error);
      buffAvgEn = avg_encoder_count;
      avg_encoder_count = 0, prevTargetEnc_Error = 0, intialEncoderCountLeft = encoderCountLeft, intialEncoderCountRight = encoderCountRight;
      FwallTriggered = 1;
    }
    if (avg_encoder_count + buffAvgEn >= float(buffAvgEn + targetStep) * 95 / 100.00 && !inTheMiddle && !diagonal) {  //in the middle, detect walls now
      sideWallsArray[0] = dis_L;
      sideWallsArray[1] = dis_R;
      inTheMiddle = 1;
      print = String(sideWallsArray[0]) + " " + String(sideWallsArray[1]);
      //MySerial.println(print);
    }
    TargetEnc_Error = targetStep - avg_encoder_count;
    float TargetEnc_Error_Difference = (TargetEnc_Error - prevTargetEnc_Error) / dt;
    prevTargetEnc_Error = TargetEnc_Error;
    baseSpeed = KpDEn * TargetEnc_Error + TargetEnc_Error_Difference * KdDEn;

    int speedLeft = baseSpeed, speedRight = baseSpeed;
    centerAdj = 0;
    {  // distance senor adjustment
      if (!diagonal) {
        if (dis_L < NondiagsideSensThr && dis_R < NondiagsideSensThr) {  //both side wall
          centerAdj = NondiagsideSensX * (dis_L - dis_R);                //+left more clear
        } else if (dis_L < NondiagsideSensThr) {                         //only left wall
          dis_R = 180 - 75 - dis_L;
          centerAdj = NondiagsideSensX * (dis_L - dis_R);
        } else if (dis_R < NondiagsideSensThr) {  //only right wall
          dis_L = 180 - 75 - dis_R;
          centerAdj = NondiagsideSensX * (dis_L - dis_R);
        }
      } else if (diagonal) {
        centerAdj = diagFSideSensorX * (dis_45L - dis_45R);
        print += " 45adj:" + String(centerAdj);
        if (dis_L < diaglSideSensor_Thr) {  //only left wall
          centerAdj += diagSideSensorX * (dis_L - diaglSideSensor_Thr);
          print += " sideAdj:" + String(diagSideSensorX * (dis_L - diaglSideSensor_Thr));
        } else if (dis_R < diaglSideSensor_Thr) {  //only right wall
          centerAdj += diagSideSensorX * (diaglSideSensor_Thr - dis_R);
          print += " sideAdj:" + String(diagSideSensorX * (diaglSideSensor_Thr - dis_R));
        }
      }
    }
    float dif_error = enL - enR + centerAdj + Ki * intg;  //+ left faster -right faster
    float difError_Diff = (dif_error - prevErrorDiff) / dt;
    prevErrorDiff = dif_error;
    intg += centerAdj;

    int diffCount = targetStep - avg_encoder_count;     //error based on target distance and current
    float error = Kp * dif_error + Kd * difError_Diff;  //+ccw -cw
    SpeedAdj_Constrain(speedLeft, speedRight, error, run_stime, diffCount, targetClose_Step);
    //    ---- Compact Debug Print ----
    {
      print = String(Count) + " ";
      //print += " " + String(dt, 6);
      print += " EnL:" + String(enL) + " EnR:" + String(enR);
      print += (error > 0) ? " biasing left" : (error < 0) ? " biasing right"
                                                           : " no error";
      //print += " Cen:" + String(centerAdj);
      // print += " diffEn:" + String(dif_error) + " FwallE:" + String(FrontWall_Error) + " TargetEnEr:" + String(TargetEnc_Error);
      //print += " diffEnD:" + String(difError_Diff);
      //print += " diffCount:" + String(diffCount);
      print += " Error:" + String(error);
      if (FwallTriggered) print += " FWall";
      print += " DF:" + String(dis_F) + " DL:" + String(dis_L) + " DR:" + String(dis_R);
      print += " D45L:" + String(dis_45L) + " D45R:" + String(dis_45R);
      print += " Go:" + String(goCount);
      if (serial) Serial.println(print);
      else delay(10);
    }
    if (hc12print) hc12Send(print);
    if (handleStop(S, TargetEnc_Error, FwallTriggered)) break;
    lastMicrosPID = now;
  }
}
void GoTilesDefault() {  //two go one cell with default parameters, used for exploring maze not for fast runs
  int mmExtra = 0;       //extra mm to be added to the distance, 1cell distance + mmExtra will be final distance to be traveled
  GoTiles(1, 75, 160, mmExtra, 0, 200);
}
void curveTurn(int angle, int waitDelay) {
  lastmicros = micros(), lastEn = 0;
  float prevTargetEnc_Error = 0, prevFrontWall__Error = 0, prevErrorDiff = 0, baseSpeed, centerAdj = 0, intg = 0;
  int dis_L, dis_R, dis_F, dis_45L, dis_45R, rightstp, leftstp;
  unsigned long run_stime = millis();
  int a = angle / abs(angle);
  int b = a * (-1);
  float co1, co2;
  slowStopData S;
  S.waitDelay = waitDelay, S.run_stime = millis();
  curveTurncount++;
  long intialEncoderCountLeft = encoderCountLeft, intialEncoderCountRight = encoderCountRight;
  if (a > 0) {                                                                       //for + angle
    rightstp = DistanceToStep(arcL_cal(midPoint - (float)disBetWheels / 2, angle));  //for cc right would have low steps
    leftstp = DistanceToStep(arcL_cal(midPoint + (float)disBetWheels / 2, angle));   //for cc left would have high steps
    co1 = (float)rightstp / (float)leftstp, co2 = 1;
    co1 = co1 * factor;
  } else if (a < 0) {                                                                //for - angle
    leftstp = DistanceToStep(arcL_cal(midPoint - (float)disBetWheels / 2, angle));   //for ccw left would have low steps
    rightstp = DistanceToStep(arcL_cal(midPoint + (float)disBetWheels / 2, angle));  //for ccw right would have high steps
    co2 = (float)leftstp / (float)rightstp, co1 = 1;
    co2 = co2 * factor;
  }
  long targetStep = abs(rightstp + leftstp) / 2, targetClose_Step = 0;
  while (1) {
    checkButtons();
    print = "", Count++;
    print += String(Count) + " ";
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    dis_F = dis_Reading[0], dis_L = dis_Reading[1], dis_R = dis_Reading[2], dis_45L = dis_Reading[3], dis_45R = dis_Reading[4];
    xSemaphoreGive(dataMutex);
    long enL = encoderCountLeft - intialEncoderCountLeft, enR = encoderCountRight - intialEncoderCountRight;
    float avg_encoder_count = (enL + enR) / 2.00;
    unsigned long now = micros();
    float dt = (now - lastMicrosPID) / 1e6;  // convert microseconds → seconds
    float TargetEnc_Error = targetStep - abs(avg_encoder_count);
    float TargetEnc_Error_Difference = (TargetEnc_Error - prevTargetEnc_Error) / dt;
    prevTargetEnc_Error = TargetEnc_Error;
    baseSpeed = KpDEn * TargetEnc_Error + TargetEnc_Error_Difference * KdDEn_curve;

    int speedLeft = baseSpeed, speedRight = baseSpeed;
    float centerAdj = 0;
    float adj = 0, coN1 = 0, coN2 = 0;
    {  // distance sensor adjustment
      if (dis_L < curveTurnSideThr) {
        adj = curveTurnSideThr - dis_L;
      } else if (dis_R < curveTurnSideThr) {
        adj = -1 * (curveTurnSideThr - dis_R);
      } else if (dis_L < curveTurnSideThr && dis_R < curveTurnSideThr) {
        adj = dis_R - dis_L;
      } else if (dis_45L < curveTurnSideThr) {
        adj = curveTurnSideThr - dis_45L;
      } else if (dis_45R < curveTurnSideThr) {
        adj = -1 * (curveTurnSideThr - dis_45R);
      } else if (dis_45L < curveTurnSideThr && dis_45R < curveTurnSideThr) {
        adj = dis_45R - dis_45L;
      }

      adj = adj * curveturnX;
      print += " adj:" + String(adj);
      //adj = constrain(adj, -0.8, 0.8);
      if (co1 == 1) {
        coN2 = co2 + adj;
        coN1 = 1;
      } else if (co2 == 1) {
        coN1 = co1 - adj;
        coN2 = 1;
      }
    }
    float dif_error = (float)enL * coN1 - (float)enR * coN2 + Ki_curve * intg;  //+ left faster -right faster
    float difError_Diff = (dif_error - prevErrorDiff) / dt;
    prevErrorDiff = dif_error;
    intg += centerAdj;
    int diffCount = targetStep - avg_encoder_count;                 //error based on target distance and current
    float error = Kp_curve * dif_error + Kd_curve * difError_Diff;  //+ccw -cw
    SpeedAdj_Constrain(speedLeft, speedRight, error, run_stime, diffCount, targetClose_Step);
    //    ---- Compact Debug Print ----
    {
      //print += String(dt, 6);
      // print += " base:" + String(baseSpeed);
      print += " EnL:" + String(enL) + " EnR:" + String(enR);
      //print += " base:" + String(baseSpeed) + " LimBase:" + String(LimBaseSpeed) + " MaxSp:" + String(maxspeed);
      print += (error > 0) ? " biasing left" : (error < 0) ? " biasing right"
                                                           : " no error";
      print += " coN1:" + String(coN1, 4) + " coN2:" + String(coN2, 4);
      //print += " diffEn:" + String(dif_error) + " TargetEnEr:" + String(TargetEnc_Error);
      //print += " diffEnD:" + String(difError_Diff);
      print += " Error:" + String(error);
      print += " DF:" + String(dis_F) + " DL:" + String(dis_L) + " DR:" + String(dis_R);
      print += " D45L:" + String(dis_45L) + " D45R:" + String(dis_45R);
      print += " curveTurn:" + String(curveTurncount);
      if (serial) Serial.println(print);
      else delay(10);
    }
    if (hc12print) hc12Send(print);
    if (handleStop(S, TargetEnc_Error, FwallTriggered)) break;
    lastMicrosPID = now;
  }
}
void turnAngle(int angle, int waitDelay) {
  if (abs(angle) >= 90 && abs(angle) < 180) angle = angle - (angle / abs(angle)) * 13;
  long targetStep = DistanceToStep(abs(angle) / float(180) / 2 * PI * disBetWheels);
  float dis_L, dis_R, dis_F, dis_45L, dis_45R;
  unsigned long run_stime = millis();
  turnCount++;
  double intrErrCen = 0;
  slowStopData S;
  S.waitDelay = waitDelay, S.run_stime = millis();
  long intialEncoderCountLeft = encoderCountLeft, intialEncoderCountRight = encoderCountRight;
  while (1) {
    checkButtons();
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    dis_F = dis_Reading[0], dis_L = dis_Reading[1], dis_R = dis_Reading[2], dis_45L = dis_Reading[3], dis_45R = dis_Reading[4];
    xSemaphoreGive(dataMutex);
    long enL = encoderCountLeft - intialEncoderCountLeft, enR = encoderCountRight - intialEncoderCountRight;
    float avg_encoder_count = abs(enL - enR) / 2.00;
    int a = angle / abs(angle);
    int b = a * (-1);
    int dif_error = a * (enL + enR);
    float TargetEnc_Error = targetStep - avg_encoder_count;
    print = "", Count++;
    print += String(Count) + " ";
    int diffCount = targetStep - avg_encoder_count;
    int LimBaseSpeed = buffsp[3], maxspeed = buffsp[4], min_effective_speed = buffsp[5];  //apply default turn speeds
    if (millis() - run_stime <= SlowDelay) {                                              //starting slow
      maxspeed = LimBaseSpeed + (millis() - run_stime) / float(SlowDelay) * (maxspeed - LimBaseSpeed);
      LimBaseSpeed = min_effective_speed + (millis() - run_stime) / float(SlowDelay) * (LimBaseSpeed - min_effective_speed);
    }
    int speedLeft = a * KpDF * TargetEnc_Error, speedRight = b * KpDF * TargetEnc_Error;
    {  //constrain speed, minimum effective spee
      speedLeft = constrain(speedLeft - a * Kp * dif_error, -maxspeed, maxspeed);
      speedRight = constrain(speedRight + b * Kp * dif_error, -maxspeed, maxspeed);
      speedLeft = applyMinSpeed(speedLeft, min_effective_speed);
      speedRight = applyMinSpeed(speedRight, min_effective_speed);
      setMotorSpeed(speedLeft, speedRight);
    }
    // ---- Compact Debug Print ----
    {
      print += "EnL:" + String(enL) + " EnR:" + String(enR);
      print += " SpL:" + String(speedLeft);
      //print += " SpR:" + String(speedRight) + " TargetDisErr:" + String(KpDF * TargetEnc_Error) + " EnDiffErr:" + String(Kp * dif_error);
      //print += " LimBase:" + String(LimBaseSpeed) + " MaxSp:" + String(maxspeed);
      print += " DF:" + String(dis_F) + " DL:" + String(dis_L) + " DR:" + String(dis_R);
      print += " D45L:" + String(dis_45L) + " D45R:" + String(dis_45R);
      print += " Turn:" + String(turnCount);
      if (serial) Serial.println(print);
      else delay(10);
    }
    if (handleStop(S, TargetEnc_Error, FwallTriggered)) break;
    if (hc12print) hc12Send(print);
  }
}
void initializeMaze() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      maze[y][x].N = false;
      maze[y][x].S = false;
      maze[y][x].E = false;
      maze[y][x].W = false;
      maze[y][x].visited = false;
    }
  }

  for (int i = 0; i < MAZE_SIZE; i++) {
    maze[0][i].S = true;
    maze[MAZE_SIZE - 1][i].N = true;
    maze[i][0].W = true;
    maze[i][MAZE_SIZE - 1].E = true;
  }
}
void enqueue(int x, int y) {
  queue_x[queue_rear] = x;
  queue_y[queue_rear] = y;
  queue_rear = (queue_rear + 1) % (MAZE_SIZE * MAZE_SIZE);
}
bool dequeue(int* x, int* y) {
  if (queue_front == queue_rear) return false;
  *x = queue_x[queue_front];
  *y = queue_y[queue_front];
  queue_front = (queue_front + 1) % (MAZE_SIZE * MAZE_SIZE);
  return true;
}
bool isQueueEmpty() {
  return queue_front == queue_rear;
}
void clearQueue() {
  queue_front = queue_rear = 0;
}
bool atGoal() {
  for (int i = 0; i < NUM_GOALS; i++) {
    if (current_x == goals[i].x && current_y == goals[i].y) {
      return true;
    }
  }
  return false;
}
bool atStart() {
  return (current_x == start_pos.x && current_y == start_pos.y);
}
void getDirectionOffset(char direction, int* dx, int* dy) {
  switch (direction) {
    case 'N':
      *dx = 0;
      *dy = 1;
      break;
    case 'E':
      *dx = 1;
      *dy = 0;
      break;
    case 'S':
      *dx = 0;
      *dy = -1;
      break;
    case 'W':
      *dx = -1;
      *dy = 0;
      break;
  }
}
char getOppositeDirection(char direction) {
  switch (direction) {
    case 'N': return 'S';
    case 'E': return 'W';
    case 'S': return 'N';
    case 'W': return 'E';
    default: return 'N';
  }
}
char getLeftDirection(char dir) {
  switch (dir) {
    case 'N': return 'W';
    case 'W': return 'S';
    case 'S': return 'E';
    case 'E': return 'N';
    default: return 'N';
  }
}
char getRightDirection(char dir) {
  switch (dir) {
    case 'N': return 'E';
    case 'E': return 'S';
    case 'S': return 'W';
    case 'W': return 'N';
    default: return 'N';
  }
}
int directionToAngle(char from, char to) {
  char directions[] = { 'N', 'E', 'S', 'W' };
  int from_idx = -1, to_idx = -1;

  for (int i = 0; i < 4; i++) {
    if (directions[i] == from) from_idx = i;
    if (directions[i] == to) to_idx = i;
  }

  int diff = (to_idx - from_idx + 4) % 4;
  switch (diff) {
    case 0: return 0;
    case 1: return 90;
    case 2: return 180;
    case 3: return -90;
    default: return 0;
  }
}
void updatePosition() {
  int dx, dy;
  getDirectionOffset(current_direction, &dx, &dy);
  current_x += dx;
  current_y += dy;

  current_x = constrain(current_x, 0, MAZE_SIZE - 1);
  current_y = constrain(current_y, 0, MAZE_SIZE - 1);
}
void returnToStart() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      flood_distances[y][x] = 999;
    }
  }
  clearQueue();
  flood_distances[start_pos.y][start_pos.x] = 0;
  enqueue(start_pos.x, start_pos.y);

  int x, y;
  while (dequeue(&x, &y)) {
    int current_dist = flood_distances[y][x];

    char directions[] = { 'N', 'E', 'S', 'W' };
    for (int i = 0; i < 4; i++) {
      char dir = directions[i];
      int dx, dy;
      getDirectionOffset(dir, &dx, &dy);
      int nx = x + dx;
      int ny = y + dy;

      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        bool hasWall = false;
        switch (dir) {
          case 'N': hasWall = maze[y][x].N; break;
          case 'E': hasWall = maze[y][x].E; break;
          case 'S': hasWall = maze[y][x].S; break;
          case 'W': hasWall = maze[y][x].W; break;
        }

        if (!hasWall && flood_distances[ny][nx] > current_dist + 1) {
          flood_distances[ny][nx] = current_dist + 1;
          enqueue(nx, ny);
        }
      }
    }
  }

  while (!atStart()) {
    char best_dir = getBestDirection();
    if (best_dir != current_direction) {
      int turn_angle = directionToAngle(current_direction, best_dir);
      if (turn_angle != 0) {
        turnAngle(turn_angle, delay1);
        current_direction = best_dir;
      }
    }
    GoTilesDefault();
    updatePosition();
  }
}
/////////////////////////////////////////////////////////////////////////////////
void detectWalls(bool* wallN, bool* wallE, bool* wallS, bool* wallW) {
  int dis_F, dis_L, dis_R;
  // Get current sensor readings
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  dis_F = dis_Reading[0];  // Front sensor
  if (sideWallsArray[0] == -1 && sideWallsArray[1] == -1) {
    dis_L = dis_Reading[1];  // Left sensor
    dis_R = dis_Reading[2];  // Right sensor
  } else {
    dis_L = sideWallsArray[0];  // Left sensor
    dis_R = sideWallsArray[1];  // Right sensor
  }
  xSemaphoreGive(dataMutex);
  // Initialize all walls as false
  *wallN = *wallE = *wallS = *wallW = false;  //no walls

  // Detect walls based on current direction
  switch (current_direction) {
    case 'N':
      *wallN = (dis_F < WALL_THRESHOLD);
      *wallW = (dis_L < WALL_THRESHOLD);
      *wallE = (dis_R < WALL_THRESHOLD);
      break;
    case 'E':
      *wallE = (dis_F < WALL_THRESHOLD);
      *wallN = (dis_L < WALL_THRESHOLD);
      *wallS = (dis_R < WALL_THRESHOLD);
      break;
    case 'S':
      *wallS = (dis_F < WALL_THRESHOLD);
      *wallE = (dis_L < WALL_THRESHOLD);
      *wallW = (dis_R < WALL_THRESHOLD);
      break;
    case 'W':
      *wallW = (dis_F < WALL_THRESHOLD);
      *wallS = (dis_L < WALL_THRESHOLD);
      *wallN = (dis_R < WALL_THRESHOLD);
      break;
  }
}
void updateMazeWalls() {
  bool wallN, wallE, wallS, wallW;
  detectWalls(&wallN, &wallE, &wallS, &wallW);
  int x = current_x;
  int y = current_y;
  if (wallN) {
    maze[y][x].N = true;
    if (y + 1 < MAZE_SIZE) maze[y + 1][x].S = true;
  }
  if (wallE) {
    maze[y][x].E = true;
    if (x + 1 < MAZE_SIZE) maze[y][x + 1].W = true;
  }
  if (wallS) {
    maze[y][x].S = true;
    if (y - 1 >= 0) maze[y - 1][x].N = true;
  }
  if (wallW) {
    maze[y][x].W = true;
    if (x - 1 >= 0) maze[y][x - 1].E = true;
  }
  maze[y][x].visited = true;
}
void floodFill() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      flood_distances[y][x] = 999;
    }
  }
  clearQueue();
  for (int i = 0; i < NUM_GOALS; i++) {
    int gx = goals[i].x;
    int gy = goals[i].y;
    flood_distances[gy][gx] = 0;
    enqueue(gx, gy);
  }

  int x, y;
  while (dequeue(&x, &y)) {
    int current_dist = flood_distances[y][x];
    char directions[] = { 'N', 'E', 'S', 'W' };
    for (int i = 0; i < 4; i++) {
      char dir = directions[i];
      int dx, dy;
      getDirectionOffset(dir, &dx, &dy);
      int nx = x + dx;
      int ny = y + dy;
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        bool hasWall = false;
        switch (dir) {
          case 'N': hasWall = maze[y][x].N; break;
          case 'E': hasWall = maze[y][x].E; break;
          case 'S': hasWall = maze[y][x].S; break;
          case 'W': hasWall = maze[y][x].W; break;
        }

        if (!hasWall && flood_distances[ny][nx] > current_dist + 1) {
          flood_distances[ny][nx] = current_dist + 1;
          enqueue(nx, ny);
        }
      }
    }
  }
}
char getBestDirection() {
  int x = current_x;
  int y = current_y;
  char best_direction = 'N';
  int best_distance = 999;

  char directions[] = { 'N', 'E', 'S', 'W' };
  for (int i = 0; i < 4; i++) {
    char dir = directions[i];
    int dx, dy;
    getDirectionOffset(dir, &dx, &dy);
    int nx = x + dx;
    int ny = y + dy;
    // Data += "dir:" + String(dir);
    // Data += " nx:" + String(nx);
    // Data += " ny:" + String(ny) + "\n";
    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      bool hasWall = false;
      switch (dir) {
        case 'N': hasWall = maze[y][x].N; break;
        case 'E': hasWall = maze[y][x].E; break;
        case 'S': hasWall = maze[y][x].S; break;
        case 'W': hasWall = maze[y][x].W; break;
      }
      if (!hasWall && flood_distances[ny][nx] < best_distance) {
        best_distance = flood_distances[ny][nx];
        best_direction = dir;
      }
    }
  }
  return best_direction;
}
void floodfillSearch() {
  long cc = 0;
  while (!atGoal()) {
    cc++;
    updateMazeWalls();
    floodFill();
    char best_dir = getBestDirection();
    if (best_dir != current_direction) {
      int turn_angle = directionToAngle(current_direction, best_dir);
      if (turn_angle != 0) {
        turnAngle(turn_angle, delay1);
        current_direction = best_dir;
      }
    }
    int x = current_x, y = current_y;
    Data = String(cc);
    Data += " x:" + String(x);
    Data += " y:" + String(y) + "\n";
    Data += "Current dir:" + String(current_direction) + "\n";
    Data += "N:" + String(maze[y][x].N);
    Data += " E:" + String(maze[y][x].E);
    Data += " S:" + String(maze[y][x].S);
    Data += " W:" + String(maze[y][x].W);
    int valueN = (y + 1 < MAZE_SIZE && !maze[y][x].N) ? flood_distances[y + 1][x] : 999;
    int valueE = (x + 1 < MAZE_SIZE && !maze[y][x].E) ? flood_distances[y][x + 1] : 999;
    int valueS = (y - 1 >= 0 && !maze[y][x].S) ? flood_distances[y - 1][x] : 999;
    int valueW = (x - 1 >= 0 && !maze[y][x].W) ? flood_distances[y][x - 1] : 999;
    GoTilesDefault();
    Data += "\nFloodFill values:";
    Data += " N_fl:" + String(valueN);
    Data += " E_fl:" + String(valueE);
    Data += " S_fl:" + String(valueS);
    Data += " W_fl:" + String(valueW) + "\n";
    Data += "Traveled one tile on:" + String(current_direction);
    // Serial.println(Data);
    // MySerial.println(Data);
    updatePosition();
  }
  at_goal_flag = true;
}
char getUnvisitedNeighborDirection() {
  int x = current_x;
  int y = current_y;
  char directions[] = { 'N', 'E', 'S', 'W' };

  for (int i = 0; i < 4; i++) {
    char dir = directions[i];
    int dx, dy;
    getDirectionOffset(dir, &dx, &dy);
    int nx = x + dx;
    int ny = y + dy;

    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      bool hasWall = false;
      switch (dir) {
        case 'N': hasWall = maze[y][x].N; break;
        case 'E': hasWall = maze[y][x].E; break;
        case 'S': hasWall = maze[y][x].S; break;
        case 'W': hasWall = maze[y][x].W; break;
      }

      if (!hasWall && !maze[ny][nx].visited) {
        return dir;
      }
    }
  }

  return 'X';
}
void floodFillToStart() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      flood_distances[y][x] = 999;
    }
  }
  clearQueue();
  flood_distances[start_pos.y][start_pos.x] = 0;
  enqueue(start_pos.x, start_pos.y);
  int x, y;
  while (dequeue(&x, &y)) {
    int current_dist = flood_distances[y][x];
    char directions[] = { 'N', 'E', 'S', 'W' };
    for (int i = 0; i < 4; i++) {
      char dir = directions[i];
      int dx, dy;
      getDirectionOffset(dir, &dx, &dy);
      int nx = x + dx;
      int ny = y + dy;
      if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        bool hasWall = false;
        switch (dir) {
          case 'N': hasWall = maze[y][x].N; break;
          case 'E': hasWall = maze[y][x].E; break;
          case 'S': hasWall = maze[y][x].S; break;
          case 'W': hasWall = maze[y][x].W; break;
        }

        if (!hasWall && flood_distances[ny][nx] > current_dist + 1) {
          flood_distances[ny][nx] = current_dist + 1;
          enqueue(nx, ny);
        }
      }
    }
  }
}
char getBestDirectionToStart() {
  int x = current_x;
  int y = current_y;
  char best_direction = 'N';
  int best_distance = 999;

  char directions[] = { 'N', 'E', 'S', 'W' };
  for (int i = 0; i < 4; i++) {
    char dir = directions[i];
    int dx, dy;
    getDirectionOffset(dir, &dx, &dy);
    int nx = x + dx;
    int ny = y + dy;

    if (nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
      bool hasWall = false;
      switch (dir) {
        case 'N': hasWall = maze[y][x].N; break;
        case 'E': hasWall = maze[y][x].E; break;
        case 'S': hasWall = maze[y][x].S; break;
        case 'W': hasWall = maze[y][x].W; break;
      }

      if (!hasWall && flood_distances[ny][nx] < best_distance) {
        best_distance = flood_distances[ny][nx];
        best_direction = dir;
      }
    }
  }

  return best_direction;
}
bool isMazeFullyExplored() {
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (!maze[y][x].visited) {
        return false;
      }
    }
  }
  return true;
}
void returnToStartWithExploration() {
  long cc = 0;
  bool maze_fully_explored = false;
  // MySerial.println("Strating from Center...");
  while (!atStart()) {
    cc++;
    updateMazeWalls();
    if (!maze_fully_explored) {
      maze_fully_explored = isMazeFullyExplored();
    }

    char exploration_dir = 'X';

    if (!maze_fully_explored) {
      exploration_dir = getUnvisitedNeighborDirection();
    }

    if (exploration_dir != 'X' && !maze_fully_explored) {
      if (exploration_dir != current_direction) {
        int turn_angle = directionToAngle(current_direction, exploration_dir);
        if (turn_angle != 0) {
          turnAngle(turn_angle, delay1);
          current_direction = exploration_dir;
        }
      }
    } else {
      floodFillToStart();
      char best_dir = getBestDirectionToStart();

      if (best_dir != current_direction) {
        int turn_angle = directionToAngle(current_direction, best_dir);
        if (turn_angle != 0) {
          turnAngle(turn_angle, delay1);
          current_direction = best_dir;
        }
      }
    }
    int x = current_x, y = current_y;
    Data = String(cc);
    Data += " x:" + String(x);
    Data += " y:" + String(y) + "\n";
    Data += "Current dir:" + String(current_direction) + "\n";
    Data += "N:" + String(maze[y][x].N);
    Data += " E:" + String(maze[y][x].E);
    Data += " S:" + String(maze[y][x].S);
    Data += " W:" + String(maze[y][x].W);
    GoTilesDefault();
    Data += "Traveled one tile on:" + String(current_direction);
    Serial.println(Data);
    //MySerial.println(Data);
    updatePosition();
  }
}
void runFloodfillMode(int run_number) {
  if (run_number == 1) {
    initializeMaze();
    current_direction = 'N';
    at_goal_flag = false;
    Data = "maze initialized \nstarting floodfill search";
    Serial.println(Data);
    //MySerial.println(Data);
    floodfillSearch();
    if (at_goal_flag) {
      run_number = 2;
      LEDBlink(red, 10);
      saveMazeAndFloodfill();
      // loadMazeAndFloodfill();
      at_goal_flag = false;
      returnToStartWithExploration();
      run_number = 3;
      LEDBlink(green, 10);
      floodFill();
      saveMazeAndFloodfill();
    }
  } else if (run_number == 2) {
  } else if (run_number == 3) {
    current_x = 0, current_y = 0;
    current_direction = 'N';
    loadMazeAndFloodfill();
    for (int y = MAZE_SIZE - 1; y > -1; y--) {
      Data = "";
      for (int x = 0; x < MAZE_SIZE; x++) {
        if (maze[y][x].N) {
          Data += "N";
        } else Data += " ";
        if (maze[y][x].E) {
          Data += "E";
        } else Data += " ";
        if (maze[y][x].S) {
          Data += "S";
        } else Data += " ";
        if (maze[y][x].W) {
          Data += "W";
        } else Data += " ";
        if (maze[y][x].visited) {
          Data += "V";
        } else Data += " ";
        Data += " ";
      }
      //MySerial.println(Data);
      delay(10);
    }
    for (int y = MAZE_SIZE - 1; y > -1; y--) {
      Data = "";
      for (int x = 0; x < MAZE_SIZE; x++) {
        Data += String(flood_distances[y][x]);
        Data += " ";
      }
      //MySerial.println(Data);
      delay(10);
    }
    int delay2 = 0;
    while (!atGoal()) {
      char best_dir = getBestDirection();
      if (best_dir != current_direction) {
        int turn_angle = directionToAngle(current_direction, best_dir);
        if (turn_angle != 0) {
          turnAngle(turn_angle, delay2);
          current_direction = best_dir;
        }
      }
      GoTiles(1, 75, 160, 0, 0, delay2);
      updatePosition();
    }
    LEDBlink(green, 5);
    floodFillToStart();
    while (!atStart()) {
      char best_dir = getBestDirection();
      if (best_dir != current_direction) {
        int turn_angle = directionToAngle(current_direction, best_dir);
        if (turn_angle != 0) {
          turnAngle(turn_angle, delay2);
          current_direction = best_dir;
        }
      }
      GoTiles(1, 75, 160, 0, 0, delay2);
      updatePosition();
    }
    run_number = 4;
    run = false;
  } else if (run_number == 4) {
    LEDBlink(blue, 2);
    current_x = 0;
    current_y = 0;
    current_direction = 'N';
    loadMazeAndFloodfill();
    executeSpeedRun4();
    run = false;
  } else {
    run = false;
  }
}
////////////////////////////////////////////////////////////////////////////////
void core0(void* parameter) {
  while (true) {
    read_tof_all();
    // String k = String(getOrientation());
    // hc12Send(k);
  }
}
void reset() {
  setMotorSpeed(0, 0);
  FwallTriggered = 0, send = 0;
}
void read_tof_all() {
  static float d_avg[5];
  static long d_total[5];
  static const int numOfSamples = 1;
  static int d_array[5][numOfSamples];
  static int readIndex = 0;
  //read raw values
  int d[5];
  d[0] = read_tof_distance(tof_front) - 20;
  if (d[0] > 60) d[0] = d[0] - 10;
  d[3] = read_tof_distance(tof_45_LEFT) - 40;
  if (d[3] > 60) d[3] = d[3] - 10;
  d[4] = read_tof_distance(tof_45_RIGHT);
  d[1] = read_tof_distance(tof_left) - 10;
  d[2] = read_tof_distance(tof_right) - 10;

  //averaging
  for (int i = 0; i < 5; i++) d_total[i] -= d_array[i][readIndex];
  for (int i = 0; i < 5; i++) d_array[i][readIndex] = d[i];
  for (int i = 0; i < 5; i++) d_total[i] += d_array[i][readIndex];
  readIndex++;
  if (readIndex >= numOfSamples) readIndex = 0;
  for (int i = 0; i < 5; i++) d_avg[i] = (float)d_total[i] / numOfSamples;
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int i = 0; i < 5; i++) dis_Reading[i] = d_avg[i];
  xSemaphoreGive(dataMutex);
}
void setupPins() {
  pinMode(LeftIN1, OUTPUT);
  pinMode(LeftIN2, OUTPUT);
  pinMode(RightIN1, OUTPUT);
  pinMode(RightIN2, OUTPUT);
  digitalWrite(LeftIN1, LOW);
  digitalWrite(LeftIN2, LOW);
  digitalWrite(RightIN1, LOW);
  digitalWrite(RightIN2, LOW);

  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(blue, OUTPUT);

  pinMode(encLeftA, INPUT);
  pinMode(encLeftB, INPUT);
  pinMode(encRightA, INPUT);
  pinMode(encRightB, INPUT);
}
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    analogWrite(LeftIN1, leftSpeed);
    analogWrite(LeftIN2, 0);
  } else if (leftSpeed < 0) {
    analogWrite(LeftIN1, 0);
    analogWrite(LeftIN2, -leftSpeed);
  } else {
    analogWrite(LeftIN1, 0);
    analogWrite(LeftIN2, 0);
  }
  if (rightSpeed > 0) {
    analogWrite(RightIN1, rightSpeed);
    analogWrite(RightIN2, 0);
  } else if (rightSpeed < 0) {
    analogWrite(RightIN1, 0);
    analogWrite(RightIN2, -rightSpeed);
  } else {
    analogWrite(RightIN1, 0);
    analogWrite(RightIN2, 0);
  }
}
void IRAM_ATTR isrEncLeft() {
  bool A = digitalRead(encLeftA);
  bool B = digitalRead(encLeftB);
  if (A == B) {
    encoderCountLeft++;
  } else {
    encoderCountLeft--;
  }
}
void IRAM_ATTR isrEncRight() {
  bool A = digitalRead(encRightA);
  bool B = digitalRead(encRightB);
  if (A == B) {
    encoderCountRight++;
  } else {
    encoderCountRight--;
  }
}
void setup_tof_sensors() {
  Wire.begin();
  pinMode(TOF_XSHUT_FRONT, OUTPUT);
  pinMode(TOF_XSHUT_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_RIGHT, OUTPUT);
  pinMode(TOF_XSHUT_45_LEFT, OUTPUT);
  pinMode(TOF_XSHUT_45_RIGHT, OUTPUT);

  digitalWrite(TOF_XSHUT_FRONT, LOW);
  digitalWrite(TOF_XSHUT_LEFT, LOW);
  digitalWrite(TOF_XSHUT_RIGHT, LOW);
  digitalWrite(TOF_XSHUT_45_LEFT, LOW);
  digitalWrite(TOF_XSHUT_45_RIGHT, LOW);
  delay(10);

  digitalWrite(TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (tof_front.init()) {
    tof_front.setAddress(0x30);
    tof_front.setTimeout(500);
    tof_front.startContinuous();
  }

  digitalWrite(TOF_XSHUT_LEFT, HIGH);
  delay(10);
  if (tof_left.init()) {
    tof_left.setAddress(0x31);
    tof_left.setTimeout(500);
    tof_left.startContinuous();
  }

  digitalWrite(TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  if (tof_right.init()) {
    tof_right.setAddress(0x32);
    tof_right.setTimeout(500);
    tof_right.startContinuous();
  }

  digitalWrite(TOF_XSHUT_45_LEFT, HIGH);
  delay(10);
  if (tof_45_LEFT.init()) {
    tof_45_LEFT.setAddress(0x33);
    tof_45_LEFT.setTimeout(500);
    tof_45_LEFT.startContinuous();
  }

  digitalWrite(TOF_XSHUT_45_RIGHT, HIGH);
  delay(10);
  if (tof_45_RIGHT.init()) {
    tof_45_RIGHT.setAddress(0x34);
    tof_45_RIGHT.setTimeout(500);
    tof_45_RIGHT.startContinuous();
  }

  tof_sensors_ready = true;
}
int read_tof_distance(VL53L0X& sensor) {
  if (!tof_sensors_ready) return 2;
  uint16_t distance_mm = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    return 2;
  }
  return distance_mm;
}
long DistanceToStep(float Distance) {
  const int wheelDiameter = 43;
  const int gearRatio = 58;
  const int ppr = 7;
  const float circumference = PI * wheelDiameter;
  return Distance / circumference * (gearRatio * ppr);
}
double StepToDistance(long step) {
  const int wheelDiameter = 43;  // mm
  const int gearRatio = 58;
  const int ppr = 7;
  const float circumference = PI * wheelDiameter;  // mm
  return ((double)step / (ppr * gearRatio)) * circumference;
}
void checkButtons() {
  unsigned long currentTime = millis();
  if (currentTime - lastReadTime < readInterval) return;  // limit reads
  lastReadTime = currentTime;

  int reading = analogRead(buttonPin);
  int pressedButton = 0;

  if (abs(reading - btn1Value) <= tolerance) pressedButton = 1;
  else if (abs(reading - btn2Value) <= tolerance) pressedButton = 2;
  else if (abs(reading - btn3Value) <= tolerance) pressedButton = 3;

  if (pressedButton != 0 && !prevPressed) {
    if (pressedButton == 1) {
      state1 = !state1;
      run = !run;
      Serial.print("Button 1 toggled: ");
      Serial.print(state1);
      Serial.print(" Run: ");
      Serial.println(run);
    } else if (pressedButton == 2) {
      state2 = !state2;
      run3 = !run3;
      Serial.print("Button 2 toggled: ");
      Serial.print(state2);
      Serial.print(" Run3: ");
      Serial.println(run3);
    } else if (pressedButton == 3) {
      state3 = !state3;
      run4 = !run4;  // renamed here
      Serial.print("Button 3 toggled: ");
      Serial.print(state3);
      Serial.print(" Run4: ");
      Serial.println(run4);
    }
    prevPressed = true;
  }

  if (pressedButton == 0) prevPressed = false;
}
int applyMinSpeed(int speed, int min_effective_speed) {
  if (abs(speed) <= min_effective_speed && speed != 0) {
    speed = (speed / abs(speed)) * min_effective_speed;  // preserve direction
  }
  return speed;
}
void LEDBlink(int pin, int times) {  //if 2 blink 2 times
  for (int i = 1; i <= times; i++) {
    digitalWrite(pin, HIGH);
    delay(100);
    digitalWrite(pin, LOW);
    delay(100);
  }
}
void setupCompleted() {
  digitalWrite(green, HIGH);
  delay(100);
  digitalWrite(green, LOW);
  delay(100);
  digitalWrite(green, HIGH);
  delay(100);
  digitalWrite(green, LOW);
  delay(100);
}
void processHC12Data(String input) {
  input.trim();
  if (input.length() == 0) return;
  Serial.println(input);

  // Handle "r" or "R" before looking for '='
  if (input.equalsIgnoreCase("r")) {
    String msg = "Current values:\n";
    msg += "Kp=" + String(Kp, 4) + "\n";
    msg += "Kd=" + String(Kd, 4) + "\n";
    msg += "Ki=" + String(Ki, 4) + "\n";
    msg += "KpDF=" + String(KpDF, 4) + "\n";
    msg += "KdDF=" + String(KdDF, 4) + "\n";
    msg += "KpDEn=" + String(KpDEn, 4) + "\n";
    msg += "KdDEn=" + String(KdDEn, 4) + "\n";

    // --- Curve PID values ---
    msg += "Kp_curve=" + String(Kp_curve, 4) + "\n";
    msg += "Kd_curve=" + String(Kd_curve, 4) + "\n";
    msg += "Ki_curve=" + String(Ki_curve, 4) + "\n";
    msg += "KdDEn_curve=" + String(KdDEn_curve, 4) + "\n";
    msg += "curveturnX=" + String(curveturnX, 4) + "\n";
    msg += "factor=" + String(factor, 4) + "\n";
    msg += "frontDisTh_curve=" + String(frontDisTh_curve) + "\n";
    msg += "ANG=" + String(ANG) + "\n";

    // --- Sensor thresholds ---
    msg += "diaglSideSensor_Thr=" + String(diaglSideSensor_Thr) + "\n";
    msg += "NondiagsideSensThr=" + String(NondiagsideSensThr) + "\n";
    msg += "diagFSideSensorX=" + String(diagFSideSensorX, 4) + "\n";
    msg += "diagSideSensorX=" + String(diagSideSensorX, 4) + "\n";
    msg += "NondiagsideSensX=" + String(NondiagsideSensX, 4) + "\n";
    msg += "curveTurnSideThr=" + String(curveTurnSideThr, 4) + "\n";
    msg += "curveTurnFSideThr=" + String(curveTurnFSideThr, 4) + "\n";
    msg += "delay1=" + String(delay1);
    MySerial.println(msg);
    return;
  }

  // Otherwise, look for '=' and parse normally
  int eqIndex = input.indexOf('=');
  if (eqIndex == -1) return;  // invalid format

  String key = input.substring(0, eqIndex);
  String valStr = input.substring(eqIndex + 1);
  key.toLowerCase();

  float val = valStr.toFloat();
  int valInt = valStr.toInt();

  if (key == "kp") Kp = val;
  else if (key == "kd") Kd = val;
  else if (key == "ki") Ki = val;
  else if (key == "kpdf") KpDF = val;
  else if (key == "kddf") KdDF = val;
  else if (key == "kpden") KpDEn = val;
  else if (key == "kdden") KdDEn = val;

  // --- Curve PID values ---
  else if (key == "kp_curve") Kp_curve = val;
  else if (key == "kd_curve") Kd_curve = val;
  else if (key == "ki_curve") Ki_curve = val;
  else if (key == "kdden_curve") KdDEn_curve = val;
  else if (key == "curveturnx") curveturnX = val;
  else if (key == "factor") factor = val;
  else if (key == "frontdisth_curve") frontDisTh_curve = valInt;
  else if (key == "ang") ANG = valInt;

  // --- Sensor thresholds ---
  else if (key == "diaglsidesensor_thr") diaglSideSensor_Thr = valInt;
  else if (key == "nondiagsidesensthr") NondiagsideSensThr = valInt;
  else if (key == "diagfsidesensorx") diagFSideSensorX = val;
  else if (key == "diagsidesensorx") diagSideSensorX = val;
  else if (key == "nondiagsidesensx") NondiagsideSensX = val;
  else if (key == "curveturnsidethr") curveTurnSideThr = val;
  else if (key == "curveturnfsidethr") curveTurnFSideThr = val;
  else if (key == "ch") mode = valInt;
  else if (key == "delay1") delay1 = valInt;
  else return;

  String msg = "Updated " + String(key) + " = " + String(valStr);
  MySerial.println(msg);
}
float arcL_cal(float r, int angle) {
  float theta = angle * M_PI / 180.0;  // convert degrees to radians
  return r * theta;                    // arc length
}
float diagonaltiles_Calc(int tiles) {
  const float halfDiagonalTile = (float)1 / 2 / cos(PI / 4);
  return halfDiagonalTile * tiles;
}
void hc12Send(String& msg) {
  static unsigned long st = 0;
  static bool pending = false;

  if (!pending) {
    pending = true;
    st = millis();
  } else if (millis() - st >= 10) {
    if (msg != "") {
      // msg += "\nEND";
      MySerial.println(msg);
    }
    pending = false;
  }
}
bool handleStop(slowStopData& state, float TargetEnc_Error, bool FwallTriggered) {
  if ((TargetEnc_Error <= 0) && !state.startofStopBool && millis() - state.run_stime > 200) {
    if (state.waitDelay == 0) {
      state.startofStopBool = false;
      setMotorSpeed(0, 0);
      reset();
      return 1;
    } else {
      state.startofStop = millis();
      state.startofStopBool = true;
      if (!FwallTriggered) digitalWrite(green, HIGH);
      else digitalWrite(red, HIGH);
      return 0;
    }
  }
  if (state.waitDelay > 0) {
    if (millis() - state.startofStop > state.waitDelay && state.startofStopBool) {
      state.startofStopBool = false;
      setMotorSpeed(0, 0);
      if (!FwallTriggered) digitalWrite(green, LOW);
      else digitalWrite(red, LOW);
      reset();
      return 1;
    }
  }
  return 0;
}
void SpeedAdj_Constrain(int speedLeft, int speedRight, float error, unsigned long run_stime, long diffCount, long targetClose_Step) {
  int LimBaseSpeed = buffsp[0], maxspeed = buffsp[1], min_effective_speed = buffsp[2];  // apply default go speeds
  if (millis() - run_stime <= SlowDelay) {                                              //starting slow
    maxspeed = LimBaseSpeed + (millis() - run_stime) / float(SlowDelay) * (maxspeed - LimBaseSpeed);
    LimBaseSpeed = min_effective_speed + (millis() - run_stime) / float(SlowDelay) * (LimBaseSpeed - min_effective_speed);
  }
  if (diffCount < targetClose_Step && targetClose_Step > 0) {  //ending slow
    maxspeed = 130;
    LimBaseSpeed = 100;
  }
  speedLeft = constrain(speedLeft, -LimBaseSpeed, LimBaseSpeed);                                                           //constraining base speed
  speedRight = constrain(speedRight, -LimBaseSpeed, LimBaseSpeed);                                                         //constraining base speed
  speedLeft = constrain(speedLeft - error, -maxspeed, maxspeed);                                                           //constraining combined speed
  speedRight = constrain(speedRight + error, -maxspeed, maxspeed);                                                         //constraining combined speed
  speedLeft = applyMinSpeed(speedLeft, min_effective_speed), speedRight = applyMinSpeed(speedRight, min_effective_speed);  //apply min effective speed
  setMotorSpeed(speedLeft, speedRight);                                                                                    //set final motor speed
  print += " LimBase:" + String(LimBaseSpeed) + " MaxSp:" + String(maxspeed);
  print += " SpL:" + String(speedLeft) + " SpR:" + String(speedRight);
}
float getOrientation() {
  noInterrupts();
  long leftCount = encoderCountLeft;
  long rightCount = encoderCountRight;
  interrupts();

  // Calculate delta counts
  long deltaLeft = leftCount - lastCountLeft;
  long deltaRight = rightCount - lastCountRight;
  lastCountLeft = leftCount;
  lastCountRight = rightCount;

  // Convert counts to distance
  float distPerCount = float(PI * 43) / 406;
  float dL = deltaLeft * distPerCount;
  float dR = deltaRight * distPerCount;

  // Odometry
  float dC = (dR + dL) / 2.0;
  float dTheta = (dR - dL) / disBetWheels;

  // Update position using midpoint method
  // posX += dC * cos(theta + dTheta / 2.0);
  // posY += dC * sin(theta + dTheta / 2.0);
  theta += dTheta;

  // Convert to degrees for printing
  float thetaDeg = theta * 180.0 / PI;
  return thetaDeg;
}
