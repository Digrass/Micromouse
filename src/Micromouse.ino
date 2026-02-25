#include <Arduino_LSM9DS1.h>

// 핀 설정
const int TRIG_F_PIN  = A5;
const int ECHO_F_PIN  = A4;
const int TRIG_L1_PIN = 5; 
const int ECHO_L1_PIN = 4;
const int TRIG_L2_PIN = A3;  
const int ECHO_L2_PIN = A2;
const int TRIG_R1_PIN = 3; 
const int ECHO_R1_PIN = 2;
const int TRIG_R2_PIN = A1;  
const int ECHO_R2_PIN = A0;
const int COLOR_SENSOR_PIN = 10; 

const int L_MOTOR_IN1 = 7;
const int L_MOTOR_IN2 = 6;
const int R_MOTOR_IN1 = 9;
const int R_MOTOR_IN2 = 8;

const int BLACK_STATE = LOW;  //컬러 센서가 검은색 감지 시의 출력
                                             
const int MAZE_SIZE = 8;

const int GOAL_MIN = 3;       //도착지 후보들의 x, y좌표의 최솟값과 최댓값
const int GOAL_MAX = 4;

const unsigned long t_1CELL_DEFAULT = 1200; //한 칸 전진 시 사용할 기본 주행 시간
const int R_MOTOR_BASE_SPEED = 115;         // 모터의 기본 속도
const int L_MOTOR_BASE_SPEED = 115;
const int F_WALL_THRESHOLD = 8;             // 벽 업데이트 시 전면 벽을 추가할 최대 거리
const int WALL_THRESHOLD = 15;              // 벽 업데이트 시 측면 벽을 추가할 최대 거리
const float CELL_LENGTH = 20.0;

const float GYRO_KP_DRIVE = 0.0; // 자이로 기반 주행 보정 시 사용할 이득 값 

const int goalList[4][2] = {{3,3}, {3,4}, {4,4}, {4,3}};
bool goalChecked[4] = {false, false, false, false}; // 각 후보지를 확인했는지 여부

enum Direction { EAST = 0, SOUTH = 1, WEST = 2, NORTH = 3 };

bool vWalls[9][8]; // 수직 방향 벽과 수평 방향 벽 저장할 배열
bool hWalls[8][9];
int cost[MAZE_SIZE][MAZE_SIZE]; // Flood Fill 알고리즘의 거리 비용
int curX = 0;
int curY = 0;
int curDir = EAST;

float currentAngle = 0.0;
float gyroOffsetZ = 0.0;
unsigned long lastGyroTime = 0; // 최근 자이로를 업데이트 한 시간 

void setup() {
  Serial.begin(115200); 
  Serial.println("System Initializing...");
  while(!Serial);                             //시리얼 디버깅을 위한 시리얼 의존성을 부여하는 코드로 실제 주행 시 제거
  pinMode(TRIG_F_PIN, OUTPUT); 
  pinMode(ECHO_F_PIN, INPUT);
  pinMode(TRIG_L1_PIN, OUTPUT); 
  pinMode(ECHO_L1_PIN, INPUT);
  pinMode(TRIG_L2_PIN, OUTPUT); 
  pinMode(ECHO_L2_PIN, INPUT);
  pinMode(TRIG_R1_PIN, OUTPUT); 
  pinMode(ECHO_R1_PIN, INPUT);
  pinMode(TRIG_R2_PIN, OUTPUT); 
  pinMode(ECHO_R2_PIN, INPUT);

  pinMode(COLOR_SENSOR_PIN, INPUT);

  pinMode(L_MOTOR_IN1, OUTPUT); 
  pinMode(L_MOTOR_IN2, OUTPUT);
  pinMode(R_MOTOR_IN1, OUTPUT); 
  pinMode(R_MOTOR_IN2, OUTPUT);

  if (!IMU.begin()) {
    Serial.println("IMU Failed!");
    while (1);
  }

  calibrateGyro();   //출발 위치는 4개의 코너 중 하나 이므로 주행 시작 시 임의의 코너에서 미로 위에서 봤을 때 차체의 앞이 시계 반대 방향을 바라보도록 하여 해당 방향 기준으로 동쪽을 설정하여 각도 오프셋 초기화 
  initMap();
  Serial.println("Setup Complete. Waiting 3s...");
  delay(3000);
  lastGyroTime = millis();
  currentAngle = 0.0; // 현재 각도
  curDir = EAST;   //현재 바라보는 방향으로 초기 방향은 동쪽
}

void loop() {
  updateGyro(); //각도 업데이트
  Serial.println(currentAngle);
  Serial.print("도");

  // 현재 위치가 도착지 후보 중 하나이고 바닥이 검은색일 시 정지, 검은색이 아니면 해당 후보 제외 후 탐색 재개
  for (int i = 0; i < 4; i++) {
    if (curX == goalList[i][0] && curY == goalList[i][1] && !goalChecked[i]) {
      if (digitalRead(COLOR_SENSOR_PIN) == BLACK_STATE) {
        stopMotors();
        Serial.println("GOAL REACHED!");
        while(1);
      } else {
        Serial.print("Cell ("); Serial.print(curX); Serial.print(","); Serial.print(curY);
        Serial.println(") is not black. Excluding...");
        goalChecked[i] = true; 
        calculateFloodFill();
      }
    }
  }

  printSensorLogs();
  
  // 초음파 센서로 벽 업데이트 및 cost 재계산
  updateWalls();
  calculateFloodFill();
  
  int nextDir = getNextDirection(); // 사이에 벽이 없는 인접한 칸 중 비용이 가장 낮은 방향 선택
  Serial.print("Next Dir: "); 
  Serial.println(nextDir);

  // 방향 조절 후 한 칸 이동 후 현재 위치 좌표 업데이트
  if (curDir != nextDir) {
    turnTo(nextDir);
    delay(2000);
  }

  moveOneCell();
  delay(2000);
  updateCoordinate(curX, curY, curDir);

  stopMotors();
  delay(200);
}

//디버깅용 센서 로그 출력 함수
void printSensorLogs() {
  float l1 = getDistance(TRIG_L1_PIN, ECHO_L1_PIN, 50);
  float r1 = getDistance(TRIG_R1_PIN, ECHO_R1_PIN, 50);
  float diff = l1 - r1;

  Serial.println("================ STATE LOG ================");
  Serial.print("COORD: ("); 
  Serial.print(curX); 
  Serial.print(", "); 
  Serial.print(curY);
  Serial.print(") | DIR: "); 
  Serial.println(curDir);
  
  Serial.print("SENSOR DIFF(L-R): "); 
  Serial.print(l1); 
  Serial.print(" - "); 
  Serial.print(r1); 
  Serial.print(" = "); 
  Serial.println(diff);

  if (abs(diff) > 5.0 && l1 < 20 && r1 < 20) {
    Serial.println("WARNING: Not Centered!");
  }
  Serial.println("===========================================");
}

// 자이로 센서 값 적분을 통해 현재 각도 계산
void updateGyro() {
  if (IMU.gyroscopeAvailable()) {
    float x, y, z;
    IMU.readGyroscope(x, y, z);

    unsigned long currentTime = millis();
    float dt = (currentTime - lastGyroTime) / 1000.0;
    lastGyroTime = currentTime;

    float dps = z - gyroOffsetZ;
    if (abs(dps) < 1.0) dps = 0;

    currentAngle += dps * dt;
  }
}

//한 칸 전진 함수
void moveOneCell() {
  float startDist = 999;
  float meanStartDist = 0;
  int dist_count = 0;

  //전방 벽과의 거리가 46cm 이하라면 2칸 이내의 벽과의 거리를 기반으로 1칸 이동하고 전방 2칸 이내에 벽이 없을 경우 시간 기반으로 1칸 이동
  for(int i = 0; i < 3 ; i++){
    startDist = getDistance(TRIG_F_PIN, ECHO_F_PIN, 150); 
    if (startDist < 46) {
      dist_count++;
    }
    meanStartDist += startDist / 3.0;
    delay(5);
  }

  float targetAngle = getTargetAngle(curDir);
  float targetDist = 0; 

  if (meanStartDist > 46) { 
    targetDist = 0;
  } else {
    if(meanStartDist < 30.0){
      targetDist = 5.5;
    } else {
      targetDist = 26.0;
    }     
  }
  
  bool useDistanceControl = (meanStartDist < 45.0 && meanStartDist > 5.0 && dist_count > 1);

  const float DIST_CLOSE_LIMIT = 3.8;
  const float DIST_FAR_LIMIT   = 6.0;
  const float REF_DIST         = 4.7;
  const float KP_CORRECT       = 18.0;
  const float KP_PARALLEL      = 12.0;
  
  const int START_KICK_PWM     = 30;    //정지 마찰을 극복하기 위한 초반 부스트

  Serial.print("[MOVE] Start(Mean): "); 
  Serial.print(meanStartDist);
  Serial.print(" Target: "); 
  Serial.println(targetDist);

  unsigned long startTime = millis();
  String stopReason = "TIMEOUT";

  while (true) {
    updateGyro();

    float currentDist = getDistance(TRIG_F_PIN, ECHO_F_PIN, 60);
    unsigned long elapsed = millis() - startTime;  

    if (currentDist < 5.5){
      stopReason = "WALL (<8.3cm)";
      break;
    }

    if (useDistanceControl) {
      if (currentDist <= targetDist) {
        stopReason = "DIST REACHED";
        break;
      }
      if (elapsed > t_1CELL_DEFAULT * 1.5 && useDistanceControl) {
        stopReason = "TIMEOUT (Dist)";
        break;
      }
    } else {
      if (elapsed > t_1CELL_DEFAULT) {
         stopReason = "TIMEOUT (Time)";
         break;
      }
    }

    float R1 = getDistance(TRIG_R1_PIN, ECHO_R1_PIN, 70);
    float R2 = getDistance(TRIG_R2_PIN, ECHO_R2_PIN, 70);
    float L1 = getDistance(TRIG_L1_PIN, ECHO_L1_PIN, 70);
    float L2 = getDistance(TRIG_L2_PIN, ECHO_L2_PIN, 70);

    int leftPWM = L_MOTOR_BASE_SPEED;
    int rightPWM = R_MOTOR_BASE_SPEED;

    //센서 기반 보정은 각 센서가 벽에서 너무 멀거나 가까운 경우 기준값과의 차이에 비례한 보정값을 알맞은 모터의 속도에 더하고 빼는 방식으로 셀 중앙에서 벽과 평행하게 주행하도록 함
    if (R1 < 20.0 && R2 < 20.0) {             //오른쪽 두 개의 센서가 모두 벽을 감지한 경우 오른쪽 벽을 기준으로 보정
      if (R1 <= DIST_CLOSE_LIMIT) {
        float diff = REF_DIST - R1;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM -= correction; 
        rightPWM += correction;
      }
      else if (R2 <= DIST_CLOSE_LIMIT) {
        float diff = REF_DIST - R2;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM += correction; 
        rightPWM -= correction;
      }
      else if (R1 >= DIST_FAR_LIMIT) {
        float diff = R1 - REF_DIST;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM += correction; 
        rightPWM -= correction;
      }
      else if (R2 >= DIST_FAR_LIMIT) {
        float diff = R2 - REF_DIST;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM -= correction; 
        rightPWM += correction;
      }
      else {
        float error = R1 - R2;
        int correction = (int)(error * KP_PARALLEL);
        leftPWM += correction;
        rightPWM -= correction;
      }
    }
    else if (L1 < 20.0 && L2 < 20.0) {          //오른쪽 벽이 감지되지 않고 왼쪽 벽이 감지된 경우 왼쪽 벽을 기준으로 보정
      if (L1 <= DIST_CLOSE_LIMIT) {
        float diff = REF_DIST - L1;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM += correction; 
        rightPWM -= correction;
      }
      else if (L2 <= DIST_CLOSE_LIMIT) {
        float diff = REF_DIST - L2;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM -= correction; 
        rightPWM += correction;
      }
      else if (L1 >= DIST_FAR_LIMIT) {
        float diff = L1 - REF_DIST;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM -= correction; 
        rightPWM += correction;
      }
      else if (L2 >= DIST_FAR_LIMIT) {
        float diff = L2 - REF_DIST;
        int correction = (int)(diff * KP_CORRECT);
        leftPWM += correction; 
        rightPWM -= correction;
      }
      else {
        float error = L1 - L2;
        int correction = (int)(error * KP_PARALLEL);
        leftPWM -= correction;
        rightPWM += correction;
      }
    }
    else {                                                  //  좌우 모두 벽이 없는 경우 자이로 기반 보정
      float angleError = targetAngle - currentAngle;
      while (angleError > 180.0) angleError -= 360.0;
      while (angleError <= -180.0) angleError += 360.0;

      int correction = (int)(angleError * GYRO_KP_DRIVE);
      leftPWM -= correction;
      rightPWM += correction;
    }

    if (elapsed < 100) {
      leftPWM += START_KICK_PWM;
      rightPWM += START_KICK_PWM;
    }

    leftPWM = constrain(leftPWM, 0, 255);
    rightPWM = constrain(rightPWM, 0, 255);

    controlMotor(L_MOTOR_IN1, L_MOTOR_IN2, leftPWM);
    controlMotor(R_MOTOR_IN1, R_MOTOR_IN2, rightPWM);
  }
  
  stopMotors();
  Serial.print("[END] Reason: "); 
  Serial.println(stopReason);
}

// 목표 방향으로 회전 함수
void turnTo(int nextDir) {
  Serial.print("[TURN] To: "); Serial.println(nextDir);
  
  float targetAngle = getTargetAngle(nextDir);
  
  unsigned long turnStartTime = millis();

  while (true) {
    
    updateGyro();
    
    float error = targetAngle - currentAngle;
    
    while (error > 180.0) error -= 360.0;
    while (error <= -180.0) error += 360.0;

    if (abs(error) < 6.0) {            // 관성과 자이로 센서의 지연 시간을 고려하여 오차가 6도 미만이 되면 정지
      break; 
    }

    if (millis() - turnStartTime > 3000) {
      break;
    }

    int turnSpeed = 0;
    
    if (abs(error) > 60.0) {      //목표 각도에 가까워지면 속도를 줄여 오차를 감소시키려 했으나 모터의 한계로 회전 도중 감속 시 멈추는 현상이 발생하여 같은 속도로 설정함
      turnSpeed = 140; 
    } else if (abs(error) > 30.0) {
      turnSpeed = 140;
    } else {
      turnSpeed = 140;
    }

    if (error > 0) { //각도 오차의 부호로 회전 방향 결정
      controlMotor(L_MOTOR_IN1, L_MOTOR_IN2, -turnSpeed); 
      controlMotor(R_MOTOR_IN1, R_MOTOR_IN2, turnSpeed);  
    } else {
      controlMotor(L_MOTOR_IN1, L_MOTOR_IN2, turnSpeed);
      controlMotor(R_MOTOR_IN1, R_MOTOR_IN2, -turnSpeed);
    }
  }
  
  stopMotors(); 
  delay(100);

  currentAngle = targetAngle; 
  
  curDir = nextDir;
}

float getTargetAngle(int dir) {
  switch (dir) {
    case EAST: return 0.0;
    case NORTH: return 90.0;
    case WEST: return 180.0;
    case SOUTH: return -90.0;
  }
  return 0.0;
}


//테두리가 모두 벽으로 막힌 8*8 미로의 맵 초기화 함수
void initMap() {
  for (int x=0; x<9; x++) 
    for (int y=0; y<8; y++) 
      vWalls[x][y] = false;

  for (int x=0; x<8; x++) 
    for (int y=0; y<9; y++) 
      hWalls[x][y] = false;

  for (int y=0; y<8; y++) { 
    vWalls[0][y] = true; 
    vWalls[8][y] = true; 
  }
  for (int x=0; x<8; x++) { 
    hWalls[x][0] = true; 
    hWalls[x][8] = true; 
  }
}

//이동 전 벽 업데이트 함수
void updateWalls() {
  bool wallF = false;
  bool wallL = false;
  bool wallR = false;
  float f = 99;
  float l = 99;
  float r = 99;
  
  for(int i = 0; i<5 ; i++){ 
    f = getDistance(TRIG_F_PIN, ECHO_F_PIN, 70); 
    Serial.println(f);
    delay(80);
    l = getDistance(TRIG_L1_PIN, ECHO_L1_PIN, 70);
    Serial.println(l);
    delay(80);
    r = getDistance(TRIG_R1_PIN, ECHO_R1_PIN, 70);
    Serial.println(r);
    delay(80);
    if(f < F_WALL_THRESHOLD) wallF = true;
    if(l < WALL_THRESHOLD) wallL = true;
    if(r < WALL_THRESHOLD) wallR = true;
  }

  Serial.print("[WALLS] F:"); 
  Serial.print(wallF);
  Serial.print(" L:"); 
  Serial.print(wallL);
  Serial.print(" R:"); 
  Serial.println(wallR);

  setWallInMap(curX, curY, curDir, wallF);
  setWallInMap(curX, curY, (curDir + 3) % 4, wallL);
  setWallInMap(curX, curY, (curDir + 1) % 4, wallR);
}

void setWallInMap(int x, int y, int dir, bool exists) {
  if (!exists) return;
  if (x < 0 || x >= MAZE_SIZE || y < 0 || y >= MAZE_SIZE) return;

  switch (dir) {
    case EAST:  vWalls[x + 1][y] = true; break;
    case WEST:  vWalls[x][y] = true; break;
    case NORTH: hWalls[x][y + 1] = true; break;
    case SOUTH: hWalls[x][y] = true; break;
  }
}

void updateCoordinate(int &x, int &y, int dir) {
  switch(dir) {
    case EAST: x++; break;
    case WEST: x--; break;
    case NORTH: y++; break;
    case SOUTH: y--; break;
  }
}


// 초기 각도 오프셋 초기화 함수
void calibrateGyro() {
  Serial.println("Calibrating Gyro...");
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    if (IMU.gyroscopeAvailable()) {
      float x, y, z;
      IMU.readGyroscope(x, y, z);
      sum += z;
    }
    delay(5);
  }
  gyroOffsetZ = sum / 200.0;
  Serial.print("Gyro Offset Z: "); 
  Serial.println(gyroOffsetZ);
}

void controlMotor(int pin1, int pin2, int speed) {
  if (speed > 0) {
    analogWrite(pin1, speed);
    analogWrite(pin2, 0);
  } else if (speed < 0) {
    analogWrite(pin1, 0);
    analogWrite(pin2, abs(speed));
  } else {
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
  }
}

void stopMotors() {
  controlMotor(L_MOTOR_IN1, L_MOTOR_IN2, 0);
  controlMotor(R_MOTOR_IN1, R_MOTOR_IN2, 0);
}

float getDistance(int trig, int echo, int maxDistCm) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  unsigned long timeout = maxDistCm * 100;
  long duration = pulseIn(echo, HIGH, timeout);
  
  if (duration == 0) return 999.0;
  return duration * 0.017;
}

//Flood Fill 알고리즘
void calculateFloodFill() {
  for (int x=0; x<MAZE_SIZE; x++)
    for (int y=0; y<MAZE_SIZE; y++)
      cost[x][y] = 999;

  int qx[64], qy[64];
  int head = 0, tail = 0;
  
  //아직 확인하지 않은 도착지 후보들을 시작점으로 cost 계산
  bool anyGoalLeft = false;
  for (int i = 0; i < 4; i++) {
    if (!goalChecked[i]) {
      int gx = goalList[i][0];
      int gy = goalList[i][1];
      cost[gx][gy] = 0;
      qx[tail] = gx; qy[tail] = gy; tail++;
      anyGoalLeft = true;
    }
  }

  // 중앙 2*2칸 모두가 도착지 아닐 때 예외처리
  if (!anyGoalLeft) {
    Serial.println("No Goals In The Center 2*2");
    while(1);
  }

  while (head < tail) {
    int cx = qx[head]; int cy = qy[head]; head++;
    int c = cost[cx][cy];

    // 인접 칸 탐색 (벽이 없고 방문하지 않은 칸)
    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};
    
    for(int i=0; i<4; i++) {
      int nx = cx + dx[i];
      int ny = cy + dy[i];
      
      if(nx >= 0 && nx < MAZE_SIZE && ny >= 0 && ny < MAZE_SIZE) {
        // 벽 체크 로직 (vWalls, hWalls 참조)
        bool wall = false;
        if(dx[i] == 1) wall = vWalls[cx+1][cy];
        else if(dx[i] == -1) wall = vWalls[cx][cy];
        else if(dy[i] == 1) wall = hWalls[cx][cy+1];
        else if(dy[i] == -1) wall = hWalls[cx][cy];

        if(!wall && cost[nx][ny] == 999) {
          cost[nx][ny] = c + 1;
          qx[tail] = nx; qy[tail] = ny; tail++;
        }
      }
    }
  }
}

int getNextDirection() {
  int bestDir = curDir;
  int minCost = 1000;

  if (!vWalls[curX+1][curY] && cost[curX+1][curY] < minCost) {
    minCost = cost[curX+1][curY]; 
    bestDir = EAST;
  }
  if (!vWalls[curX][curY] && cost[curX-1][curY] < minCost) {
    minCost = cost[curX-1][curY]; 
    bestDir = WEST;
  }
  if (!hWalls[curX][curY+1] && cost[curX][curY+1] < minCost) {
    minCost = cost[curX][curY+1]; 
    bestDir = NORTH;
  }
  if (!hWalls[curX][curY] && cost[curX][curY-1] < minCost) {
    minCost = cost[curX][curY-1]; 
    bestDir = SOUTH;
  }
  return bestDir;
}
