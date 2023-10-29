#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#define LEFT 0
#define RIGHT 1
#define TIME_STEP 32
#define NB_GROUND_SENS 8
//센서 개수 8개

typedef struct{
double min;
double max;
double nor;
}value; // 센서 각각의 최소, 최대, 정규화수치값

void Turn_Left(value a4[]);
void Turn_Right(value a5[]);
// IR Ground Sensors
WbDeviceTag gs[NB_GROUND_SENS];
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};
int heavyvalue[8] = {-8, -4, -2, -1, 1, 2, 4, 8}; //가중치
int x=0;
double lineposition, error;
int speed[2] = {500, 500}; // Speed initialization

// Motors
WbDeviceTag left_motor, right_motor;

void ReadGroudSensors(void){
  for(int i=0; i<NB_GROUND_SENS; i++){
    gs_value[i] = wb_distance_sensor_get_value(gs[i]);
  }
}//센서 인식 위치 받아오는 함수

void MinMax(value* a1, unsigned short arr[]){
  for(int i=0;i<NB_GROUND_SENS;i++){
    if(x==0){//처음에 센서값을 한 개 얻었을 경우
      a1[i].min = arr[i];
      a1[i].max = arr[i];}

    else if(x>0){//이후에 작동하는 경우
      if(arr[i]<a1[i].min)
        a1[i].min = arr[i];
      else if(arr[i]>a1[i].max)
        a1[i].max = arr[i];
      else
        continue;}
  }
x++;
}//min, max 값 새로 설정하는 함수

void Normalize(value* a2, unsigned short arr[]){
  for(int i=0;i<NB_GROUND_SENS;i++){
    a2[i].nor = (arr[i]-a2[i].min)/(a2[i].max-a2[i].min); 
  }
}//센서마다 정규화하는 함수

void LinePosition(value* a3){
  double first = 0, second = 0;

  for(int i=0;i<NB_GROUND_SENS;i++){
    first += a3[i].nor * heavyvalue[i]; //분자
    second += a3[i].nor; //분모
  }
  lineposition = first / second; //선의 위치 계산
}//선의 위치 계산하여 반환하는 함수

void Error(double line){
  error = 0 - line;
}//오류값 계산하는 함수

int main() {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /* initialization */
  char name[20];
  value sens[NB_GROUND_SENS] = {{0, 0, 0}, }; //센서 구조체 배열 포인터 선언
  
  for (int i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors, gs0을 gs[0]에 연결 */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }//센서 측정 시작
  
  // motors, 왼쪽/오른쪽 바퀴 설정
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  for (;;) {
    // Run one simulation step
    wb_robot_step(TIME_STEP);

    ReadGroudSensors();//1~8번째 센서가 가져온 값을 gs_value배열에 저장
    
    MinMax(sens, gs_value);//최대 최소 계산
    
    Normalize(sens, gs_value);//정규화수치 계산

    LinePosition(sens);//선의 위치 계산

    Error(lineposition);//오류값 계산
    
    // Speed computation
    wb_motor_set_velocity(left_motor, 0.03 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.03 * speed[RIGHT]);
    
    if(x > 10){
      if((gs_value[0] > 400) && (gs_value[3] > 300 || gs_value[4] > 300)){
        if(x < 1800 && x > 1200){
          if((gs_value[0] > 400 || gs_value[1] > 400) && (gs_value[6] > 400 || gs_value[7] > 400))
            continue;}
        else if(x > 2000){
          if(gs_value[0] > 400 && gs_value[7] > 400){
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            break;} //정지선 도착
        }
        Turn_Left(sens);}
    
      else if((gs_value[2] > 300 || gs_value[3] > 300) && (gs_value[6] > 400 || gs_value[7] > 400)){
        if(x < 1800 && x > 1200){
          if((gs_value[0] > 400 || gs_value[1] > 400) && (gs_value[6] > 400 || gs_value[7] > 400))
            continue;}
        else if(x > 2000){
          if(gs_value[0] > 400 && gs_value[7] > 400){
            wb_motor_set_velocity(left_motor, 0.0);
            wb_motor_set_velocity(right_motor, 0.0);
            break;} //정지선 도착
        }
        Turn_Right(sens);}
    
      else{
        if(error > 0){
          wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] - 7.5 * error);
          wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] + 15 * error);}
        else if(error < 0){
          wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] - 15 * error);
          wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] + 7.5 * error);}
      }
    }
  }
  wb_robot_cleanup();

  return 0;
}
void Turn_Left(value* a4){ //좌회전
  printf("Left\n");
  int cnt = 0;    
  wb_motor_set_velocity(left_motor, 0.03 * speed[LEFT]);
  wb_motor_set_velocity(right_motor, 0.03 * speed[RIGHT]);
  while(1){
    wb_robot_step(TIME_STEP);
    ReadGroudSensors();
    MinMax(a4, gs_value);
    Normalize(a4, gs_value);
    LinePosition(a4);
    Error(lineposition);
    
   if((gs_value[0] > 400 || gs_value[1] > 400) && (gs_value[3] > 300 || gs_value[4] > 300)){
     wb_motor_set_velocity(left_motor, 0.03 * speed[LEFT]);
     wb_motor_set_velocity(right_motor, 0.03 * speed[RIGHT]);}
   else{
     if(error < -1){
       wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] + 0.65 * error);
       wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] - 0.33 * error);}
     else{
       wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] - 7.5 * error);
       wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] + 15 * error);}
   }
   
   if(cnt > 3){
     if((gs_value[0] > 400 || gs_value[1] > 400) && (gs_value[3] > 250 || gs_value[4] > 250)){
       printf("Left End\n");
       break;}
     else if(gs_value[6] > 400 || gs_value[7] > 400){
       Turn_Right(a4);
       break;}
   }
   cnt++;}
}

void Turn_Right(value* a5){ //우회전
  printf("Right\n");
  int cnt = 0;
  wb_motor_set_velocity(left_motor, 0.03 * speed[LEFT]);
  wb_motor_set_velocity(right_motor, 0.03 * speed[RIGHT]);
  while(1){
    wb_robot_step(TIME_STEP);
    ReadGroudSensors();
    MinMax(a5, gs_value);
    Normalize(a5, gs_value);
    LinePosition(a5);
    Error(lineposition);
  
    if((gs_value[3] > 250 || gs_value[4] > 250) && (gs_value[6] > 400 || gs_value[7] > 400)){
      wb_motor_set_velocity(left_motor, 0.03 * speed[LEFT]);
      wb_motor_set_velocity(right_motor, 0.03 * speed[RIGHT]);}
    else{
      if(error > 1){
        wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] + 0.65 * error);
        wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] - 0.33 * error);}
      else{
        wb_motor_set_velocity(left_motor, 0.02826 * speed[LEFT] - 15 * error);
        wb_motor_set_velocity(right_motor, 0.02826 * speed[RIGHT] + 7.5 * error);}
    }
    
    if(cnt > 3){
      if((gs_value[3] > 300 || gs_value[4] > 300) && (gs_value[6] > 400 || gs_value[7] > 400)){
        printf("Right End\n");
        break;}
      else if(gs_value[0] > 400 || gs_value[1] > 400){
        Turn_Left(a5);
        break;}
    }
    cnt++;}
}
