/*
 * File:          wheelRobot.c
 * Date:
 * Description:   可以平衡、行走、转弯，采用虚拟模型控制，
                 腿部具有弹性，提高抗干扰能力，能过多个坡，能上下蹲
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "include/arx.h"
#include "include/print.h"
#include "include/PID.h"
#include "include/motor.h"
#include "include/position_sensor.h"
#include "include/inertial_unit.h"
#include "include/accelerometer.h"
#include "include/gyro.h"
#include "include/keyboard.h"
#include "include/mouse.h"
#include "include/types.h"
#include "include/robot.h"
#include "include/mathFuch.h"
#include "include/distance_sensor.h"
/*
 * You may want to add macros here.
 */
#define TIME_STEP 1

Balence_robot wlr;
float hl=0.4, hr=0.4;       //左右两边的期望高度
int time=0,num=0, t = 0;              //时实时间
int flag_forward, flag_back, flag_left, flag_right, up, down,jump; //前进、后退、左转、右转、上升、下蹲标志位
float p[6];
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
 
int main(int argc, char **argv) {

  /* necessary to initialize webots stuff */
  wb_robot_init();
  //机器人初始化
  RobotInit();

  printf("init_over\n");  

  //wb_display_draw_text(arx.display.ID,"t", 100, 90);
  //arx.motor[KNEE_R].angle = -0.2;
  while (wb_robot_step(5) != -1) {
  
    //速度角度检测, 数据存入全局变量中
    VelocityDetect();
   // DistanceDetect();
    AngleDetect();//角度, IMU
    GyroDetect(); //角速度
   
    time++;
    TorqueControl(); 
   
    if(time%4==0) { //20ms进入一次
        
        BalancePIDcal();
      if(time%8==0) //40ms进入一次
      {
         time=0;
         //上位机控制
         ReveiveMessage();
      }   
     }
  }

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  return 0;
}

//初始化函数
void RobotInit() {
  wlr.height_of_center = 0;
  wlr.mass_body = 3.77;
  wlr.mass_wheel = 0.75;
  wlr.radius_of_wheel = 0.076;
  wlr.module = NORMAL;
  wlr.displacement = 0;
  wlr.displacement_last = 0;
  wlr.velocity_target = 0;

  //最初的标定
  MotorInit(0);
  PositionSensorInit();
  ImuInit();
  AcceInit();
  GyroInit();
  KeyMouseInit();
}

void MotorInit(double angle_set) {
  //每加一个电机, (1)在宏定义中修改MOTOR_NUM 赋值, (2)增加代号的数量, (3)在这里写上name
 
  wlr.motor[0].name = "left__motor1";
  wlr.motor[1].name = "left__motor2";
  wlr.motor[2].name = "left__motor3";
  wlr.motor[3].name = "right_motor1";
  wlr.motor[4].name = "right_motor2"; //右
  wlr.motor[5].name = "right_motor3"; //左
 
  int i;
  for( i = 0; i < MOTOR_NUM; i++) {
    //获取电机ID
    wlr.motor[i].ID = wb_robot_get_device(wlr.motor[i].name);
    assert(wlr.motor[i].ID);
    //获取最大扭矩
    wlr.motor[i].MAX_TORQUE = wb_motor_get_max_torque(wlr.motor[i].ID);
    //使能扭矩反馈
    int sampling_period;
    sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(wlr.motor[i].ID);
    wb_motor_enable_torque_feedback(wlr.motor[i].ID, sampling_period);
    //归零
    wlr.motor[i].torque = 0;
    wlr.motor[i].omg = 0;
    wlr.motor[i].torque_fb = 0;
    wlr.motor[i].angle = angle_set;
    printf("get motor %s succeed: %d\n", wlr.motor[i].name, wlr.motor[i].ID);
  }
};

void PositionSensorInit() {
  wlr.position_sensor[0].name = "left__pos1";
  wlr.position_sensor[1].name = "left__pos2";
  wlr.position_sensor[2].name = "left__pos3";
  wlr.position_sensor[3].name = "right_pos1";
  wlr.position_sensor[4].name = "right_pos2";
  wlr.position_sensor[5].name = "right_pos3";

  int i;
  for( i = 0; i < MOTOR_NUM; i++) {
  wlr.position_sensor[i].ID = wb_robot_get_device(wlr.position_sensor[i].name);
  assert(wlr.position_sensor[i].ID);
  wb_position_sensor_enable(wlr.position_sensor[i].ID, (int)TIME_STEP);
  printf("get position senser %s succeed: %d\n",wlr.position_sensor[i].name, wlr.position_sensor[i].ID);

  wlr.position_sensor[i].position = 0;
  wlr.position_sensor[i].position_last = 0;
  wlr.position_sensor[i].w = 0;
  wlr.position_sensor[i].w_last = 0;
  }
};

void ImuInit() {
  wlr.imu.name = "imu";
  wlr.imu.ID = wb_robot_get_device(wlr.imu.name);
  wb_inertial_unit_enable(wlr.imu.ID, (int)TIME_STEP);
  wlr.imu.angle_value[yaw]   = 0;
  wlr.imu.angle_value[pitch] = 0;
  wlr.imu.angle_value[roll]  = 0;
  printf("imu over!\n");
};

void AcceInit() {
  wlr.acce.accelerometer_name = "Acce";
  wlr.acce.accelerometer_ID = wb_robot_get_device(wlr.acce.accelerometer_name);
  wb_accelerometer_enable(wlr.acce.accelerometer_ID, (int)TIME_STEP);
  printf("accelerometer over!\n");
}

void GyroInit() {
  wlr.gyro[0].gyro_ID  = wb_robot_get_device("gyro");
  wlr.gyro[1].gyro_ID = wb_robot_get_device("gyro1");
  wlr.gyro[2].gyro_ID = wb_robot_get_device("gyro2");
  wlr.gyro[3].gyro_ID = wb_robot_get_device("gyro3");
  wlr.gyro[4].gyro_ID = wb_robot_get_device("gyro4");
  for(int i=0;i<5;i++) {
    wb_gyro_enable(wlr.gyro[i].gyro_ID, (int)TIME_STEP);
    wlr.gyro[i].gyro_value[yaw]   = 0;
    wlr.gyro[i].gyro_value[pitch] = 0;
    wlr.gyro[i].gyro_value[roll]  = 0;
  }
};

void KeyMouseInit() {
  wb_keyboard_enable((int)TIME_STEP);
  wb_mouse_enable((int)TIME_STEP);
};

//传感器检测函数
/*
void DistanceDetect() {
   wlr.distance.ID = wb_robot_get_device ("dis_left");
   wb_distance_sensor_enable(wlr.distance.ID, (int)TIME_STEP);
}
*/
void AngleDetect() {
  assert(wlr.imu.ID);
  wlr.imu.angle_value[roll]  = wb_inertial_unit_get_roll_pitch_yaw(wlr.imu.ID)[roll];
  wlr.imu.angle_value[pitch] = wb_inertial_unit_get_roll_pitch_yaw(wlr.imu.ID)[pitch];
  wlr.imu.angle_value[yaw]   = wb_inertial_unit_get_roll_pitch_yaw(wlr.imu.ID)[yaw];
  //IMU/GYRO信息
  // printf("0---0yaw: %.3f\t 0pitch: %.3f\t 0roll: %.3f\t", 
  // dgr(wlr.imu.angle_value[yaw]), dgr(wlr.imu.angle_value[pitch]), dgr(wlr.imu.angle_value[roll]));
};

void GyroDetect() {
  assert(wlr.gyro.gyro_ID);
  for(int i=0;i<5;i++) {
    wlr.gyro[i].gyro_value[0] = wb_gyro_get_values(wlr.gyro[i].gyro_ID)[0];
    wlr.gyro[i].gyro_value[1] = wb_gyro_get_values(wlr.gyro[i].gyro_ID)[1];
    wlr.gyro[i].gyro_value[2] = wb_gyro_get_values(wlr.gyro[i].gyro_ID)[2];
  }
 // printf("w---Wyaw: %.3f\t Wpitch: %.3f\t Wroll: %.3f\t", 
  //dgr(wlr.gyro.gyro_value[yaw]), dgr(wlr.gyro.gyro_value[pitch]), dgr(wlr.gyro.gyro_value[roll]));
};

void VelocityDetect() {   //轮询所有电机传感器
  
  for( int i = 0; i < MOTOR_NUM; i++) {
    assert(wlr.position_sensor[i].ID);
    wlr.position_sensor[i].position = wb_position_sensor_get_value(wlr.position_sensor[i].ID);//GET THE POSITION
    wlr.position_sensor[i].w = wlr.position_sensor[i].position - wlr.position_sensor[i].position_last;//CAL THE ANGULAR V
    wlr.position_sensor[i].position_last = wlr.position_sensor[i].position;
    wlr.motor[i].torque_fb = wb_motor_get_torque_feedback(wlr.motor[i].ID);
  }
};

float ForceX(float xd, float diff_xd, float x, float diff_x, float kx, float bx) {//期望位置  期望速度  实际位置  实际速度  虚拟弹簧刚度 虚拟阻尼系数
   float bias_x, bias_diff_x;  //x方向位置偏差  x方向速度偏差
   float Fx;                   //虚拟模型的水平作用力
   bias_x = xd - x;      
   bias_diff_x = diff_xd - diff_x;            		                                                                  		                     
   Fx = kx*bias_x + bx*bias_diff_x;  
  // printf("bias_x:%f    bias_diff_x:%f  \r\n",bias_x ,bias_diff_x);
   return Fx;
}

float ForceY(float yd, float diff_yd, float y, float diff_y, float ky, float by) {  
   float bias_y, bias_diff_y;    //y方向位置偏差  y方向速度偏差
   float Fy;                     //虚拟模型的垂直作用力
   bias_y = yd - y ;
   bias_diff_y = diff_yd - diff_y;
   Fy = ky*bias_y+ by*bias_diff_y;  
  // printf("bias_y:%f    diff_y:%f \r\n",bias_y ,diff_y);
   return Fy;
}

float VaryingHeight(float angle,float gyro) {  
   float bias;     
   float v_h;          
   bias = angle-0; 
   v_h = 1.5*bias + 0*gyro;
   //printf("bias:%f    gyro:%f  ",bias ,gyro);
   return v_h;
}

//5ms进入一次
void TorqueControl () {
    t++;
    static float xl_least, yl_least, xr_least, yr_least;  //记录上一次的x和y方向的位置
    float force_xl, force_xr, force_yl, force_yr;
    float roll_theta = wlr.imu.angle_value[roll];  //绕x轴旋转角度
    
   
    float roll_omega = wlr.gyro[0].gyro_value[0];  //绕x轴旋转角速度
    float th1 = wlr.position_sensor[0].position + 0.6232; //左大腿与垂直方向的夹角 
    float th2 = wlr.position_sensor[1].position + 1.8462;  //左大腿与左小腿之间的夹角
    float th3 = wlr.position_sensor[3].position + 0.6232;
    float th4 = wlr.position_sensor[4].position + 1.8462;
    
    if(t == 4) {
    
       printf(": %.5f: " , roll_theta);
       printf("%.5f: %.5f : %.5f: %.5f :",th1,th2,th3,th4);
       t = 0;
    
    }
   
    float xl=0.3*sin(th1) - 0.2*sin(th1 +th2); //左足端相对于髋关节x方向上的位置
    float yl=0.3*cos(th1) - 0.2*cos(th1 +th2); //左足端相对于髋关节y方向上的位置
    float xr=0.3*sin(th3) - 0.2*sin(th3 +th4);
    float yr=0.3*cos(th3) - 0.2*cos(th3 +th4);
    
    float diff_xl = ( xl -  xl_least)*200; //平均时间内位移的变化量为速度   乘以200是为了将5ms/s单位换算成1m/s
    float diff_yl = ( yl -  yl_least)*200;  //位移的单位为 m
    float diff_xr = ( xr -  xr_least)*200;
    float diff_yr = ( yr -  yr_least)*200;
    float variable_h = VaryingHeight(roll_theta,roll_omega);    //过坡时腿应伸长或压缩的长度
    
     if (jump == 1) {
           num++;
           if(num>0 &&num<20)   
          {
             force_xl = ForceX( 0, 0.5, xl, diff_xl, 500, 18);      //左腿虚拟模型的水平作用力
             force_xr = ForceX( 0, 0.5, xr, diff_xr, 500, 18);  
             force_yl = ForceY( 0.55,0, yl, diff_yl, 700, 40);    //900  40
             force_yr = ForceY( 0.55, 0, yr, diff_yr,700, 40);
          }
          
          else if(num>=20 && num <40) 
          {
             force_xl = ForceX( 0, 0, xl, diff_xl, 500, 18);      //左腿虚拟模型的水平作用力
             force_xr = ForceX( 0, 0, xr, diff_xr, 500, 18);  
             force_yl = ForceY( 0.18, 0, yl, diff_yl, 700, 40); 
             force_yr = ForceY( 0.18, 0, yr, diff_yr, 700, 40);
          }
          else {
            jump=0;
            num=0;
            force_xl = ForceX( 0, 0, xl, diff_xl, 500, 18);      //左腿虚拟模型的水平作用力
            force_xr = ForceX( 0, 0, xr, diff_xr, 500, 18);  
            force_yl = ForceY( hl-variable_h, 0, yl, diff_yl, 700, 40); 
            force_yr = ForceY( hl+variable_h, 0, yr, diff_yr, 700, 40) ;
          }
      }
      else {
         force_xl = ForceX( 0, 0, xl, diff_xl, 500, 18);      //左腿虚拟模型的水平作用力
         force_xr = ForceX( 0, 0, xr, diff_xr, 500, 18);     //500 60
         force_yl = ForceY( hl-variable_h, 0, yl, diff_yl, 700, 40); //左腿虚拟模型的垂直作用力
         force_yr = ForceY( hr+variable_h, 0, yr, diff_yr, 700, 40) ;  //900 40
      }
    xl_least = xl;  //记录上次的位置
    yl_least = yl;
    xr_least = xr;
    yr_least = yr;       
    
    wlr.motor[HIP_L].torque  =  (0.3*cos(th1)-0.2*cos(th1+th2))*force_xl +(-0.3*sin(th1)+0.2*sin(th1+th2)) *force_yl ;  //左髋关节的期望力矩
    wlr.motor[HIP_R].torque  =  (0.3*cos(th3)-0.2*cos(th3+th4))*force_xr +(-0.3*sin(th3)+0.2*sin(th3+th4)) *force_yr ;
    wlr.motor[KNEE_L].torque =  -0.2*cos(th1+th2)*force_xl +0.2*sin(th1+th2)*force_yl;                                  //左膝关节的期望力矩
    wlr.motor[KNEE_R].torque =  -0.2*cos(th3+th4)*force_xr +0.2*sin(th3+th4)*force_yr;
    
    if (wlr.motor[HIP_L].torque > 8)    wlr.motor[HIP_L].torque  = 8;
    if (wlr.motor[HIP_L].torque < -8)   wlr.motor[HIP_L].torque  = -8;
    if (wlr.motor[HIP_R].torque > 8)    wlr.motor[HIP_R].torque  = 8;
    if (wlr.motor[HIP_R].torque < -8)   wlr.motor[HIP_R].torque  = -8;
    if (wlr.motor[KNEE_L].torque > 20)  wlr.motor[KNEE_L].torque = 20;
    if (wlr.motor[KNEE_L].torque < -20) wlr.motor[KNEE_L].torque = -20;
    if (wlr.motor[KNEE_R].torque > 20)  wlr.motor[KNEE_R].torque = 20;
    if (wlr.motor[KNEE_R].torque < -20) wlr.motor[KNEE_R].torque = -20;
    
    
     
    wb_motor_set_torque(wlr.motor[HIP_L].ID,wlr.motor[HIP_L].torque);      //设置左右膝关节髋关节扭矩
    wb_motor_set_torque(wlr.motor[KNEE_L].ID,wlr.motor[KNEE_L].torque);
    wb_motor_set_torque(wlr.motor[HIP_R].ID,wlr.motor[HIP_R].torque);
    wb_motor_set_torque(wlr.motor[KNEE_R].ID,wlr.motor[KNEE_R].torque); 
     
}



//20ms进入一次
void BalancePIDcal() {
    float Motor1,Motor2, Balance_Pwm,Velocity_Pwm,Turn_Pwm;   
    float mechanical_angle = 0;                                 //中值角度
    float pitch_theta = dgr(wlr.imu.angle_value[pitch]);        //pitch倾角 rad
    
    float pitch_omega = dgr( wlr.gyro[0].gyro_value[2]);        //绕z轴角速度 rad/s 
    float yaw_omega   = dgr(wlr.gyro[0].gyro_value[1]);          //绕y轴角速度 rad/s
    wlr.motor[WHEEL_L].torque_fb = wb_motor_get_torque_feedback(wlr.motor[WHEEL_L].ID);
    wlr.motor[WHEEL_R].torque_fb = wb_motor_get_torque_feedback(wlr.motor[WHEEL_R].ID);
    wlr.motor[WHEEL_L].omg = wb_motor_get_velocity(wlr.motor[WHEEL_L].ID);    //左边电机的真实角速度 rad/s 
    wlr.motor[WHEEL_R].omg = wb_motor_get_velocity(wlr.motor[WHEEL_R].ID);    //右边电机的真实角速度 rad/s 
    

    Balance_Pwm  = BalanceUp( pitch_theta, mechanical_angle, pitch_omega);    
    Velocity_Pwm = Velocity( wlr.motor[WHEEL_L].omg ,wlr.motor[WHEEL_R].omg);   
    Turn_Pwm = Turn(yaw_omega);	                 
    Motor1=(Balance_Pwm - Velocity_Pwm + Turn_Pwm)/240;  //左边电机的期望速度
    Motor2=(Balance_Pwm - Velocity_Pwm - Turn_Pwm)/240;  //右边电机的期望速度
    if(Motor1<-30 ) Motor1=-30 ;       //左右电机角速度限幅
    if(Motor1>30 )  Motor1=30 ;
    if(Motor2<-30 ) Motor2=-30;
    if(Motor2>30 )  Motor2=30 ;
    if (num>20 && num <40) {
      Motor1=0;
      Motor2=0;
   }
    if( flag_forward == 1 || flag_back == 1 ||flag_left == 1 || flag_right == 1 || up==1 || down==1 ) {  //及时将标志位清零
        flag_forward = 0;
        flag_back = 0;
        flag_left = 0;
        flag_right = 0;
        up=0;
        down=0;
     }
    wb_motor_set_position(wlr.motor[2].ID,INFINITY);
    wb_motor_set_position(wlr.motor[5].ID,INFINITY);     
    wb_motor_set_velocity(wlr.motor[2].ID,-1*Motor1);     //设置左右电机角速度
    wb_motor_set_velocity(wlr.motor[5].ID,-1*Motor2);
    for (int i = 0; i < 6; i++) {
        if (i == 2 || i == 5) p[i] =  wlr.motor[i].torque_fb* 200*wlr.position_sensor[i].w;
        else   p[i] = wlr.motor[i].torque* 200*wlr.position_sensor[i].w;
    }
   
   // printf("实际速度: %.5f   : %.5f    : %.5f    : %.5f : %.5f : %.5f \r\n", 200*wlr.position_sensor[0].w, 200*wlr.position_sensor[1].w, 200*wlr.position_sensor[2].w, 200*wlr.position_sensor[3].w, 200*wlr.position_sensor[4].w, 200*wlr.position_sensor[5].w);
   // printf("实际扭矩 :%f  :%f  :%f  :%f  :%f  :%f \r\n", wlr.motor[0].torque_fb, wlr.motor[1].torque_fb, wlr.motor[2].torque_fb, wlr.motor[3].torque_fb ,wlr.motor[4].torque_fb, wlr.motor[5].torque_fb);
   // printf("实际速度: %.5f   : %.5f  \r\n",200*wlr.position_sensor[2].w, 200*wlr.position_sensor[5].w);
   // printf("轮子处期望速度: %.5f : %.5f \t\n",-Motor1,-Motor2);
    printf("%.5f: %.5f : %.5f : %.5f:",wlr.motor[0].torque, wlr.motor[1].torque, wlr.motor[3].torque,wlr.motor[4].torque);
   // printf("功率: %.5f   : %.5f    : %.5f    : %.5f : %.5f : %.5f \r\n", p[0],p[1],p[2],p[3],p[4],p[5]);
     printf("%.5f \r\n", pitch_theta);
}
//上位机函数  40ms进入一次
void ReveiveMessage() {
  int key = -1;
  //mouseNow = wb_mouse_get_state();
  key = wb_keyboard_get_key();
  if (key != -1) {
    // update var according to 'key' value
    switch (key) {
      case 'Q'://前进
        flag_forward = 1;      
        break;
      case 'H': //后退
       flag_back = 1;
       break;
      case 'L'://左转
       flag_left = 1;
       hl = 0.32;
       hr = 0.42;
      case 'R'://右转
       flag_right = 1;
       hl = 0.42;
       hr = 0.32;
       break;
      case 'U'://上升
      if(hl>0.31&&hl<0.43)
      {
        up=1;   
        hl+=0.005;
        hr+=0.005;
        if( hl>=0.43)
        {
          hl=0.42;
          hr=0.42;
          break;
        }
      }
        break;
      case 'D'://下降
      if(hl>0.31&&hl<0.43)
      {
        down =1;
        hl-=0.005;
        hr-=0.005;
        if(hl<=0.31)
        { 
          hl=0.32;
          hr=0.32;
          break;
        }
      }
       break;
      case 'T':
     
        jump=1;
        break;
      default:
        break;
     }
   }
}; 

//直立环
float BalanceUp(float angle,float mechanical_balance,float gyro) {  
   float balance_up_kp=-180, balance_up_kd=-20;  //直立环kp  kd    -180,-45
   float bias, balance_pwm;
   bias  = angle-mechanical_balance;  
   balance_pwm =balance_up_kp*bias+balance_up_kd*gyro;  
   //printf("bias:%f  gyro:%f  \r\n",bias,gyro);  		                            
   return balance_pwm;
}

//速度环
float Velocity(float motor1_v,float motor2_v) {  
    float velocity_kp=180, velocity_ki=1;     //速度环kp  ki  180 1
    static float velocity_pwm,velocity_least,velocity, velocity_integral,movement;
    if(1==flag_forward) {  
       movement = -60; 
    }
     else if(1==flag_back) {
        movement = 60;
    }
     else {	
       movement = 0;
    }   		
    velocity_least =(motor1_v+motor2_v)-0;                     		
    velocity =velocity* 0.7f;		                                             
    velocity =velocity + velocity_least*0.3f;	                                                                            
    		
    velocity_integral += velocity ;                                                           
    velocity_integral -= movement ;//接收遥控数据，控制前进后退           		                     
  
    if(velocity_integral> 10000)  	velocity_integral= 10000;           
    if(velocity_integral<-10000)	velocity_integral=-10000;           	
    		
    velocity_pwm = velocity*velocity_kp + velocity_integral*velocity_ki;    
    if(wlr.imu.angle_value[pitch]<-30 || wlr.imu.angle_value[pitch] >30)  velocity_integral=0; 
   // printf("velocity:%f  velocity_integral:%f  \r\n",velocity,velocity_integral);  		                                						
    return velocity_pwm;
}

float Turn(float gyro) {
    float turn_pwm,  turn_kp= 25;	
    if(flag_left == 1)        turn_pwm=-1000;
    else if(flag_right == 1)  turn_pwm=1000;
    else  turn_pwm =  gyro*turn_kp;
    return turn_pwm;		
}

