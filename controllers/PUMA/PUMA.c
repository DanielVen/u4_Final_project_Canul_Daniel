/*
 * File:          PUMA.c
 * Date: 21/07/2019
 * Description: Final project
 * Author: Canul Daniel, Chacon Iván, Ventura Cesar, Ek Gustavo.
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/differential_wheels.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include <stdio.h>
#include <math.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

/*
Variables
 */
 int vel_actual=0;
 int vel_actual2=0;
 int vel_anterior=0;
 int vel_anterior2=0;
 double kp=5.7; //proportional gain
 double kp2=5.7;
 double ki=0.0; //ganancia integral
 double ki2=0.0;
 double Ea=0; //Error acumulado
 double Ea2=0;
 double Ep=0; //Error
 double Ep2=0;
 double Act_position=0.0; // actual position
 double Act_position2=0.0;
 double Ref_position= -1; //Reference position
 double Ref_position2=-1.60;
 double position_value; //sensor lecture
 double velocity1=0; // velocity variable
 double velocity2=0; //velocity variable 2

 double PIcontroller (double Ref_position, double Act_position) {
   Ea = fabs(Ref_position - Act_position);
   vel_actual = vel_anterior + (Ea * (kp + (ki * (TIME_STEP/1000)))) - (kp * Ep);
   vel_anterior = vel_actual;
   Ep=Ea;
   printf("error%lf\t acumulated eror%lf\t \n", Ep, Ea);
   
    if (Ref_position < 0){
     vel_actual = vel_actual * (-1);
    }
  return vel_actual;
 }
 
 double PIcontroller2 (double Ref_position2, double Act_position2) {
   Ea2 = fabs(Ref_position2 - Act_position2);
   vel_actual2 = vel_anterior2 + (Ea2 * (kp2 + (ki2 * (TIME_STEP/1000)))) - (kp2 * Ep2);
   vel_anterior2 = vel_actual2;
   Ep2=Ea2;
   printf("error2%lf\t acumulated eror2%lf\t \n", Ep2, Ea2);
   
    if (Ref_position2 < 0) {
     vel_actual2 = vel_actual2 * (-1);
    }
    
  return vel_actual2;
 }
 
int main(int argc, char **argv) {

    wb_robot_init();

    WbDeviceTag link1 = wb_robot_get_device("DC_MOTOR1");
    WbDeviceTag link2 = wb_robot_get_device("DC_MOTOR2");
    WbDeviceTag link3 = wb_robot_get_device("SERVO_MOTOR1");
    WbDeviceTag Gripper = wb_robot_get_device("SERVO_MOTOR2");

    WbDeviceTag Position_sen1 = wb_robot_get_device("ENCODER1");
    WbDeviceTag Position_sen2 = wb_robot_get_device("ENCODER2");

    wb_position_sensor_enable(Position_sen1, TIME_STEP);
    wb_motor_set_position(link1, INFINITY);
    wb_position_sensor_enable(Position_sen2, TIME_STEP);
    wb_motor_set_position(link2, INFINITY);

  while (wb_robot_step(TIME_STEP) != -1) {

   Act_position = wb_position_sensor_get_value(Position_sen1);
   
    if(velocity1 > 0){
       ki = 5;
    }
    ////////////////////////////////////////////////////
   velocity1 = PIcontroller(Ref_position, Act_position);
   
    if(velocity1 > wb_motor_get_max_velocity(link1)) {
      velocity1 = wb_motor_get_max_velocity(link1);
    }
    
// For velocity2

    if(velocity2 > 0) {
       ki2 = 6;
    }
    
   velocity2 = PIcontroller2(Ref_position2, Act_position2);
   
    if(velocity2 > wb_motor_get_max_velocity(link2)) {
      velocity2 = wb_motor_get_max_velocity(link2);
    }
    
    ////////////////////////////////////////////////////
    Act_position2 = wb_position_sensor_get_value(Position_sen2);
    printf("position %lf\t position 2 %lf\n", Act_position, Act_position2);
    printf("velocity 1 %lf\t velocity 2 %lf\t \n", velocity1, velocity2);

    wb_motor_set_velocity(link1, velocity1);
    wb_motor_set_velocity(link2, velocity2);
    wb_motor_set_position(link3, -0.5);
    wb_motor_set_position(Gripper, 0.78);
  };

  wb_robot_cleanup();

  return 0;
}
