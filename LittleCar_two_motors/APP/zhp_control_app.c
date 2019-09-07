
#include "zhp_control_app.h"


ctrl_val_typedef ctrl_val;
uint16_t motor_a_pwm;
uint16_t motor_b_pwm;

double velocity, angular;

float pwm_resolution;

/*********************************************
 *功能：解析出遥控器数据
 *********************************************/
static void ctrl_val_update()
{
  //油门
  if (rc.Tim2Ch2CapVal >= CTRL_VAL_LIMIT_H)
  {
    ctrl_val.ctrl_ly = CTRL_VAL_LIMIT_H;
  }
  else if (rc.Tim2Ch2CapVal <= CTRL_VAL_LIMIT_L)
  {
    ctrl_val.ctrl_ly = CTRL_VAL_LIMIT_L;
  }
//  else if (rc.Tim2Ch2CapVal < VERIFYED_MIDPOINT_H && rc.Tim2Ch2CapVal > VERIFYED_MIDPOINT_L)
//  {
//    ctrl_val.ctrl_ly = CTRL_VAL_LIMIT_M;
//  }
  else
  {
    ctrl_val.ctrl_ly = rc.Tim2Ch2CapVal;
  }

  //方向
  if (rc.Tim2Ch1CapVal >= CTRL_VAL_LIMIT_H)
  {
    ctrl_val.ctrl_rx = CTRL_VAL_LIMIT_H;
  }
  else if (rc.Tim2Ch1CapVal <= CTRL_VAL_LIMIT_L)
  {              
    ctrl_val.ctrl_rx = CTRL_VAL_LIMIT_L;
  }       
//  else if (rc.Tim2Ch1CapVal < VERIFYED_MIDPOINT_H && rc.Tim2Ch1CapVal > VERIFYED_MIDPOINT_L)
//  {
//    ctrl_val.ctrl_rx = CTRL_VAL_LIMIT_M;
//  }
  else
  {
    ctrl_val.ctrl_rx = rc.Tim2Ch1CapVal;
  }

  //两段开关
  if (rc.Tim3Ch3CapVal > VERIFYED_MIDPOINT_H)
  {
    ctrl_val.ctrl_en_state = CTRL_ENABLE;
  }
  else if (rc.Tim3Ch3CapVal < VERIFYED_MIDPOINT_L)
  {
    ctrl_val.ctrl_en_state = CTRL_DISABLE;
  }
  else
  {
    ctrl_val.ctrl_en_state = CTRL_OFF;
  }

  //三段开关
  if (rc.Tim3Ch4CapVal >= CTRL_TRI_SWI_JUDGE_H)
  {
    ctrl_val.ctrl_dat_state = CTRL_TRI_SWI_L;
  }
  else if (rc.Tim3Ch4CapVal <= CTRL_TRI_SWI_JUDGE_L)
  {
    ctrl_val.ctrl_dat_state = CTRL_TRI_SWI_H;
  }
  else
  {
    ctrl_val.ctrl_dat_state = CTRL_TRI_SWI_M;
  }

  uint32_t t_thr = CTRL_VAL_LIMIT_M, t_dir = CTRL_VAL_LIMIT_M;

  sscanf((char *)final_rx_frame, "%d %d", &t_thr, &t_dir);
  if (t_thr >= CTRL_VAL_LIMIT_H)
  {
    ctrl_val.ctrl_comp_thr = CTRL_VAL_LIMIT_H;
  }
  else if (t_thr <= CTRL_VAL_LIMIT_L)
  {
    ctrl_val.ctrl_comp_thr = CTRL_VAL_LIMIT_L;
  }
  else
  {
    ctrl_val.ctrl_comp_thr = t_thr;
  }
  if (t_dir >= CTRL_VAL_LIMIT_H)
  {
    ctrl_val.ctrl_comp_dir = CTRL_VAL_LIMIT_H;
  }
  else if (t_dir <= CTRL_VAL_LIMIT_L)
  {
    ctrl_val.ctrl_comp_dir = CTRL_VAL_LIMIT_L;
  }
  else
  {
    ctrl_val.ctrl_comp_dir = t_dir;
  }
}


/*********************************************
 *功能：根据解析出的遥控器数据控制小车电机，
 *无PID算法版本
 *********************************************/
static void speed_control(uint32_t thr, uint32_t dir)
{
  int16_t temp_motor_a_pwm;
  int16_t temp_motor_b_pwm;
  if (thr >= VERIFYED_MIDPOINT_H )
  {
    temp_motor_a_pwm = pwm_resolution * (thr - VERIFYED_MIDPOINT_H);
    temp_motor_b_pwm = pwm_resolution * (thr - VERIFYED_MIDPOINT_H);
                        
    if(dir < VERIFYED_MIDPOINT_L)
    {
      //temp_motor_a_pwm += pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
      temp_motor_b_pwm -= TURN_SENSITIVE * pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
    }
    else if (dir > VERIFYED_MIDPOINT_H)
    {
      temp_motor_a_pwm -= TURN_SENSITIVE * pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
      //temp_motor_b_pwm += pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
    }

    temp_motor_a_pwm = - temp_motor_a_pwm;
    temp_motor_b_pwm = - temp_motor_b_pwm;

  }
  else if (thr <= VERIFYED_MIDPOINT_L)
  {
    temp_motor_a_pwm = pwm_resolution * (VERIFYED_MIDPOINT_L - thr);
    temp_motor_b_pwm = pwm_resolution * (VERIFYED_MIDPOINT_L - thr);

    if(dir < VERIFYED_MIDPOINT_L)
    {
      //temp_motor_a_pwm += pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
      temp_motor_b_pwm -= TURN_SENSITIVE * pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
    }
    else if (dir > VERIFYED_MIDPOINT_H)
    {
      temp_motor_a_pwm -= TURN_SENSITIVE * pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
      //temp_motor_b_pwm += pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
    }
  }
  else if(dir < VERIFYED_MIDPOINT_L)
  {
    temp_motor_a_pwm =   pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
    temp_motor_b_pwm = - pwm_resolution*(VERIFYED_MIDPOINT_L - dir);
  }
  else if (dir > VERIFYED_MIDPOINT_H)
  {
    temp_motor_a_pwm = - pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
    temp_motor_b_pwm =   pwm_resolution*(dir - VERIFYED_MIDPOINT_H);
  }
  else
  {
    temp_motor_a_pwm = 0;
    temp_motor_b_pwm = 0;
    MOTOR_A_Brake();
    MOTOR_B_Brake();
  }


  if (temp_motor_a_pwm > 0)
  {                          
    MOTOR_A_SetForward();  
  }
  else if (temp_motor_a_pwm < 0)
  {                     
    MOTOR_A_SetBackward();
  }
  else
  {
    MOTOR_A_Brake();
  }
  if (temp_motor_b_pwm > 0)
  {                          
    MOTOR_B_SetForward();  
  }
  else if (temp_motor_b_pwm < 0)
  {                     
    MOTOR_B_SetBackward();
  }
  else
  {
    MOTOR_B_Brake();
  }
  
  if (abs(temp_motor_a_pwm) > TIMER_CYCLE)
  {
    car_pwm.motor_a_pwm = TIMER_CYCLE;
  }
  else
  {
    car_pwm.motor_a_pwm = abs(temp_motor_a_pwm);
  }
  if (abs(temp_motor_b_pwm) >= TIMER_CYCLE)
  {
    car_pwm.motor_b_pwm = TIMER_CYCLE;
  }
  else
  {
    car_pwm.motor_b_pwm = abs(temp_motor_b_pwm);
  }


  MOTOR_A_SetPwm(car_pwm.motor_a_pwm);
  MOTOR_B_SetPwm(car_pwm.motor_b_pwm);
}

pid_t spd_pid_l = 
{
  .ek = 0,
  .p = 1,
  .i = 0,
  .d = 0,
  .target = 0,
  .pout = 0,
  .iout = 0,
  .dout = 0,
  .out =0,
};
pid_t spd_pid_r = 
{
  .ek = 0,
  .p = 1,
  .i = 0,
  .d = 0,
  .target = 0,
  .pout = 0,
  .iout = 0,
  .dout = 0,
  .out =0,
};
/*********************************************
 *功能：通过PID算法控制小车电机
 *********************************************/
void pid_speed_control(uint32_t thr, uint32_t dir)
{
  
}

/*********************************************
 *功能：遥控器控制小车
 *********************************************/
void zhp_remote_control(void)
{
  ctrl_val_update();
  if (ctrl_val.ctrl_en_state == CTRL_ENABLE) //判断小车运动功能是否使能
  {
    speed_control(ctrl_val.ctrl_ly, ctrl_val.ctrl_rx); 
  }
  else
  {
    MOTOR_A_Brake();
    MOTOR_B_Brake();
  }
}
      
/*********************************************
 *功能：电脑控制小车
 *********************************************/
void zhp_computer_control()
{                      
  ctrl_val_update();
  if (ctrl_val.ctrl_en_state == CTRL_ENABLE) //判断小车运动功能是否使能
  {
    speed_control(ctrl_val.ctrl_comp_thr, ctrl_val.ctrl_comp_dir);
  }
  else
  {
    MOTOR_A_Brake();
    MOTOR_B_Brake();
  }
}
