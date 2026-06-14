/*
* Change Logs:
* Date            Author          Notes
* 2023-08-23      ChuShicheng     first version
*/

/* -----------------------------------------------机器人功能选择------------------------------------------------------ */

/* ----------------------RoboMaster Modules -------------------*/

//#define BSP_USING_MOTOR
#define BSP_USING_DJI_MOTOR
//#define BSP_USING_INA226
#define BSP_USING_MSG
//#define BSP_USING_IMU
//#define BSP_USING_BMI088
//#define BSP_BMI088_CALI
#define BSP_USING_RC_DBUS
#define BSP_USING_RC_KEYBOARD
// #define BSP_USING_SUPERCAP                //使用超电


/* end of RoboMaster Modules */

/* ----------------------RoboMaster Algorithms------------------- */

/*底盘功率限制开关*/
#define BSP_USING_DEFAULT_MODE
//#define BSP_USING_POWER_LIMIT


/* ------------------------RoboMaster Tasks（在rm_task.h中定义）------------------------ */

/*底盘类型选择*/
//#define BSP_CHASSIS_OMNI_MODE
#define BSP_CHASSIS_MECANUM_MODE

/* ------------------------------------------------------------------------------------------------------------ */

#define CPU_FREQUENCY 480     /* CPU主频(mHZ) */


#include "stm32h7xx_hal.h" // 使用的芯片
#include "cmsis_os2.h" // 使用的 OS 头文件

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

#include "FreeRTOS.h"
#include "task.h"
#define user_malloc pvPortMalloc
#define user_free vPortFree

/* 底盘和云台分别对应的 can 总线 */
#define CAN_CHASSIS    "hfdcan1"
#define CAN_OTHER      "hfdcan2"
#define CAN_GIMBAL     "hfdcan3"

/* ------------------------------------------------------- IMU相关 ------------------------------------------------ */
// 加速度计零偏值 需定期校准后手动修改,此为40度实测
// 陀螺仪零偏（手动标定，和旧工程一致）
#define GxOFFSET  0.000969390618f
#define GyOFFSET  0.00278034038f
#define GzOFFSET  0.00124635932f
//static float gyro_init[3] = {0};//用于零漂矫正，见ins线程


/* -------------------------------------------------------- 遥控器相关 ------------------------------------------------ */
#define RC_MAX_VALUE      784.0f  /* 遥控器通道最大值 */
#define RC_DBUS_MAX_VALUE      660.0f  /* DBUS遥控器通道最大值 */


#define RC_RATIO          0.0009f

#define KB_RATIO          0.010f
/* 遥控器模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_RC_MOVE_RATIO_X 0.5f
/* 底盘前进速度 */
#define CHASSIS_RC_MOVE_RATIO_Y 0.8f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_RC_MOVE_RATIO_R 0.85f//1.0f

/* 鼠标键盘模式下的底盘最大速度限制 */
/* 底盘平移速度 */
#define CHASSIS_PC_MOVE_RATIO_X 1.0f
/* 底盘前进速度 */
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f
/* 底盘旋转速度，只在底盘开环模式下使用 */
#define CHASSIS_PC_MOVE_RATIO_R 3.5f//5.0f


/* 遥控器模式下的云台速度限制 */


/* 云台pitch轴速度 */
#define GIMBAL_RC_MOVE_RATIO_PIT 0.2f
/* 云台yaw轴速度 */
#define GIMBAL_RC_MOVE_RATIO_YAW 0.35f

/* 鼠标键盘模式下的云台速度限制 */
/* 云台pitch轴速度 */
#define GIMBAL_PC_MOVE_RATIO_PIT 0.1f
/* 云台yaw轴速度 */
#define GIMBAL_PC_MOVE_RATIO_YAW 0.25f



/* ------------------------------------------------------ 底盘相关 ---------------------------------------------------- */
/* 底盘轮距(mm) */
#define WHEELTRACK       465// 340
/* 底盘轴距(mm) */
#define WHEELBASE        476//388

/* 底盘轮子周长(mm) */
#define WHEEL_PERIMETER   478

#define LENGTH_A 238 //底盘长的一半(mm)
#define LENGTH_B 233//232 //底盘宽的一半(mm)
#define LENGTH_RADIUS 333//332 //底盘半径(mm)
/******** 底盘电机使用3508 *******/
/* 3508底盘电机减速比 */
#define CHASSIS_DECELE_RATIO (1.0f/19.0f)
/* 单个电机速度极限，单位是分钟每转 */
#define MAX_WHEEL_RPM        9000   //8347rpm = 3500mm/s

/******** 底盘最大速度设置 *******/
/* 底盘移动最大速度，单位是毫米每秒 */
#define MAX_CHASSIS_VX_SPEED 5000
#define MAX_CHASSIS_VY_SPEED 5000

#define MAX_CHASSIS_VX_SPEED_HIGH 11000
#define MAX_CHASSIS_VY_SPEED_HIGH 11000

#define MAX_CHASSIS_VX_SPEED_LOW 5000
#define MAX_CHASSIS_VY_SPEED_LOW 5000

/* 底盘旋转最大速度，单位是度每秒 */
#define MAX_CHASSIS_VR_SPEED 8


/* --------------------------------- 底盘PID参数 -------------------------------- */
/* 电机速度环 */
#define CHASSIS_KP_V_MOTOR              3//5//3.2f
#define CHASSIS_KI_V_MOTOR              0.0f//0.1f
#define CHASSIS_KD_V_MOTOR              0.07//0.001f
#define CHASSIS_INTEGRAL_V_MOTOR        0
#define CHASSIS_MAX_V_MOTOR             16384
// TODO: 参数待整定
/* 跟随云台PID */
#define CHASSIS_KP_V_FOLLOW             0.45f//0.045f//0.1205f//0.1f
#define CHASSIS_KI_V_FOLLOW             0//0.001f
#define CHASSIS_KD_V_FOLLOW             0.005f//0.0008f
#define CHASSIS_INTEGRAL_V_FOLLOW       0
#define CHASSIS_MAX_V_FOLLOW            5

/* ----------------------------------------------------- 云台相关 --------------------------------------------------- */
#define YAW_MOTOR_ID     0x207
#define PITCH_MOTOR_ID   0x205

/* [0]为yaw，[1]为pitch */
#define YAW 0
#define PITCH 1


/*云台编码器归中*/
//#define GIMBAL_SIDEWAYS
#ifdef GIMBAL_SIDEWAYS
#define SIDEWAYS_ANGLE   36
#define CENTER_ECD_YAW   3818         //云台yaw轴编码器归中值(侧身)
#else
#define CENTER_ECD_YAW   635//2088         //云台yaw轴编码器归中值
#define SIDEWAYS_ANGLE   0
#endif


/* pitch轴最大仰角 */
#define PIT_ANGLE_MAX        4.0f//8.0f//15.0f
/* pitch轴最大俯角 */
#define PIT_ANGLE_MIN        -5.0f//-10//-18.0f

/* 云台控制周期 (ms) */
#define GIMBAL_PERIOD 1
/* 云台回中初始化时间 (ms) */
#define BACK_CENTER_TIME 100
#define INIT_TIMEOUT 1000  // 单位: ms 初始化归中超时时间


/* -------------------------------- 云台电机PID参数 ------------------------------- */
/* 云台yaw轴电机PID参数 */
/* imu速度环 */
#define YAW_KP_V_IMU             10000//5000//9250//12000
#define YAW_KI_V_IMU             200//200
#define YAW_KD_V_IMU             0.001f//0.01f//0.005f
#define YAW_INTEGRAL_V_IMU       200//1000 //2000
#define YAW_MAX_V_IMU            25000
/* imu角度环 */
#define YAW_KP_A_IMU             0.15f//0.1f//0.15f   //降低响应速度减小超调以提升稳定性
#define YAW_KI_A_IMU             0.003//0.001f
#define YAW_KD_A_IMU             0.003f//0.002f //0.005响应好，但是归中有振动
#define YAW_INTEGRAL_A_IMU       0.2f
#define YAW_MAX_A_IMU            5
/* auto速度环 */
#define YAW_KP_V_AUTO            8500//12000
#define YAW_KI_V_AUTO            200
#define YAW_KD_V_AUTO            0.005f
#define YAW_INTEGRAL_V_AUTO      2000
#define YAW_MAX_V_AUTO           25000
/* auto角度环 */
#define YAW_KP_A_AUTO            0.1f//0.2f
#define YAW_KI_A_AUTO            0.001f//0.008f
#define YAW_KD_A_AUTO            0.005f
#define YAW_INTEGRAL_A_AUTO      1.0f
#define YAW_MAX_A_AUTO           5

/* 云台PITCH轴电机PID参数 /*
/* imu速度环 */
/* imu速度环 */
#define PITCH_KP_V_IMU           1500//2000//800//1000//3000
#define PITCH_KI_V_IMU           40//0//50
#define PITCH_KD_V_IMU           0.1//0.001
#define PITCH_INTEGRAL_V_IMU     1000
#define PITCH_MAX_V_IMU          16384

/* imu角度环 */

#define PITCH_KP_A_IMU           0.18//0.2f
#define PITCH_KI_A_IMU           0.005f//0//0.05f
#define PITCH_KD_A_IMU           0.006f//0.001f
#define PITCH_INTEGRAL_A_IMU     1.0f
#define PITCH_MAX_A_IMU          10
// #define PITCH_KP_V_IMU           1000//3000
// #define PITCH_KI_V_IMU           100
// #define PITCH_KD_V_IMU           0.1
// #define PITCH_INTEGRAL_V_IMU     1000
// #define PITCH_MAX_V_IMU          16384
//
// /* imu角度环 */
//
// #define PITCH_KP_A_IMU           0.2f
// #define PITCH_KI_A_IMU           0.05f
// #define PITCH_KD_A_IMU           0.005f
// #define PITCH_INTEGRAL_A_IMU     1.0f
// #define PITCH_MAX_A_IMU          10

/* auto速度环 */
#define PITCH_KP_V_AUTO          1000//1500
#define PITCH_KI_V_AUTO          50//100
#define PITCH_KD_V_AUTO          0.001f//0.01f
#define PITCH_INTEGRAL_V_AUTO    1500
#define PITCH_MAX_V_AUTO         16384
/* auto角度环 */
#define PITCH_KP_A_AUTO          0.2f//0.25f
#define PITCH_KI_A_AUTO          0.05f
#define PITCH_KD_A_AUTO          0.001f
#define PITCH_INTEGRAL_A_AUTO    1.0f
#define PITCH_MAX_A_AUTO         10

/* ------------------------------------------------------- 发射相关 --------------------------------------------------- */
#define RIGHT_FRICTION_MOTOR_ID     0x201
#define LEFT_FRICTION_MOTOR_ID   0x202
#define MIDDLE_FRICTION_MOTOR_ID   0x203
#define TRIGGER_MOTOR_ID  0x205 //0x205

#define FRICTION_SPEED_ONE  6000
#define FRICTION_SPEED_CONTINUE  6000


#define TRIGGER_MOTOR_120_TO_ANGLE 120.0f
/* -------------------------------- 发射电机PID参数 ------------------------------- */
// TODO: 速度期望应改为变量应对速度切换。初次参数调整已完成
/* 右摩擦轮M3508电机PID参数 */
/* 速度环 */
#define RIGHT_KP_V             18
#define RIGHT_KI_V             0.0f
#define RIGHT_KD_V             0.001f
#define RIGHT_INTEGRAL_V       0
#define RIGHT_MAX_V            16384

/* 左摩擦轮M3508电机PID参数 */
/* 速度环 */
#define LEFT_KP_V           18
#define LEFT_KI_V           0.0f
#define LEFT_KD_V           0.001f
#define LEFT_INTEGRAL_V     0
#define LEFT_MAX_V          16384

/* 中摩擦轮M3508电机PID参数 */
/* 速度环 */
#define MIDDLE_KP_V           18
#define MIDDLE_KI_V           0.0f
#define MIDDLE_KD_V           0.001f
#define MIDDLE_INTEGRAL_V     0
#define MIDDLE_MAX_V          16384



// TODO：PID参数初次微调已完成，期待后续微调
/* 拨弹电机M3508电机PID参数 */
/* 速度环 */
#define TRIGGER_KP_V           5//10
#define TRIGGER_KI_V           3//5
#define TRIGGER_KD_V           0.0001f//0.01
#define TRIGGER_INTEGRAL_V     1500
#define TRIGGER_MAX_V          16384
/* 角度环 */
#define TRIGGER_KP_A           2
#define TRIGGER_KI_A           0.01
#define TRIGGER_KD_A           0.003
#define TRIGGER_INTEGRAL_A     5.0f
#define TRIGGER_MAX_A          10000


