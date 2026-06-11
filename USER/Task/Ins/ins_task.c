/**
 * @file ins_task.c
 * @brief INS (Inertial Navigation System) task
 * @date 2026-06-07
 * @author Jasper
 */

#include "rm_task.h"
#include "rm_module.h"
#include "ins_task.h"
#include "cmsis_os2.h"

/* ----------------------------------------------- IMU温度控制（关闭） --------------------------------------------------- */
#define IMU_TARGET_TEMP           40

static uint32_t pulse = 0;
static float temp;
// static pid_obj_t *imu_temp_pid;
// static pid_config_t imu_temp_config = {
//         .Kp = 500,
//         .Ki = 100,
//         .Kd = 10,
//         .IntegralLimit = 4000,
//         .Improve = PID_Integral_Limit,
//         .MaxOut = 10000,
// };

/* ----------------------------------------------- IMU数据解算 --------------------------------------------------- */
static ins_t ins;
static imu_param_t imu_param;

static float acc_init[3] = {0};
static float gyro_init[3] = {0};

/* ----------------------------------------------- 零偏校准 --------------------------------------------------- */
float GxOFFSET = GxOFFSET_DEFAULT;
float GyOFFSET = GyOFFSET_DEFAULT;
float GzOFFSET = GzOFFSET_DEFAULT;

// 静止检测 + 连续零偏估计
static float bias_lp[3] = {0};
static const float REST_GYRO_TH = 0.05f;   // 静止阈值 (rad/s)
static const float BIAS_ALPHA = 0.001f;     // 零偏更新系数

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static void ins_init(void);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3]);
static void InitQuaternion(float *init_q4);

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static struct ins_msg ins_data;

static publisher_t *ins_pub;

static void ins_pub_init(void);

/* ------------------------------------------------- ins线程入口 ------------------------------------------------------ */
static float ins_dt;
static uint32_t count = 0;

void InsTask_Entry(void *argument)
{
    static float ins_start;
    static uint32_t ins_dwt = 0;
    static float dt = 0;
    const float gravity[3] = {0, 0, 9.81f};

    // 温控关闭，只启动PWM
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    // imu_temp_pid = pid_register(&imu_temp_config);

    while (BMI088_init())
    {
        ;
    }

    /* 启动时校准陀螺仪零偏（需静止500ms） */
    {
        float gyro_sum[3] = {0};
        uint8_t is_rest = 1;
        for (int i = 0; i < 500; i++)
        {
            BMI088_read(ins.gyro, ins.accel, &temp);
            float norm = sqrtf(ins.gyro[0] * ins.gyro[0] +
                               ins.gyro[1] * ins.gyro[1] +
                               ins.gyro[2] * ins.gyro[2]);
            if (norm > REST_GYRO_TH * 3)
                is_rest = 0;
            gyro_sum[0] += ins.gyro[0];
            gyro_sum[1] += ins.gyro[1];
            gyro_sum[2] += ins.gyro[2];
            osDelay(1);
        }
        if (is_rest)
        {
            GxOFFSET = gyro_sum[0] / 500.0f;
            GyOFFSET = gyro_sum[1] / 500.0f;
            GzOFFSET = gyro_sum[2] / 500.0f;
        }
        bias_lp[0] = GxOFFSET;
        bias_lp[1] = GyOFFSET;
        bias_lp[2] = GzOFFSET;
    }

    ins_init();
    ins_pub_init();

    for(;;)
    {
        ins_start = dwt_get_time_ms();

        dt = dwt_get_delta(&ins_dwt);
        BMI088_read(ins.gyro, ins.accel, &temp);

        // 减去陀螺仪零偏
        ins.gyro[0] -= GxOFFSET;
        ins.gyro[1] -= GyOFFSET;
        ins.gyro[2] -= GzOFFSET;

        // 静止检测：持续更新零偏估计
        {
            float gyro_raw[3] = {ins.gyro[0] + GxOFFSET, ins.gyro[1] + GyOFFSET, ins.gyro[2] + GzOFFSET};
            float gyro_norm = sqrtf(gyro_raw[0]*gyro_raw[0] + gyro_raw[1]*gyro_raw[1] + gyro_raw[2]*gyro_raw[2]);
            if (gyro_norm < REST_GYRO_TH) {
                bias_lp[0] += BIAS_ALPHA * (gyro_raw[0] - bias_lp[0]);
                bias_lp[1] += BIAS_ALPHA * (gyro_raw[1] - bias_lp[1]);
                bias_lp[2] += BIAS_ALPHA * (gyro_raw[2] - bias_lp[2]);
                GxOFFSET = bias_lp[0];
                GyOFFSET = bias_lp[1];
                GzOFFSET = bias_lp[2];
            }
        }

        // IMU 安装校正
        IMU_Param_Correction(&imu_param, ins.gyro, ins.accel);

        // EKF 更新四元数
        IMU_QuaternionEKF_Update(ins.gyro[INS_X], ins.gyro[INS_Y], ins.gyro[INS_Z],
                                 ins.accel[INS_X], ins.accel[INS_Y], ins.accel[INS_Z], dt);
        memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体坐标系 → 大地坐标系
        BodyFrameToEarthFrame(xb, ins.xn, ins.q);
        BodyFrameToEarthFrame(yb, ins.yn, ins.q);
        BodyFrameToEarthFrame(zb, ins.zn, ins.q);

        // 计算运动加速度（去掉重力）
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
        for (uint8_t i = 0; i < 3; ++i)
        {
            ins.motion_accel_b[i] = (ins.accel[i] - gravity_b[i]) * dt / (ins.accel_lpf + dt)
                                   + ins.motion_accel_b[i] * ins.accel_lpf / (ins.accel_lpf + dt);
        }
        BodyFrameToEarthFrame(ins.motion_accel_b, ins.motion_accel_b, ins.q);

        // 提取姿态角
        ins.yaw = QEKF_INS.Yaw;
        ins.pitch = QEKF_INS.Pitch;
        ins.roll = QEKF_INS.Roll;
        ins.yaw_total_angle = QEKF_INS.YawTotalAngle;

        // 发布消息（轴映射）
        ins_data.yaw = -ins.yaw;
        ins_data.pitch = ins.roll;
        ins_data.roll = ins.pitch;
        ins_data.yaw_total_angle = -ins.yaw_total_angle;
        ins_data.accel[0] = ins.accel[0];
        ins_data.accel[1] = ins.accel[1];
        ins_data.accel[2] = ins.accel[2];
        ins_data.gyro[0] = -ins.gyro[0];
        ins_data.gyro[1] = ins.gyro[1];
        ins_data.gyro[2] = -ins.gyro[2];

        pub_push_msg(ins_pub, &ins_data);

        // 温控关闭
        // if ((count % 2) == 0)
        // {
        //     pulse = (uint32_t)pid_calculate(imu_temp_pid, temp, IMU_TARGET_TEMP);
        //     if (pulse < 0) pulse = 0;
        //     htim3.Instance->CCR4 = (uint16_t)pulse;
        // }
        count++;

        ins_dt = dwt_get_time_ms() - ins_start;
        osDelay(1);
    }
}

/* ----------------------------------------------- 线程间通讯话题相关 ---------------------------------------------------- */
static void ins_pub_init(void)
{
    ins_pub = pub_register("ins_msg", sizeof(struct ins_msg));
}

/* ----------------------------------------------- INS初始化 --------------------------------------------------- */
static void ins_init(void)
{
    imu_param.scale[INS_X] = 1;
    imu_param.scale[INS_Y] = 1;
    imu_param.scale[INS_Z] = 1;
    imu_param.yaw = 0;
    imu_param.pitch = 0;
    imu_param.roll = 0;
    imu_param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    ins.accel_lpf = 0.0085;
}

/**
 * @brief 使用加速度计数据初始化四元数
 */
static void InitQuaternion(float *init_q4)
{
    float temp_init = 0;
    float acc_single[3];
    float gyro_single[3];
    float gravity_norm[3] = {0, 0, 1};
    float axis_rot[3] = {0};

    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_read(gyro_single, acc_single, &temp_init);
        acc_init[0] += acc_single[0];
        acc_init[1] += acc_single[1];
        acc_init[2] += acc_single[2];
        gyro_init[0] += gyro_single[0];
        gyro_init[1] += gyro_single[1];
        gyro_init[2] += gyro_single[2];
        osDelay(1);
    }
    for (uint8_t i = 0; i < 3; ++i)
    {
        acc_init[i] /= 100;
        gyro_init[i] /= 100;
    }
    Norm3d(acc_init);

    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f);
}

/**
 * @brief 机体坐标系 → 大地坐标系
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2]*q[2] - q[3]*q[3]) * vecBF[0] +
                        (q[1]*q[2] - q[0]*q[3]) * vecBF[1] +
                        (q[1]*q[3] + q[0]*q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1]*q[2] + q[0]*q[3]) * vecBF[0] +
                        (0.5f - q[1]*q[1] - q[3]*q[3]) * vecBF[1] +
                        (q[2]*q[3] - q[0]*q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1]*q[3] - q[0]*q[2]) * vecBF[0] +
                        (q[2]*q[3] + q[0]*q[1]) * vecBF[1] +
                        (0.5f - q[1]*q[1] - q[2]*q[2]) * vecBF[2]);
}

/**
 * @brief 大地坐标系 → 机体坐标系
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2]*q[2] - q[3]*q[3]) * vecEF[0] +
                        (q[1]*q[2] + q[0]*q[3]) * vecEF[1] +
                        (q[1]*q[3] - q[0]*q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1]*q[2] - q[0]*q[3]) * vecEF[0] +
                        (0.5f - q[1]*q[1] - q[3]*q[3]) * vecEF[1] +
                        (q[2]*q[3] + q[0]*q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1]*q[3] + q[0]*q[2]) * vecEF[0] +
                        (q[2]*q[3] - q[0]*q[1]) * vecEF[1] +
                        (0.5f - q[1]*q[1] - q[2]*q[2]) * vecEF[2]);
}

/**
 * @brief IMU 安装偏差校正
 */
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->yaw - lastYawOffset) > 0.001f ||
        fabsf(param->pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->roll / 57.295779513f);

        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }

    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[INS_X] = c_11 * gyro_temp[INS_X] + c_12 * gyro_temp[INS_Y] + c_13 * gyro_temp[INS_Z];
    gyro[INS_Y] = c_21 * gyro_temp[INS_X] + c_22 * gyro_temp[INS_Y] + c_23 * gyro_temp[INS_Z];
    gyro[INS_Z] = c_31 * gyro_temp[INS_X] + c_32 * gyro_temp[INS_Y] + c_33 * gyro_temp[INS_Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[INS_X] = c_11 * accel_temp[INS_X] + c_12 * accel_temp[INS_Y] + c_13 * accel_temp[INS_Z];
    accel[INS_Y] = c_21 * accel_temp[INS_X] + c_22 * accel_temp[INS_Y] + c_23 * accel_temp[INS_Z];
    accel[INS_Z] = c_31 * accel_temp[INS_X] + c_32 * accel_temp[INS_Y] + c_33 * accel_temp[INS_Z];

    lastYawOffset = param->yaw;
    lastPitchOffset = param->pitch;
    lastRollOffset = param->roll;
}
