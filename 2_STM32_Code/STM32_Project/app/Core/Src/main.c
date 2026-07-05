/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "OLED.h"
#include "board.h"
#include "servo.h"
#include "motor.h"
#include "uart2_oled.h"
#include "icm42688_imu.h"
#include "fusion_tracker.h"
#include "ina219_sensor.h"
#include "w25q64.h"
#include "boot_shared.h"
#include "my_rtc.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

void Board_SystemClock_Config(void);

/* Private defines -----------------------------------------------------------*/
#define WHEEL_DIAMETER_MM            68.4f
#define SYSTEM_READY_BEEP_MS         120U
#define FUSION_UPDATE_PERIOD_MS      5U
#define FUSION_SAMPLE_RATE_HZ        (1000U / FUSION_UPDATE_PERIOD_MS)
#define IMU_GYRO_BIAS_SAMPLES        1200U  /* 6 s calibration: chip has more time to thermally settle */
#define BATTERY_SEED_SAMPLE_COUNT    20U
#define BATTERY_SEED_SAMPLE_DELAY_MS 100U
#define TELEMETRY_UPDATE_PERIOD_MS   20U
#define COMMAND_TIMEOUT_MS           500U
#define MAX_FORWARD_SPEED_M_S        0.60f
#define MAX_REVERSE_SPEED_M_S        0.30f
#define MAX_DUTY_PERCENT             55U
#define MIN_EFFECTIVE_DUTY_PERCENT   5U
#define LINEAR_CMD_DEADBAND_M_S      0.01f
#define SERVO_MAX_DELTA_DEG          50.0f
#define SERVO_YAW_GAIN_DEG_PER_RAD   18.0f
#define STRAIGHT_HOLD_MIN_SPEED_M_S  0.08f
#define STRAIGHT_HOLD_WZ_DEADBAND    0.01f
#define STRAIGHT_HOLD_SERVO_SIGN     (-1.0f)
#define STRAIGHT_HOLD_KP_DEG_PER_RAD 100.0f
#define STRAIGHT_HOLD_KI_DEG_PER_RAD_S 20.0f
#define STRAIGHT_HOLD_KD_DEG_PER_RADS 0.0f
#define STRAIGHT_HOLD_INTEGRAL_LIMIT 0.20f
#define CONTROL_LOOP_DT_S            0.001f
#define WHEEL_SYNC_PID_KP            3.0f
#define WHEEL_SYNC_PID_KI            2.0f
#define WHEEL_SYNC_PID_KD            0.0f
#define WHEEL_SYNC_INTEGRAL_LIMIT    0.20f
#define WHEEL_SYNC_CORRECTION_LIMIT  12.0f
#define TEST_SERVO_PID_KP_DEG_PER_RAD 90.0f
#define TEST_SERVO_PID_KI_DEG_PER_RAD_S 0.0f
#define TEST_SERVO_PID_KD_DEG_PER_RADS 0.0f
#define TEST_SERVO_PID_INTEGRAL_LIMIT 0.0f
#define TEST_SERVO_PID_SIGN         (-1.0f)
#define DEG_TO_RAD                   0.0174532925f
#define TURN90_DELTA_RAD             (90.0f * DEG_TO_RAD)
#define TURN180_DELTA_RAD            (180.0f * DEG_TO_RAD)
#define STANDARD_GRAVITY_M_S2        9.80665f
#define IMU_GYRO_DEADBAND_DPS        0.05f
#define IMU_GYRO_YAW_SCALE_FACTOR    1.000f
/* Runtime bias adaptation:
 *  THRESHOLD: only update bias when all 3 axes are within this band.
 *             Tighter = more conservative. 0.3 dps rejects light vibration.
 *  ALPHA:     EMA coefficient. Time constant = 1/ALPHA * 5ms.
 *             0.002 -> ~2.5 s time constant, tracks temperature drift faster. */
#define IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS  0.3f
#define IMU_GYRO_BIAS_ADAPT_ALPHA          0.002f

/* Private function prototypes -----------------------------------------------*/
static float Battery_ReadSeedVoltage(void);
static void Motion_Command_Reset(void);
static void Motion_ProcessLine(const char *line);
static void Motion_ApplyCommand(void);
static void Motion_UpdateOdometry(int32_t left_delta_count, int32_t right_delta_count,
                                  float wheel_circumference_mm, uint32_t now_ms);
static void Motion_SendTelemetry(void);
static int8_t Motion_ComputeSignedDutyPercent(float linear_cmd_m_s);
static float Motion_WrapAngleRad(float angle_rad);
static float Motion_ClampFloat(float value, float min_value, float max_value);
static void Motion_ClearBrakeState(void);
static float Motion_GetCorrectedYawRad(void);
static void Motion_ResetServoImuPidTest(void);
static void Motion_ApplyServoImuPidTest(void);
static void Motion_ClearTurn90Target(void);
static void Motion_StartTurn90(float delta_yaw_rad);

/* Private variables ---------------------------------------------------------*/
static FusionTracker fusion_tracker;
static ICM42688_GyroBias imu_gyro_bias = {0.0f, 0.0f, 0.0f};
static uint8_t icm42688_ready = 0U;
static uint8_t ina219_ready = 0U;
static int32_t motor_left_total_count = 0;
static int32_t motor_right_total_count = 0;
static float fusion_yaw_debug = 0.0f;
static float fusion_yaw_zero_deg = 0.0f;
static uint8_t fusion_yaw_zero_valid = 0U;
static float command_linear_m_s = 0.0f;
static float command_angular_rad_s = 0.0f;
static uint32_t command_last_rx_ms = 0U;
static uint32_t telemetry_last_update_ms = 0U;
static uint32_t odom_last_update_ms = 0U;
static float odom_x_m = 0.0f;
static float odom_y_m = 0.0f;
static float odom_theta_rad = 0.0f;
static float odom_linear_speed_m_s = 0.0f;
static float odom_angular_speed_rad_s = 0.0f;
static int32_t imu_last_accel_x_mg = 0;
static int32_t imu_last_accel_y_mg = 0;
static int32_t imu_last_accel_z_mg = 0;
static float imu_last_gyro_x_dps = 0.0f;
static float imu_last_gyro_y_dps = 0.0f;
static float imu_last_gyro_z_dps = 0.0f;
static uint8_t straight_hold_active = 0U;
static float straight_hold_target_yaw_rad = 0.0f;
static float straight_hold_integral = 0.0f;
static float straight_hold_prev_error = 0.0f;
static uint8_t command_timeout_brake_active = 0U;
static uint8_t command_rx_active = 0U;
static uint8_t command_brake_active = 0U;
static uint8_t servo_imu_pid_test_active = 0U;
static float servo_imu_pid_target_yaw_rad = 0.0f;
static uint8_t servo_imu_pid_target_initialized = 0U;
static float servo_imu_pid_integral = 0.0f;
static float servo_imu_pid_prev_error = 0.0f;
static float wheel_left_speed_m_s = 0.0f;
static float wheel_right_speed_m_s = 0.0f;
static float wheel_sync_integral = 0.0f;
static float wheel_sync_prev_error = 0.0f;
static uint8_t heading_is_locked = 0U;
static uint8_t turn90_target_valid = 0U;
static float turn90_target_yaw_rad = 0.0f;
static int8_t turn_direction = 0;
static float battery_voltage_v = 0.0f;
static float battery_current_a = 0.0f;
static float battery_soc_percent = 0.0f;

volatile uint8_t system_power_mode = 0; // 0: RUN, 1: SLEEP_WAIT
extern volatile uint8_t rtc_alarm_triggered;

typedef void (*pFunction)(void);

static void JumpToBootloader(void)
{
  uint32_t bootloader_address = 0x08000000;
  
  __disable_irq();
  
  /* Switch system clock back to HSI */
  RCC->CR |= RCC_CR_HSION;
  while ((RCC->CR & RCC_CR_HSIRDY) == 0) {}

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}

  /* Disable PLL and HSE */
  RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEON);
  
  /* Reset peripherals except GPIOC */
  RCC->AHB1RSTR = 0xFFFFFFFFU;
  RCC->AHB1RSTR = 0x00000000U;
  
  RCC->AHB2RSTR = (0xFFFFFFFFU & ~(RCC_AHB2RSTR_GPIOCRST));
  RCC->AHB2RSTR = 0x00000000U;

  RCC->AHB3RSTR = 0xFFFFFFFFU;
  RCC->AHB3RSTR = 0x00000000U;

  RCC->APB1RSTR1 = 0xFFFFFFFFU;
  RCC->APB1RSTR1 = 0x00000000U;

  RCC->APB1RSTR2 = 0xFFFFFFFFU;
  RCC->APB1RSTR2 = 0x00000000U;

  RCC->APB2RSTR = 0xFFFFFFFFU;
  RCC->APB2RSTR = 0x00000000U;
  
  /* De-init SysTick */
  SysTick->CTRL = 0;
  SysTick->LOAD = 0;
  SysTick->VAL  = 0;
  
  /* Clear all pending interrupts */
  for (int i = 0; i < 8; i++) {
    NVIC->ICER[i] = 0xFFFFFFFFU;
    NVIC->ICPR[i] = 0xFFFFFFFFU;
  }
  
  /* Reset Vector Table Offset Register to Bootloader's vector table */
  SCB->VTOR = 0x08000000U;
  
  /* Enable global interrupts since Bootloader expects PRIMASK=0 */
  __enable_irq();
  
  /* Set Main Stack Pointer */
  __set_MSP(*(__IO uint32_t*)bootloader_address);
  
  /* Jump to Bootloader */
  uint32_t jump_address = *(__IO uint32_t*)(bootloader_address + 4);
  pFunction JumpToApp = (pFunction)jump_address;
  JumpToApp();
  
  while(1);
}

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  uint32_t last_oled_update_ms = 0U;

  HAL_Init();
  Board_Init();
  
  W25Q64_Init();
  // OTA / Bootloader Insurance Logic
  BootInfo_t boot_info;
  W25Q64_ReadData(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
  if (boot_info.magic == BOOT_MAGIC_WORD) {
      if (boot_info.boot_failure_count > 0) {
          boot_info.boot_failure_count = 0;
          W25Q64_SectorErase(BOOT_INFO_ADDRESS);
          W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
      }
  }

  UART2_OLED_Init();
  Servo_Init();
  Motor_Init();
  
  /* W25Q64 Flash Test (Disabled) */
  // W25Q64_Init();
  // uint8_t MID;
  // uint16_t DID;
  // W25Q64_ReadID(&MID, &DID);
  // UART2_OLED_Printf("====================================\r\n");
  // UART2_OLED_Printf("W25Q64 ID: MID=%02X, DID=%04X\r\n", MID, DID);
  // UART2_OLED_Printf("====================================\r\n");
  
  Motion_Command_Reset();

  /* Hardware Test: Force motors to 20% duty cycle on power up (Disabled) */
  // Motor_SetLeftDutyPercent(20);
  // Motor_SetRightDutyPercent(20);
  MyRTC_Init();

  // Hardware Self-Test
  uint8_t hardware_ok = 1;
  
  uint8_t MID;
  uint16_t DID;
  W25Q64_ReadID(&MID, &DID);
  if (MID != 0xEF) {
      hardware_ok = 0;
  }

  UART2_OLED_ClearText();
  ina219_ready = (INA219_Sensor_Init() == HAL_OK);
  if (ina219_ready == 0U) {
      hardware_ok = 0;
  } else {
      battery_voltage_v = Battery_ReadSeedVoltage();
      (void)INA219_BatterySeedSocFromVoltage4S(battery_voltage_v);
      battery_soc_percent = INA219_BatteryGetSocPercent();
  }
  Board_LoadSwitch_Write(GPIO_PIN_SET);

  icm42688_ready = (ICM42688_IMU_Init(0) == HAL_OK);
  if (icm42688_ready == 0U) {
      hardware_ok = 0;
  }

  if (hardware_ok) {
      Board_Buzzer_Beep(SYSTEM_READY_BEEP_MS);
  } else {
      Board_Buzzer_Beep(50);
      HAL_Delay(50);
      Board_Buzzer_Beep(50);
      HAL_Delay(50);
      Board_Buzzer_Beep(50);
  }

  if (icm42688_ready)
  {
    UART2_OLED_Printf("IMU CAL..... KEEP STILL\r\n");
    icm42688_ready = (ICM42688_IMU_CalibrateGyroBias(IMU_GYRO_BIAS_SAMPLES, &imu_gyro_bias) == HAL_OK);
    if (icm42688_ready) {
        UART2_OLED_Printf("IMU CAL DONE\r\n");
        Board_Buzzer_Beep(SYSTEM_READY_BEEP_MS);
    } else {
        UART2_OLED_Printf("IMU CAL FAIL\r\n");
    }
  }

  FusionTracker_Init(&fusion_tracker, FUSION_SAMPLE_RATE_HZ);

  while (1)
  {
    if (system_power_mode == 1)
    {
      UART2_OLED_Printf("Shutting down RK3588 in 15s...\r\n");
      // 1. 延迟15秒，给 RK3588 留出执行 Linux shutdown -h now 的时间，防止文件系统损坏
      uint32_t shutdown_wait = HAL_GetTick();
      while ((HAL_GetTick() - shutdown_wait) < 15000)
      {
        HAL_Delay(100);
      }
      
      // 2. 切断 RK3588 物理电源
      Board_LoadSwitch_Write(GPIO_PIN_RESET);
      UART2_OLED_Printf("Power Cut. Entering STOP mode.\r\n");
      HAL_Delay(100); // 留出一点时间让串口打印完毕
      
      // 3. 准备进入 STOP 模式
      HAL_SuspendTick(); // 挂起 SysTick 避免被其周期中断唤醒
      rtc_alarm_triggered = 0;
      
      // 进入低功耗 STOP 模式（PLL 停止），等待 RTC EXTI17 唤醒
      HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
      
      // 4. --- 系统从 STOP 模式唤醒 ---
      HAL_ResumeTick(); // 恢复 SysTick
      
      // STOP 唤醒后，硬件会自动回退到 HSI 时钟，必须立刻重新配置 PLL 恢复 170MHz
      Board_SystemClock_Config();
      
      // 检查是否是 RTC 闹钟引发的真实唤醒
      if (rtc_alarm_triggered != 0U)
      {
        rtc_alarm_triggered = 0;
        system_power_mode = 0; // 恢复正常运行模式
        
        // 重新拉高 PC13 给 RK3588 供电，启动系统
        Board_LoadSwitch_Write(GPIO_PIN_SET);
        Board_Buzzer_Beep(200);
        HAL_Delay(100);
        Board_Buzzer_Beep(200);
      }
      continue; // 跳过后续的传感器读取，直接进入下一次循环
    }

    uint32_t now_ms;
    uint32_t fusion_delta_ms;
    float wheel_circumference_mm;
    char uart_line_buffer[96];
    int32_t accel_x_mg;
    int32_t accel_y_mg;
    int32_t accel_z_mg;
    int32_t motor_left_delta_count = 0;
    int32_t motor_right_delta_count = 0;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    static uint32_t servo_last_update_ms = 0U;
    uint32_t servo_delta_ms;

    now_ms = HAL_GetTick();
    fusion_delta_ms = now_ms - fusion_tracker.last_update_ms;
    servo_delta_ms = now_ms - servo_last_update_ms;

    if (servo_delta_ms >= 5U) {
      Servo_Update(servo_delta_ms);
      servo_last_update_ms = now_ms;
    }

    UART2_OLED_ProcessRx();
    while (UART2_OLED_ReadLine(uart_line_buffer, sizeof(uart_line_buffer)) != 0U)
    {
      if (strncmp(uart_line_buffer, "update", 6) == 0) {
        BootInfo_t boot_info;
        W25Q64_ReadData(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        boot_info.iap_request = 1;
        W25Q64_SectorErase(BOOT_INFO_ADDRESS);
        W25Q64_PageProgram(BOOT_INFO_ADDRESS, (uint8_t*)&boot_info, sizeof(BootInfo_t));
        UART2_OLED_Printf("Jumping to OTA Bootloader...\r\n");
        HAL_Delay(100);
        JumpToBootloader();
      }
      Motion_ProcessLine(uart_line_buffer);
    }

    wheel_circumference_mm = 3.1415926f * WHEEL_DIAMETER_MM;

    if (icm42688_ready && (fusion_delta_ms >= FUSION_UPDATE_PERIOD_MS))
    {
      fusion_tracker.last_update_ms = now_ms;
      if ((ICM42688_IMU_ReadAccelMg(&accel_x_mg, &accel_y_mg, &accel_z_mg) == HAL_OK)
          && (ICM42688_IMU_ReadGyroDps(&gyro_x_dps, &gyro_y_dps, &gyro_z_dps) == HAL_OK))
      {
        float raw_gyro_x = gyro_x_dps;
        float raw_gyro_y = gyro_y_dps;
        float raw_gyro_z = gyro_z_dps;

        if ((command_linear_m_s == 0.0f) && (command_angular_rad_s == 0.0f)
            && (wheel_left_speed_m_s == 0.0f) && (wheel_right_speed_m_s == 0.0f)
            && (raw_gyro_x > imu_gyro_bias.x_dps - IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS)
            && (raw_gyro_x < imu_gyro_bias.x_dps + IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS)
            && (raw_gyro_y > imu_gyro_bias.y_dps - IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS)
            && (raw_gyro_y < imu_gyro_bias.y_dps + IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS)
            && (raw_gyro_z > imu_gyro_bias.z_dps - IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS)
            && (raw_gyro_z < imu_gyro_bias.z_dps + IMU_GYRO_BIAS_ADAPT_THRESHOLD_DPS))
        {
          imu_gyro_bias.x_dps = ((1.0f - IMU_GYRO_BIAS_ADAPT_ALPHA) * imu_gyro_bias.x_dps) + (IMU_GYRO_BIAS_ADAPT_ALPHA * raw_gyro_x);
          imu_gyro_bias.y_dps = ((1.0f - IMU_GYRO_BIAS_ADAPT_ALPHA) * imu_gyro_bias.y_dps) + (IMU_GYRO_BIAS_ADAPT_ALPHA * raw_gyro_y);
          imu_gyro_bias.z_dps = ((1.0f - IMU_GYRO_BIAS_ADAPT_ALPHA) * imu_gyro_bias.z_dps) + (IMU_GYRO_BIAS_ADAPT_ALPHA * raw_gyro_z);
        }

        gyro_x_dps -= imu_gyro_bias.x_dps;
        gyro_y_dps -= imu_gyro_bias.y_dps;
        gyro_z_dps -= imu_gyro_bias.z_dps;

        if (gyro_x_dps > -IMU_GYRO_DEADBAND_DPS && gyro_x_dps < IMU_GYRO_DEADBAND_DPS) gyro_x_dps = 0.0f;
        if (gyro_y_dps > -IMU_GYRO_DEADBAND_DPS && gyro_y_dps < IMU_GYRO_DEADBAND_DPS) gyro_y_dps = 0.0f;
        if (gyro_z_dps > -IMU_GYRO_DEADBAND_DPS && gyro_z_dps < IMU_GYRO_DEADBAND_DPS) gyro_z_dps = 0.0f;
        gyro_z_dps *= IMU_GYRO_YAW_SCALE_FACTOR;

        imu_last_accel_x_mg = accel_x_mg;
        imu_last_accel_y_mg = accel_y_mg;
        imu_last_accel_z_mg = accel_z_mg;
        imu_last_gyro_x_dps = gyro_x_dps;
        imu_last_gyro_y_dps = gyro_y_dps;
        imu_last_gyro_z_dps = gyro_z_dps;

        int32_t fusion_ax = accel_x_mg;
        int32_t fusion_ay = accel_y_mg;
        int32_t fusion_az = accel_z_mg;
        if ((gyro_z_dps > 5.0f) || (gyro_z_dps < -5.0f) ||
            (wheel_left_speed_m_s != 0.0f) || (wheel_right_speed_m_s != 0.0f) ||
            (command_linear_m_s != 0.0f))
        {
          fusion_ax = 0;
          fusion_ay = 0;
          fusion_az = 1000;
        }

        FusionTracker_Update(&fusion_tracker, gyro_x_dps, gyro_y_dps, gyro_z_dps,
                             fusion_ax, fusion_ay, fusion_az,
                             (float)fusion_delta_ms / 1000.0f);
        if (fusion_yaw_zero_valid == 0U)
        {
          fusion_yaw_zero_deg = fusion_tracker.euler.angle.yaw;
          fusion_yaw_zero_valid = 1U;
        }
        if ((servo_imu_pid_test_active != 0U) && (servo_imu_pid_target_initialized == 0U))
        {
          servo_imu_pid_target_yaw_rad = Motion_GetCorrectedYawRad();
          servo_imu_pid_target_initialized = 1U;
        }

        fusion_yaw_debug += gyro_z_dps * ((float)fusion_delta_ms / 1000.0f);
        if (fusion_yaw_debug > 180.0f)
        {
          fusion_yaw_debug -= 360.0f;
        }
        else if (fusion_yaw_debug < -180.0f)
        {
          fusion_yaw_debug += 360.0f;
        }
      }
    }

    if (Motor_TakeControl1msFlag() != 0U)
    {
      motor_left_delta_count = Motor_GetLeftDeltaCount();
      motor_right_delta_count = Motor_GetRightDeltaCount();
      motor_left_total_count += motor_left_delta_count;
      motor_right_total_count += motor_right_delta_count;
      Motion_UpdateOdometry(motor_left_delta_count, motor_right_delta_count,
                            wheel_circumference_mm, now_ms);
      Motion_ApplyCommand(); // Restored to allow host movement control
    }

    if ((now_ms - telemetry_last_update_ms) >= TELEMETRY_UPDATE_PERIOD_MS)
    {
      Motion_SendTelemetry();
      telemetry_last_update_ms = now_ms;
    }

    if ((now_ms - last_oled_update_ms) < 100U)
    {
      HAL_Delay(1);
      continue;
    }

    last_oled_update_ms = now_ms;
    if (ina219_ready != 0U)
    {
      battery_voltage_v = INA219_Sensor_ReadBusVoltage_V();
      battery_current_a = INA219_Sensor_ReadCurrent_A();
      (void)INA219_BatteryUpdateSoc4S(battery_voltage_v, battery_current_a);
      battery_soc_percent = INA219_BatteryGetSocPercent();
    }
    UART2_OLED_RenderDashboard(ina219_ready,
                               battery_voltage_v, battery_current_a, battery_soc_percent,
                               imu_last_accel_x_mg, imu_last_accel_y_mg, imu_last_accel_z_mg,
                               imu_last_gyro_x_dps, imu_last_gyro_y_dps, imu_last_gyro_z_dps);
  }
}

static float Battery_ReadSeedVoltage(void)
{
  uint32_t i;
  float voltage_sum = 0.0f;

  for (i = 0U; i < BATTERY_SEED_SAMPLE_COUNT; i++)
  {
    voltage_sum += INA219_Sensor_ReadBusVoltage_V();
    HAL_Delay(BATTERY_SEED_SAMPLE_DELAY_MS);
  }

  return voltage_sum / (float)BATTERY_SEED_SAMPLE_COUNT;
}

static void Motion_Command_Reset(void)
{
  command_linear_m_s = 0.0f;
  command_angular_rad_s = 0.0f;
  command_last_rx_ms = 0U;
  telemetry_last_update_ms = command_last_rx_ms;
  odom_last_update_ms = command_last_rx_ms;
  odom_x_m = 0.0f;
  odom_y_m = 0.0f;
  odom_theta_rad = 0.0f;
  odom_linear_speed_m_s = 0.0f;
  odom_angular_speed_rad_s = 0.0f;
  straight_hold_active = 0U;
  straight_hold_target_yaw_rad = 0.0f;
  straight_hold_integral = 0.0f;
  straight_hold_prev_error = 0.0f;
  command_timeout_brake_active = 0U;
  command_rx_active = 0U;
  command_brake_active = 0U;
  servo_imu_pid_test_active = 0U;
  servo_imu_pid_target_yaw_rad = 0.0f;
  servo_imu_pid_target_initialized = 0U;
  servo_imu_pid_integral = 0.0f;
  servo_imu_pid_prev_error = 0.0f;
  wheel_left_speed_m_s = 0.0f;
  wheel_right_speed_m_s = 0.0f;
  wheel_sync_integral = 0.0f;
  wheel_sync_prev_error = 0.0f;
  heading_is_locked = 0U;
  turn90_target_valid = 0U;
  turn90_target_yaw_rad = 0.0f;
  turn_direction = 0;
  Motor_SetLeftDutyPercent(0);
  Motor_SetRightDutyPercent(0);
  Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
}

static void Motion_ProcessLine(const char *line)
{
  const char *payload;
  const char *angular_payload;
  char *end_ptr;
  float linear_cmd;
  float angular_cmd;

  if (line == NULL)
  {
    return;
  }

  payload = line;
  if (*payload != '@')
  {
    return;
  }
  payload++;

  if (strncmp(payload, "RTC,", 4) == 0)
  {
    int h = 0, m = 0, s = 0;
    if (sscanf(payload + 4, "%d,%d,%d", &h, &m, &s) == 3)
    {
      MyRTC_SetTime((uint8_t)h, (uint8_t)m, (uint8_t)s);
      UART2_OLED_Printf("RTC Set: %02d:%02d:%02d\r\n", h, m, s);
    }
    return;
  }

  if (strncmp(payload, "ALARM,", 6) == 0)
  {
    int h = 0, m = 0, s = 0;
    if (sscanf(payload + 6, "%d,%d,%d", &h, &m, &s) == 3)
    {
      MyRTC_SetAlarmA((uint8_t)h, (uint8_t)m, (uint8_t)s);
      UART2_OLED_Printf("ALARM Set: %02d:%02d:%02d\r\n", h, m, s);
    }
    return;
  }

  if (strcmp(payload, "FINISH") == 0)
  {
    system_power_mode = 1; // 收到结束信号，准备切断电源并休眠
    return;
  }

  if ((payload[0] == 'B') && (payload[1] == 'R') && (payload[2] == 'A')
      && (payload[3] == 'K') && (payload[4] == 'E') && (payload[5] == '\0'))
  {
    Motion_ClearTurn90Target();
    Motion_ResetServoImuPidTest();
    command_linear_m_s = 0.0f;
    command_angular_rad_s = 0.0f;
    straight_hold_active = 0U;
    straight_hold_integral = 0.0f;
    straight_hold_prev_error = 0.0f;
    wheel_sync_integral = 0.0f;
    wheel_sync_prev_error = 0.0f;
    heading_is_locked = 0U;
    command_last_rx_ms = HAL_GetTick();
    command_rx_active = 1U;
    command_timeout_brake_active = 0U;
    command_brake_active = 1U;
    return;
  }

/*
  if ((payload[0] == 'S') && (payload[1] == 'E') && (payload[2] == 'R') && (payload[3] == 'V') && (payload[4] == 'O') && (payload[5] == ','))
  {
    const char *servo_arg = payload + 6;
    float angle = strtof(servo_arg, &end_ptr);
    if ((end_ptr != servo_arg) && (*end_ptr == '\0'))
    {
      Servo_SetAngleDeg(angle);
      UART2_OLED_Printf("SERVO ANGLE SET: %.1f\r\n", (double)angle);
    }
    return;
  }
*/

  if ((payload[0] == 'C') && (payload[1] == 'M') && (payload[2] == 'D') && (payload[3] == ','))
  {
    payload += 4;
    if ((payload[0] == 'B') && (payload[1] == 'R') && (payload[2] == 'A')
        && (payload[3] == 'K') && (payload[4] == 'E') && (payload[5] == '\0'))
    {
      Motion_ClearTurn90Target();
      Motion_ResetServoImuPidTest();
      command_linear_m_s = 0.0f;
      command_angular_rad_s = 0.0f;
      straight_hold_active = 0U;
      Motion_ClearTurn90Target();
      wheel_sync_integral = 0.0f;
      wheel_sync_prev_error = 0.0f;
      heading_is_locked = 0U;
      command_last_rx_ms = HAL_GetTick();
      command_rx_active = 1U;
      command_timeout_brake_active = 0U;
      command_brake_active = 1U;
      return;
    }
  }

  if ((payload[0] == 'P') && (payload[1] == 'I') && (payload[2] == 'D') && (payload[3] == 'T')
      && (payload[4] == 'E') && (payload[5] == 'S') && (payload[6] == 'T') && (payload[7] == ','))
  {
    const char *test_arg = payload + 8;

    if ((test_arg[0] == 'O') && (test_arg[1] == 'F') && (test_arg[2] == 'F') && (test_arg[3] == '\0'))
    {
      Motion_ClearTurn90Target();
      Motion_ResetServoImuPidTest();
      Motor_SetLeftDutyPercent(0);
      Motor_SetRightDutyPercent(0);
      Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
      return;
    }

    if ((test_arg[0] == 'O') && (test_arg[1] == 'N') && (test_arg[2] == '\0'))
    {
      Motion_ClearTurn90Target();
      Motion_ResetServoImuPidTest();
      servo_imu_pid_test_active = 1U;
      servo_imu_pid_target_yaw_rad = Motion_GetCorrectedYawRad();
      servo_imu_pid_target_initialized = (fusion_yaw_zero_valid != 0U) ? 1U : 0U;
      command_brake_active = 0U;
      command_timeout_brake_active = 0U;
      command_rx_active = 1U;
      command_last_rx_ms = HAL_GetTick();
      Motor_SetLeftDutyPercent(0);
      Motor_SetRightDutyPercent(0);
      Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
      return;
    }

    linear_cmd = strtof(test_arg, &end_ptr);
    if ((end_ptr == test_arg) || (*end_ptr != '\0'))
    {
      return;
    }

    Motion_ClearTurn90Target();
    Motion_ResetServoImuPidTest();
    servo_imu_pid_test_active = 1U;
    servo_imu_pid_target_yaw_rad = Motion_WrapAngleRad(linear_cmd * DEG_TO_RAD);
    servo_imu_pid_target_initialized = 1U;
    command_brake_active = 0U;
    command_timeout_brake_active = 0U;
    command_rx_active = 1U;
    command_last_rx_ms = HAL_GetTick();
    Motor_SetLeftDutyPercent(0);
    Motor_SetRightDutyPercent(0);
    Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
    return;
  }

  if (((payload[0] == 'L') && (payload[1] == 'E') && (payload[2] == 'F') && (payload[3] == 'T')
       && (payload[4] == '9') && (payload[5] == '0') && (payload[6] == '\0'))
      || ((payload[0] == 'T') && (payload[1] == 'U') && (payload[2] == 'R') && (payload[3] == 'N')
          && (payload[4] == ',') && (payload[5] == 'L') && (payload[6] == 'E') && (payload[7] == 'F')
          && (payload[8] == 'T') && (payload[9] == '9') && (payload[10] == '0') && (payload[11] == '\0')))
  {
    Motion_StartTurn90(TURN90_DELTA_RAD);
    return;
  }

  if (((payload[0] == 'R') && (payload[1] == 'I') && (payload[2] == 'G') && (payload[3] == 'H')
       && (payload[4] == 'T') && (payload[5] == '9') && (payload[6] == '0') && (payload[7] == '\0'))
      || ((payload[0] == 'T') && (payload[1] == 'U') && (payload[2] == 'R') && (payload[3] == 'N')
          && (payload[4] == ',') && (payload[5] == 'R') && (payload[6] == 'I') && (payload[7] == 'G')
          && (payload[8] == 'H') && (payload[9] == 'T') && (payload[10] == '9') && (payload[11] == '0')
          && (payload[12] == '\0')))
  {
    Motion_StartTurn90(-TURN90_DELTA_RAD);
    return;
  }

  if (((payload[0] == 'L') && (payload[1] == 'E') && (payload[2] == 'F') && (payload[3] == 'T')
       && (payload[4] == '1') && (payload[5] == '8') && (payload[6] == '0') && (payload[7] == '\0')))
  {
    Motion_StartTurn90(TURN180_DELTA_RAD);
    return;
  }

  if (((payload[0] == 'R') && (payload[1] == 'I') && (payload[2] == 'G') && (payload[3] == 'H')
       && (payload[4] == 'T') && (payload[5] == '1') && (payload[6] == '8') && (payload[7] == '0')
       && (payload[8] == '\0')))
  {
    Motion_StartTurn90(-TURN180_DELTA_RAD);
    return;
  }

  linear_cmd = strtof(payload, &end_ptr);
  if ((end_ptr == payload) || (*end_ptr != ','))
  {
    return;
  }

  angular_payload = end_ptr + 1;
  angular_cmd = strtof(angular_payload, &end_ptr);
  if ((end_ptr == angular_payload) || (*end_ptr != '\0'))
  {
    return;
  }

  if (linear_cmd > MAX_FORWARD_SPEED_M_S)
  {
    linear_cmd = MAX_FORWARD_SPEED_M_S;
  }
  else if (linear_cmd < -MAX_REVERSE_SPEED_M_S)
  {
    linear_cmd = -MAX_REVERSE_SPEED_M_S;
  }

  if ((linear_cmd > -LINEAR_CMD_DEADBAND_M_S) && (linear_cmd < LINEAR_CMD_DEADBAND_M_S))
  {
    linear_cmd = 0.0f;
  }

  if (angular_cmd > 2.0f)
  {
    angular_cmd = 2.0f;
  }
  else if (angular_cmd < -2.0f)
  {
    angular_cmd = -2.0f;
  }

  command_linear_m_s = linear_cmd;
  command_angular_rad_s = angular_cmd;
  command_last_rx_ms = HAL_GetTick();
  command_rx_active = 1U;
  Motion_ResetServoImuPidTest();
  Motion_ClearBrakeState();
  if (fusion_yaw_zero_valid != 0U)
  {
    if (linear_cmd == 0.0f && (angular_cmd > -STRAIGHT_HOLD_WZ_DEADBAND && angular_cmd < STRAIGHT_HOLD_WZ_DEADBAND))
    {
      heading_is_locked = 0U;
      straight_hold_active = 0U;
    }
    else if (angular_cmd > -STRAIGHT_HOLD_WZ_DEADBAND && angular_cmd < STRAIGHT_HOLD_WZ_DEADBAND)
    {
      if (heading_is_locked == 0U)
      {
        if (turn90_target_valid != 0U)
        {
          straight_hold_target_yaw_rad = turn90_target_yaw_rad;
          turn90_target_valid = 0U;
        }
        else
        {
          straight_hold_target_yaw_rad = Motion_GetCorrectedYawRad();
        }
        heading_is_locked = 1U;
        
        // 【修复2】只在“刚刚锁定航向”的这一瞬间，才清空 PID 的历史数据
        straight_hold_integral = 0.0f;
        straight_hold_prev_error = 0.0f;
        wheel_sync_integral = 0.0f;
        wheel_sync_prev_error = 0.0f;
      }
      straight_hold_active = 1U;
    }
    else
    {
      straight_hold_active = 0U;
      
    }
  }
  
  // 【修复2】删除原本写在这里的无条件清零这四行变量的代码
  // straight_hold_integral = 0.0f;
  // straight_hold_prev_error = 0.0f;
  // wheel_sync_integral = 0.0f;
  // wheel_sync_prev_error = 0.0f;
}

static void Motion_ApplyCommand(void)
{
  float servo_angle_deg;
  float servo_correction_deg;
  float left_duty_percent;
  float right_duty_percent;
  float current_yaw_rad;
  float yaw_error_rad;
  float wheel_speed_error_m_s;
  float wheel_speed_error_rate;
  float wheel_sync_correction;
  int8_t duty_percent;
  int8_t direction_sign;
  uint32_t now_ms;

  now_ms = HAL_GetTick();
  if (servo_imu_pid_test_active != 0U)
  {
    command_brake_active = 0U;
    command_timeout_brake_active = 0U;
    command_rx_active = 0U;
    Motion_ApplyServoImuPidTest();
    return;
  }

  if (command_brake_active != 0U)
  {
    command_linear_m_s = 0.0f;
    command_angular_rad_s = 0.0f;
    straight_hold_active = 0U;
    straight_hold_integral = 0.0f;
    straight_hold_prev_error = 0.0f;
    wheel_sync_integral = 0.0f;
    wheel_sync_prev_error = 0.0f;
    if (command_timeout_brake_active == 0U)
    {
      Motor_BrakeAll();
      command_timeout_brake_active = 1U;
    }
    Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
    return;
  }

  if ((command_rx_active == 0U) || ((now_ms - command_last_rx_ms) > COMMAND_TIMEOUT_MS))
  {
    command_linear_m_s = 0.0f;
    command_angular_rad_s = 0.0f;
    straight_hold_active = 0U;
    wheel_sync_integral = 0.0f;
    wheel_sync_prev_error = 0.0f;
    heading_is_locked = 0U;
    command_rx_active = 0U;
    if (command_timeout_brake_active == 0U)
    {
      Motor_SetLeftDutyPercent(0);
      Motor_SetRightDutyPercent(0);
      command_timeout_brake_active = 1U;
    }
    Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
    return;
  }

  duty_percent = Motion_ComputeSignedDutyPercent(command_linear_m_s);
  left_duty_percent = (float)duty_percent;
  right_duty_percent = (float)duty_percent;
  servo_angle_deg = SERVO_CENTER_ANGLE_DEG
                  + (command_angular_rad_s * SERVO_YAW_GAIN_DEG_PER_RAD);

  if ((duty_percent != 0)
      && (fusion_yaw_zero_valid != 0U)
      && (straight_hold_active != 0U)
      && (command_linear_m_s >= STRAIGHT_HOLD_MIN_SPEED_M_S
          || command_linear_m_s <= -STRAIGHT_HOLD_MIN_SPEED_M_S)
      && (command_angular_rad_s > -STRAIGHT_HOLD_WZ_DEADBAND)
      && (command_angular_rad_s < STRAIGHT_HOLD_WZ_DEADBAND))
  {
    float yaw_error_rate;

    current_yaw_rad = Motion_GetCorrectedYawRad();
    yaw_error_rad = Motion_WrapAngleRad(straight_hold_target_yaw_rad - current_yaw_rad);
    
    if ((turn_direction == 1) && (yaw_error_rad < -1.57f))
    {
      yaw_error_rad += 6.2831853f;
    }
    else if ((turn_direction == -1) && (yaw_error_rad > 1.57f))
    {
      yaw_error_rad -= 6.2831853f;
    }
    
    straight_hold_integral += yaw_error_rad * CONTROL_LOOP_DT_S;
    straight_hold_integral = Motion_ClampFloat(
        straight_hold_integral,
        -STRAIGHT_HOLD_INTEGRAL_LIMIT,
        STRAIGHT_HOLD_INTEGRAL_LIMIT);
    yaw_error_rate = (yaw_error_rad - straight_hold_prev_error) / CONTROL_LOOP_DT_S;
    straight_hold_prev_error = yaw_error_rad;
    servo_correction_deg = STRAIGHT_HOLD_SERVO_SIGN
                         * ((STRAIGHT_HOLD_KP_DEG_PER_RAD * yaw_error_rad)
                         + (STRAIGHT_HOLD_KI_DEG_PER_RAD_S * straight_hold_integral)
                         + (STRAIGHT_HOLD_KD_DEG_PER_RADS * yaw_error_rate));
                         
    if (command_linear_m_s < 0.0f)
    {
      servo_correction_deg = -servo_correction_deg;
    }
    
    servo_angle_deg += servo_correction_deg;

    direction_sign = (duty_percent > 0) ? 1 : -1;
    wheel_speed_error_m_s = (wheel_left_speed_m_s * (float)direction_sign)
                          - (wheel_right_speed_m_s * (float)direction_sign);
    wheel_sync_integral += wheel_speed_error_m_s * CONTROL_LOOP_DT_S;
    wheel_sync_integral = Motion_ClampFloat(
        wheel_sync_integral,
        -WHEEL_SYNC_INTEGRAL_LIMIT,
        WHEEL_SYNC_INTEGRAL_LIMIT);
    wheel_speed_error_rate = (wheel_speed_error_m_s - wheel_sync_prev_error) / CONTROL_LOOP_DT_S;
    wheel_sync_prev_error = wheel_speed_error_m_s;
    wheel_sync_correction = (WHEEL_SYNC_PID_KP * wheel_speed_error_m_s)
                          + (WHEEL_SYNC_PID_KI * wheel_sync_integral)
                          + (WHEEL_SYNC_PID_KD * wheel_speed_error_rate);
    wheel_sync_correction = Motion_ClampFloat(
        wheel_sync_correction,
        -WHEEL_SYNC_CORRECTION_LIMIT,
        WHEEL_SYNC_CORRECTION_LIMIT);
    left_duty_percent -= ((float)direction_sign * wheel_sync_correction);
    right_duty_percent += ((float)direction_sign * wheel_sync_correction);
  }
  else
  {
    straight_hold_active = 0U;
    straight_hold_integral = 0.0f;
    straight_hold_prev_error = 0.0f;
    wheel_sync_integral = 0.0f;
    wheel_sync_prev_error = 0.0f;
  }

  if (servo_angle_deg > (SERVO_CENTER_ANGLE_DEG + SERVO_MAX_DELTA_DEG))
  {
    servo_angle_deg = SERVO_CENTER_ANGLE_DEG + SERVO_MAX_DELTA_DEG;
  }
  else if (servo_angle_deg < (SERVO_CENTER_ANGLE_DEG - SERVO_MAX_DELTA_DEG))
  {
    servo_angle_deg = SERVO_CENTER_ANGLE_DEG - SERVO_MAX_DELTA_DEG;
  }

  left_duty_percent = Motion_ClampFloat(left_duty_percent, -(float)MAX_DUTY_PERCENT, (float)MAX_DUTY_PERCENT);
  right_duty_percent = Motion_ClampFloat(right_duty_percent, -(float)MAX_DUTY_PERCENT, (float)MAX_DUTY_PERCENT);

  Motor_SetLeftDutyPercent((int8_t)left_duty_percent);
  Motor_SetRightDutyPercent((int8_t)right_duty_percent);
  Servo_SetAngleDeg(servo_angle_deg);
}

static void Motion_ClearBrakeState(void)
{
  if (command_timeout_brake_active != 0U)
  {
    Motor_ReleaseAll();
  }
  command_timeout_brake_active = 0U;
  command_brake_active = 0U;
}

static void Motion_ResetServoImuPidTest(void)
{
  servo_imu_pid_test_active = 0U;
  servo_imu_pid_target_yaw_rad = 0.0f;
  servo_imu_pid_target_initialized = 0U;
  servo_imu_pid_integral = 0.0f;
  servo_imu_pid_prev_error = 0.0f;
}

static void Motion_ClearTurn90Target(void)
{
  turn90_target_valid = 0U;
  turn90_target_yaw_rad = 0.0f;
  turn_direction = 0;
}

static void Motion_StartTurn90(float delta_yaw_rad)
{
  float target_yaw_rad;
  turn_direction = (delta_yaw_rad > 0.0f) ? 1 : -1;

  if (fusion_yaw_zero_valid == 0U)
  {
    Motion_ClearTurn90Target();
    return;
  }

  Motion_ClearBrakeState();
  command_brake_active = 0U;
  command_timeout_brake_active = 0U;
  target_yaw_rad = Motion_WrapAngleRad(Motion_GetCorrectedYawRad() + delta_yaw_rad);
  turn90_target_valid = 1U;
  turn90_target_yaw_rad = target_yaw_rad;
  heading_is_locked = 0U;
  straight_hold_active = 0U;
  command_last_rx_ms = HAL_GetTick();
  command_rx_active = 1U;
}

static void Motion_ApplyServoImuPidTest(void)
{
  if ((fusion_yaw_zero_valid == 0U) || (servo_imu_pid_target_initialized == 0U))
  {
    Motor_SetLeftDutyPercent(0);
    Motor_SetRightDutyPercent(0);
    Servo_SetAngleDeg(SERVO_CENTER_ANGLE_DEG);
    return;
  }

  float current_yaw_rad = Motion_GetCorrectedYawRad();
  float yaw_error_rad = Motion_WrapAngleRad(servo_imu_pid_target_yaw_rad - current_yaw_rad);
  float servo_correction_deg;
  float servo_angle_deg;

  servo_imu_pid_integral = 0.0f;
  servo_imu_pid_prev_error = yaw_error_rad;

  servo_correction_deg = TEST_SERVO_PID_SIGN
                       * (TEST_SERVO_PID_KP_DEG_PER_RAD * yaw_error_rad);
  servo_angle_deg = SERVO_CENTER_ANGLE_DEG + servo_correction_deg;

  if (servo_angle_deg > (SERVO_CENTER_ANGLE_DEG + SERVO_MAX_DELTA_DEG))
  {
    servo_angle_deg = SERVO_CENTER_ANGLE_DEG + SERVO_MAX_DELTA_DEG;
  }
  else if (servo_angle_deg < (SERVO_CENTER_ANGLE_DEG - SERVO_MAX_DELTA_DEG))
  {
    servo_angle_deg = SERVO_CENTER_ANGLE_DEG - SERVO_MAX_DELTA_DEG;
  }

  Motor_SetLeftDutyPercent(0);
  Motor_SetRightDutyPercent(0);
  Servo_SetAngleDeg(servo_angle_deg);
}

static float Motion_GetCorrectedYawRad(void)
{
  float corrected_yaw_deg = fusion_tracker.euler.angle.yaw;

  if (fusion_yaw_zero_valid != 0U)
  {
    corrected_yaw_deg -= fusion_yaw_zero_deg;
  }

  return Motion_WrapAngleRad(corrected_yaw_deg * DEG_TO_RAD);
}

static int8_t Motion_ComputeSignedDutyPercent(float linear_cmd_m_s)
{
  float magnitude_cmd;
  float speed_limit;
  float normalized_linear;
  int8_t duty_percent;
  uint8_t reverse;

  if ((linear_cmd_m_s > -LINEAR_CMD_DEADBAND_M_S) && (linear_cmd_m_s < LINEAR_CMD_DEADBAND_M_S))
  {
    return 0;
  }

  reverse = (linear_cmd_m_s < 0.0f) ? 1U : 0U;
  magnitude_cmd = reverse ? -linear_cmd_m_s : linear_cmd_m_s;
  if (reverse && magnitude_cmd > MAX_REVERSE_SPEED_M_S)
  {
    magnitude_cmd = MAX_REVERSE_SPEED_M_S;
  }
  else if (!reverse && magnitude_cmd > MAX_FORWARD_SPEED_M_S)
  {
    magnitude_cmd = MAX_FORWARD_SPEED_M_S;
  }

  normalized_linear = magnitude_cmd / MAX_FORWARD_SPEED_M_S;

  duty_percent = (int8_t)(normalized_linear * (float)MAX_DUTY_PERCENT);
  if ((duty_percent > 0) && (duty_percent < (int8_t)MIN_EFFECTIVE_DUTY_PERCENT))
  {
    duty_percent = (int8_t)MIN_EFFECTIVE_DUTY_PERCENT;
  }

  return reverse ? (int8_t)(-duty_percent) : duty_percent;
}

static void Motion_UpdateOdometry(int32_t left_delta_count, int32_t right_delta_count,
                                  float wheel_circumference_mm, uint32_t now_ms)
{
  float delta_left_mm;
  float delta_right_mm;
  float delta_center_m;
  float dt_s = CONTROL_LOOP_DT_S;

  if (odom_last_update_ms == 0U)
  {
    odom_last_update_ms = now_ms;
    return;
  }

  delta_left_mm = ((float)left_delta_count * wheel_circumference_mm) / MOTOR_LEFT_ENCODER_COUNTS_PER_REV;
  delta_right_mm = ((float)right_delta_count * wheel_circumference_mm) / MOTOR_RIGHT_ENCODER_COUNTS_PER_REV;
  delta_center_m = ((delta_left_mm + delta_right_mm) * 0.5f) / 1000.0f;
  wheel_left_speed_m_s = (delta_left_mm / 1000.0f) / dt_s;
  wheel_right_speed_m_s = (delta_right_mm / 1000.0f) / dt_s;

  odom_theta_rad = Motion_GetCorrectedYawRad();
  odom_x_m += delta_center_m * cosf(odom_theta_rad);
  odom_y_m += delta_center_m * sinf(odom_theta_rad);
  odom_linear_speed_m_s = delta_center_m / dt_s;
  odom_angular_speed_rad_s = imu_last_gyro_z_dps * DEG_TO_RAD;
  odom_last_update_ms = now_ms;
}

static float Motion_WrapAngleRad(float angle_rad)
{
  while (angle_rad > 3.1415926f)
  {
    angle_rad -= 6.2831852f;
  }

  while (angle_rad < -3.1415926f)
  {
    angle_rad += 6.2831852f;
  }

  return angle_rad;
}

static float Motion_ClampFloat(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static void Motion_SendTelemetry(void)
{
  float ax_m_s2 = ((float)imu_last_accel_x_mg / 1000.0f) * STANDARD_GRAVITY_M_S2;
  float ay_m_s2 = ((float)imu_last_accel_y_mg / 1000.0f) * STANDARD_GRAVITY_M_S2;
  float az_m_s2 = ((float)imu_last_accel_z_mg / 1000.0f) * STANDARD_GRAVITY_M_S2;
  float gx_rad_s = imu_last_gyro_x_dps * DEG_TO_RAD;
  float gy_rad_s = imu_last_gyro_y_dps * DEG_TO_RAD;
  float gz_rad_s = imu_last_gyro_z_dps * DEG_TO_RAD;
  float current_yaw_rad = Motion_GetCorrectedYawRad();
  float yaw_target_rad = straight_hold_target_yaw_rad;
  float yaw_error_rad = Motion_WrapAngleRad(yaw_target_rad - current_yaw_rad);
  float pending_turn_target_rad = turn90_target_yaw_rad;

  UART2_OLED_Printf("ENC,%ld,%ld\r\n",
                    motor_left_total_count,
                    motor_right_total_count);
  UART2_OLED_Printf("ODOM,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                    odom_x_m,
                    odom_y_m,
                    odom_theta_rad,
                    odom_linear_speed_m_s,
                    odom_angular_speed_rad_s);
  UART2_OLED_Printf("IMU,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                    ax_m_s2,
                    ay_m_s2,
                    az_m_s2,
                    gx_rad_s,
                    gy_rad_s,
                    gz_rad_s);
  UART2_OLED_Printf("CTRL,lock=%u,active=%u,pending=%u,yaw=%.3f,target=%.3f,err=%.3f,pend=%.3f,v=%.3f,w=%.3f\r\n",
                    heading_is_locked,
                    straight_hold_active,
                    turn90_target_valid,
                    current_yaw_rad,
                    yaw_target_rad,
                    yaw_error_rad,
                    pending_turn_target_rad,
                    command_linear_m_s,
                    command_angular_rad_s);
  UART2_OLED_Printf("YAW,%.3f\r\n", current_yaw_rad * (180.0f / 3.14159265f));

  uint8_t h, m, s;
  MyRTC_GetTime(&h, &m, &s);
  UART2_OLED_Printf("RTC,%02d:%02d:%02d\r\n", h, m, s);

  UART2_OLED_Printf("BATT,%.2f,%.2f,%.1f\r\n",
                    battery_voltage_v,
                    battery_current_a,
                    battery_soc_percent);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
