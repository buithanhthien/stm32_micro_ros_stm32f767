/*
 * robot_params.c
 *
 *  Created on: Dec 3, 2025
 *      Author: khoi2
 */
#include "robot_params.h"

int32_t g_encCount1 = 0;
int32_t g_encCount2 = 0;
float g_speed1_mps = 0.0f;
float g_speed2_mps = 0.0f;

PID_TypeDef PosPID1, PosPID2;
PID_TypeDef SpeedPID1, SpeedPID2;

double CurPos1 = 0, CurSpeed1 = 0, PosPIDOut1 = 0, SpeedPIDOut1 = 0, DesiredPos1 = 0, DesiredSpeed1 = 0;
double CurPos2 = 0, CurSpeed2 = 0, PosPIDOut2 = 0, SpeedPIDOut2 = 0, DesiredPos2 = 0, DesiredSpeed2 = 0;

double Spd1_cmdfromROS = 0, Spd2_cmdfromROS = 0;

// Thông số PID tối ưu cho tải nặng ~50kg (2 bánh hoverboard 6.5" + 4 bánh đa hướng):
// Kp=220 phản ứng vững vàng, không gây giật PWM; Ki=50 bù tích phân mượt mà không vọt lố
double Kp1 = 1800, Ki1 = 800, Kd1 = 0;
double Kp2 = 1800, Ki2 = 800, Kd2 = 0;

double Kp_fromLCD1=2500, Ki_fromLCD1=350, Kd_fromLCD1=0;
double Kp_fromLCD2=2500, Ki_fromLCD2=350, Kd_fromLCD2=0;

uint8_t pid_update_pending = 0;  //biến dùng để update PID từ ROS

float g_revCount1 = 0.0f;
float g_revCount2 = 0.0f;

// Biến trạng thái Vòng tham chiếu đồng bộ 2 bánh xe
volatile float g_sync_e_pos = 0.0f;
volatile float g_sync_v_corr = 0.0f;
volatile uint8_t g_sync_active = 0;

// Hệ thống Telemetry Logging (Ring Buffer RAM & Live Watch)
RobotTelemetry_t g_telemetry_buf[TELEMETRY_LOG_SIZE];
volatile RobotTelemetry_t g_telemetry_latest = {0};
volatile uint16_t g_telemetry_idx = 0;
volatile uint32_t g_telemetry_total_samples = 0;
volatile uint8_t  g_telemetry_enable_uart = 1;  // Mặc định bật truyền UART3 (115200 baud)



