/*
 * robot_params.h
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */

#ifndef INC_ROBOT_PARAMS_H_
#define INC_ROBOT_PARAMS_H_

#include "pid.h"

/* ====================================================================
 * CẤU HÌNH BẬT / TẮT MICRO-ROS
 * 0: TẮT micro-ROS (chạy test phần cứng độc lập: Hall, PWM, PID, PS2 pad)
 * 1: BẬT micro-ROS (khi đã build thư viện libmicroros)
 * ==================================================================== */
#define ENABLE_MICROROS            1

/* ====================================================================
 * THÔNG SỐ CẢM BIẾN HALL & ĐỘNG CƠ BLDC HOVERBOARD
 * (CẤU HÌNH CHO NGƯỜI DÙNG ĐIỀN THÔNG SỐ THỰC TẾ)
 * ==================================================================== */

/**
 * Số đôi cực từ (Pole pairs) của động cơ BLDC Hoverboard:
 * - Động cơ hoverboard phổ biến (bánh 6.5" - 10") thường có 15 đôi cực (30 nam châm vĩnh cửu).
 * - Người dùng điền thông số thực tế tại đây:
 */
#define MOTOR_LEFT_POLE_PAIRS      15      /* ĐIỀN THÔNG SỐ THỰC TẾ SỐ ĐÔI CỰC MOTOR TRÁI */
#define MOTOR_RIGHT_POLE_PAIRS     15      /* ĐIỀN THÔNG SỐ THỰC TẾ SỐ ĐÔI CỰC MOTOR PHẢI */

/**
 * Tỷ số truyền (Gear ratio):
 * - Động cơ Hoverboard gắn trực tiếp vào bánh xe (Direct-Drive Hub Motor): GEAR_RATIO = 1.0f.
 * - Nếu dùng động cơ có hộp số giảm tốc, điền tỷ số truyền thực tế tại đây:
 */
#define MOTOR_LEFT_GEAR_RATIO      1.0f    /* ĐIỀN TỶ SỐ TRUYỀN MOTOR TRÁI (Hub motor = 1.0) */
#define MOTOR_RIGHT_GEAR_RATIO     1.0f    /* ĐIỀN TỶ SỐ TRUYỀN MOTOR PHẢI (Hub motor = 1.0) */

/**
 * Chiều quay đếm Hall (+1 hoặc -1):
 * - Đảm bảo CurSpeed CÙNG DẤU với SpeedPIDOut và DesiredSpeed (Phản hồi âm ổn định):
 */
#define MOTOR_LEFT_DIRECTION      (+1)    /* Đã đảo lại +1 để CurSpeed2 cùng dấu với SpeedPIDOut2/DesiredSpeed2 */
#define MOTOR_RIGHT_DIRECTION     (-1)    /* Motor Phai: (-1) vi chuoi pha Hall quay tien cho step=-1, nhan voi (-1) se dem DUONG (+) va CurSpeed1 DUONG (+) */

/**
 * Số cạnh chuyển trạng thái Hall trên 1 vòng quay bánh xe:
 *   HALL_EDGES_PER_MOTOR_REV = 6 * POLE_PAIRS
 *   HALL_TICKS_PER_WHEEL_REV = 6 * POLE_PAIRS * GEAR_RATIO
 */
#define HALL_TICKS_PER_REV_L       ((float)(6 * MOTOR_LEFT_POLE_PAIRS) * MOTOR_LEFT_GEAR_RATIO)
#define HALL_TICKS_PER_REV_R       ((float)(6 * MOTOR_RIGHT_POLE_PAIRS) * MOTOR_RIGHT_GEAR_RATIO)

/**
 * Tương thích ngược với TICKS_PER_REV cũ (thay thế giá trị 28000 của encoder cũ):
 */
#define TICKS_PER_REV              ((HALL_TICKS_PER_REV_L + HALL_TICKS_PER_REV_R) * 0.5f)

/* ====================================================================
 * THÔNG SỐ CƠ KHÍ ROBOT (GIỮ NGUYÊN)
 * ==================================================================== */
#define WHEEL_DIAMETER_M   0.1651f     // Đường kính bánh xe (10 cm)
#define WHEEL_CIRC_M       (3.1415926f * WHEEL_DIAMETER_M)
#define WHEEL_BASE_M       0.551f    // Khoảng cách 2 bánh xe (m)

// Chu kỳ lấy mẫu cho các task (ms)
#define ODOMETRY_PERIOD_MS     5
#define MOTORCONTROL_PERIOD_MS 5
#define ENCODER_PERIOD_MS      5
#define UART_PERIOD_MS         50
#define DT_TIME                0.005f

// Giới hạn tốc độ và gia tốc (Tối ưu cho robot cao 1.6m, nặng 50kg, 2 bánh 6.5" + 4 bánh đa hướng)
#define MAXSPEED           0.08f     // Vận tốc trần 0.08 m/s cho chuyển động siêu êm
#define V_MAX              0.08f     // Tốc độ dài cực đại (m/s)
#define V_STEP             0.02f     // Bước tốc độ D-pad (m/s)

// Mức bù công suất ma sát tĩnh cơ bản (Stiction Offset PWM) cho tải nặng 50kg:
// Được tăng dốc mềm mại (Ramped Stiction Feedforward) theo DesiredSpeed từ 0 -> 125 PWM trong 110ms,
// cung cấp đầy đủ lực kéo mạnh mẽ để Bánh 1 & 2 cùng thắng ma sát nghỉ mà không gây giật đề-ba.
#define MOTOR_MIN_PWM          125.0f

// Hệ số bù trước Feedforward vận tốc động (Dynamic Feedforward)
#define MOTOR_FEEDFORWARD_KV   350.0f

#define MAXACC             2.5f

// Hằng số bộ điều khiển PS2 (Tốc độ góc giới hạn ~14.3 độ/s để robot cao 1.6m quay mượt, an toàn)
#define W_MAX              0.25f     // Tốc độ góc cực đại (rad/s) (~14.3 độ/giây)
#define W_STEP             0.08f     // Bước tốc độ xoay D-pad (~4.6 độ/giây)

#define PS2_TIMEOUT_MS     500
#define STICK_DEADBAND     12        // Vùng chết cần analog (0..255)

// Ngưỡng kích hoạt đi thẳng / quay đầu
#define EPS_TURN_MPS       0.015f   // Nếu |vR_ref - vL_ref| < this => lệnh đi thẳng (m/s)
#define EPS_MOVE_MPS       0.015f   // Ngưỡng tốc độ để kích hoạt vòng đồng bộ (phù hợp cả với bước D-pad 0.02 m/s)

// Thông số Vòng tham chiếu đồng bộ 2 bánh xe (Dual-Wheel Cross-Coupling Reference Loop)
#define SYNC_KP            0.80f    // Hệ số bù sai số quãng đường dịu dàng, nắn thẳng từ từ
#define SYNC_KD            0.02f    // Hệ số bù sai số vận tốc tức thời
#define V_SYNC_CORR_MAX    0.015f   // Giới hạn vận tốc bù đồng bộ cực đại (m/s)

// Hệ số bộ điều khiển hướng cũ (giữ tương thích)
#define HEADING_KP         2.0f
#define HEADING_KD         0.1f
#define HEADING_DT         0.02f   // 20 ms (50 Hz)
#define HEADING_DIV        4       // 4 ticks * 5ms = 20ms
#define OMEGA_CORR_MAX     2.0f

// Biến trạng thái Vòng tham chiếu đồng bộ (Live Debug)
extern volatile float g_sync_e_pos;
extern volatile float g_sync_v_corr;
extern volatile uint8_t g_sync_active;

// Biến điều khiển PID
extern PID_TypeDef PosPID1, PosPID2;
extern PID_TypeDef SpeedPID1, SpeedPID2;

extern double CurPos1, CurSpeed1, PosPIDOut1, SpeedPIDOut1, DesiredPos1, DesiredSpeed1;
extern double CurPos2, CurSpeed2, PosPIDOut2, SpeedPIDOut2, DesiredPos2, DesiredSpeed2;

extern volatile double SetPos1, SetPos2, SetSpeed1, SetSpeed2;
extern double Spd1_cmdfromROS, Spd2_cmdfromROS;

extern double Kp1, Ki1, Kd1;
extern double Kp2, Ki2, Kd2;

// Các biến lưu thông số PID từ giao diện (bảo toàn định nghĩa để giữ tương thích)
extern double Kp_fromLCD1, Ki_fromLCD1, Kd_fromLCD1;
extern double Kp_fromLCD2, Ki_fromLCD2, Kd_fromLCD2;
extern uint8_t pid_update_pending;

extern int32_t PID_sampletime;

// Các biến encoder / hall toàn cục
extern int32_t g_encCount1;
extern int32_t g_encCount2;
extern float g_speed1_mps;
extern float g_speed2_mps;
extern float g_revCount1;
extern float g_revCount2;

#endif /* INC_ROBOT_PARAMS_H_ */
