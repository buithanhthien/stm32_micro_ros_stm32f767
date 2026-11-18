/*
 * motor.c
 *
 *  Created on: Dec 2, 2025
 *      Author: khoi2
 */
#include "motor.h"
#include "tim.h"
#include "gpio.h"   // HAL_GPIO_WritePin
#include <stdlib.h> // abs
#include <math.h>   // fabs
#include "robot_params.h"

// Map chân DIR (Mới: DIR1=PF15, DIR2=PE13)
#define MOTOR1_DIR_GPIO_Port   GPIOF
#define MOTOR1_DIR_Pin         GPIO_PIN_15   // PF15

#define MOTOR2_DIR_GPIO_Port   GPIOE
#define MOTOR2_DIR_Pin         GPIO_PIN_13   // PE13

// Hướng quay thực tế của 2 động cơ (để 2 bánh cùng quay TIẾN khi SpeedPIDOut > 0):
// Motor 1 / Bánh phải: DIR1=SET (1) khi tiến cơ học
// Motor 2 / Bánh trái: DIR2=RESET (0) khi tiến cơ học
#define MOTOR1_SIGN   (+1)   // Motor 1 / Bánh phải: DIR=SET(1) khi tiến
#define MOTOR2_SIGN   (-1)   // Motor 2 / Bánh trái: DIR=RESET(0) khi tiến

// Dùng TIM1 Channel 1 (PE9) & Channel 2 (PE11)
extern TIM_HandleTypeDef htim1;

// Trạng thái đảo chiều an toàn cho từng motor
typedef struct {
    uint8_t current_dir;       // Chiều hiện tại đang đặt trên chân DIR
    uint8_t pending_dir;       // Chiều đích cần chuyển sang
    uint8_t deadtime_ticks;    // Số chu kỳ 5ms giữ PWM=0 trong quá trình đổi chiều
} MotorDirSafeState;

static MotorDirSafeState s_motor_state[2] = {
    { .current_dir = 1, .pending_dir = 1, .deadtime_ticks = 0 },
    { .current_dir = 1, .pending_dir = 1, .deadtime_ticks = 0 }
};

#define REVERSAL_DEADTIME_TICKS   10 // 10 ticks * 5ms = 50ms chu kỳ bảo vệ khi đảo chiều (tăng từ 35ms→50ms cho Motor1 đảo chiều dứt khoát hơn)
#define REVERSAL_FLIP_TICK        5  // Tại tick 5 (sau 25ms ngắt PWM) thì mới lật chân DIR (đủ thời gian xả dòng cuộn dây)

static void Motor_SetDir(MotorId id, uint8_t dir)
{
    GPIO_TypeDef *port;
    uint16_t pin;

    if (id == MOTOR_1) {
        port = MOTOR1_DIR_GPIO_Port;
        pin  = MOTOR1_DIR_Pin;
    } else {
        port = MOTOR2_DIR_GPIO_Port;
        pin  = MOTOR2_DIR_Pin;
    }

    HAL_GPIO_WritePin(port, pin, dir ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Init(void)
{
    // Start PWM cho 2 kênh TIM1 (PE9 & PE11)
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    __HAL_TIM_MOE_ENABLE(&htim1); // Kích hoạt Main Output Enable cho Timer nâng cao TIM1

    // Khởi tạo chiều quay mặc định ban đầu đồng bộ với hướng tiến cơ học:
    // Motor 1: DIR1=SET (1) khi tiến
    // Motor 2: DIR2=RESET (0) khi tiến
    s_motor_state[MOTOR_1].current_dir = 1;
    s_motor_state[MOTOR_1].pending_dir = 1;
    s_motor_state[MOTOR_1].deadtime_ticks = 0;
    Motor_SetDir(MOTOR_1, 1);

    s_motor_state[MOTOR_2].current_dir = 0;
    s_motor_state[MOTOR_2].pending_dir = 0;
    s_motor_state[MOTOR_2].deadtime_ticks = 0;
    Motor_SetDir(MOTOR_2, 0);

    // Mặc định: Motor dừng
    Motor_SetSpeed(MOTOR_1, 0);
    Motor_SetSpeed(MOTOR_2, 0);
}

void Motor_SetSpeed(MotorId id, int16_t speed)
{
    if (id != MOTOR_1 && id != MOTOR_2) return;

    // Clamp speed vào [-MOTOR_SPEED_MAX .. MOTOR_SPEED_MAX]
    if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
    if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;

    int16_t final_speed = speed;

    // Áp dụng SIGN cho từng motor trước khi xử lý
    if (id == MOTOR_1) {
        final_speed = (int16_t)(MOTOR1_SIGN * final_speed);
    } else {
        final_speed = (int16_t)(MOTOR2_SIGN * final_speed);
    }

    uint8_t target_dir = (final_speed >= 0) ? 1 : 0;
    int16_t mag = (final_speed >= 0) ? final_speed : -final_speed;   // độ lớn tuyệt đối

    uint32_t channel = (id == MOTOR_1) ? TIM_CHANNEL_1 : TIM_CHANNEL_2;

    // 1. Khi lệnh dừng hoặc công suất quá nhỏ (< 5/1200): ngắt PWM ngay lập tức
    if (mag < 5) {
        __HAL_TIM_SET_COMPARE(&htim1, channel, 0);
        // Khi PWM = 0 (dừng/không có dòng), cập nhật ngay chân DIR an toàn tuyệt đối, triệt tiêu kẹt deadtime
        if (target_dir != s_motor_state[id].current_dir) {
            s_motor_state[id].current_dir = target_dir;
            s_motor_state[id].pending_dir = target_dir;
            s_motor_state[id].deadtime_ticks = 0;
            Motor_SetDir(id, target_dir);
        } else if (s_motor_state[id].deadtime_ticks > 0) {
            s_motor_state[id].deadtime_ticks = 0;
        }
        return;
    }

    // 2. Kiểm tra nếu có yêu cầu đổi chiều khi motor đang chạy:
    // Tuyệt đối không đổi DIR khi PWM > 0 (tránh nổ/treo driver hoặc driver bỏ qua lệnh)
    if (target_dir != s_motor_state[id].current_dir) {
        if (s_motor_state[id].deadtime_ticks == 0) {
            s_motor_state[id].pending_dir = target_dir;
            s_motor_state[id].deadtime_ticks = REVERSAL_DEADTIME_TICKS;
            // Bước 1: NGAY LẬP TỨC triệt tiêu PWM về 0
            __HAL_TIM_SET_COMPARE(&htim1, channel, 0);
            return;
        }
    }

    // 3. Đang trong giai đoạn dead-time đảo chiều an toàn
    if (s_motor_state[id].deadtime_ticks > 0) {
        // Giữ PWM = 0 tuyệt đối trong toàn bộ deadtime
        __HAL_TIM_SET_COMPARE(&htim1, channel, 0);

        s_motor_state[id].deadtime_ticks--;

        // Đến giữa dead-time (sau 25ms xả dòng cảm ứng cuộn dây): lật chân DIR
        if (s_motor_state[id].deadtime_ticks == REVERSAL_FLIP_TICK) {
            s_motor_state[id].current_dir = s_motor_state[id].pending_dir;
            Motor_SetDir(id, s_motor_state[id].current_dir);
        }

        return; // Chưa hết dead-time thì không cấp PWM
    }

    // 4. Khi chiều đã đồng bộ và không trong dead-time: Xuất PWM an toàn
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); // = 1199
    uint32_t max_counts = arr + 1U;                  // = 1200

    // Scale từ [0..MOTOR_SPEED_MAX] → [0..ARR]
    uint32_t duty_counts = (uint32_t)mag * max_counts / MOTOR_SPEED_MAX;
    if (duty_counts > arr) {
        duty_counts = arr;   // đảm bảo không vượt ARR
    }

    __HAL_TIM_SET_COMPARE(&htim1, channel, duty_counts);
}

void Apply_Robot_Ramping(double *current_target, double raw_target)
{
    // Gia tốc giới hạn cho robot cao 1.6m, nặng 50kg:
    // ACCEL_RATE = 0.18 m/s^2: tăng tốc siêu êm (0 -> 0.08 m/s mất ~0.44s)
    // DECEL_RATE = 0.22 m/s^2: hãm phanh từ từ, triệt tiêu hoàn toàn cú giật chúi đầu do quán tính cao
    const double ACCEL_RATE = 0.18;
    const double DECEL_RATE = 0.22;

    double max_step;
    if ((*current_target * raw_target < 0.0) ||
        (fabs(raw_target) < fabs(*current_target))) {
        // Giảm tốc khi giảm độ lớn hoặc khi cần đảo chiều
        max_step = DECEL_RATE * (double)DT_TIME;
    } else {
        // Đang tăng tốc lên
        max_step = ACCEL_RATE * (double)DT_TIME;
    }

    if (*current_target < raw_target) {
        *current_target += max_step;
        if (*current_target > raw_target) {
            *current_target = raw_target;
        }
    }
    else if (*current_target > raw_target) {
        *current_target -= max_step;
        if (*current_target < raw_target) {
            *current_target = raw_target;
        }
    }
}
