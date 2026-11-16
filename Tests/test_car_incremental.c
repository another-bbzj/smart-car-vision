#include "Car_Core.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* 增量式速度环接线测试：本文件须用 -DCAR_SPEED_PID_INCREMENTAL=1 编译。
 * 验证 Car_Core 会选择 Car_PidInc_* 且增益同步、输出有限、不触发故障。 */

int main(void) {
    Car_Core car;
    int i;
    Car_Init(&car);
    assert(car.state == CAR_STOP);
    assert(car.pid_inc[0].kp == car.pid[0].kp);
    assert(car.pid_inc[1].ki == car.pid[1].ki);

    assert(Car_SetPid(&car, 0, 3.0f, 0.4f, 0.1f) == 0);
    assert(car.pid[0].kp == 3.0f && car.pid_inc[0].kp == 3.0f);
    assert(car.pid[0].ki == 0.4f && car.pid_inc[0].ki == 0.4f);
    assert(car.pid[0].kd == 0.1f && car.pid_inc[0].kd == 0.1f);

    /* 直接置 RUN 并给定目标，模拟闭环运行。 */
    car.state = CAR_RUN;
    car.target[0] = car.target[1] = 80.0f;
    for (i = 0; i < 400; i++) {
        car.link_us = 0;           /* 本测试不模拟蓝牙心跳，手动喂看门狗 */
        Car_Tick(&car, 12, 12);   /* 固定 12 计数/5 ms，对应约 327 rpm */
        assert(car.state == CAR_RUN);
        assert(isfinite(car.output[0]) && isfinite(car.output[1]));
        assert(car.output[0] >= -CAR_PWM_LIMIT && car.output[0] <= CAR_PWM_LIMIT);
        assert(car.output[1] >= -CAR_PWM_LIMIT && car.output[1] <= CAR_PWM_LIMIT);
    }
    assert(fabsf(car.speed[0] - car.speed[1]) < 1e-3f);

    /* 本地停车立即撤输出并复位增量历史。 */
    Car_LocalStop(&car);
    assert(car.state == CAR_STOP);
    assert(car.output[0] == 0.0f && car.output[1] == 0.0f);
    assert(car.pid_inc[0].last_output == 0.0f);

    puts("car incremental PID tests passed");
    return 0;
}
