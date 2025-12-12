# Proteus 仿真 STM32 PID 恒转速方案（详细版，面向Cursor编写）

> 目标：在无实物条件下，用 Proteus + STM32 仿真 PID 调速 12V 电机，串口打印调试信息。
> 回答插件疑问：Cursor 不需要额外插件，可选装 C/C++ 语法提示，必需的是 Proteus 与 STM32 编译工具链。

---

## 1. 必备工具与版本
- Proteus：推荐 8.13 以上，需包含 Virtual Terminal、Logic Analyzer。
- STM32 开发环境：STM32CubeIDE（推荐）或 Keil MDK（任意一种即可）。
- 设备型号：STM32F103C8T6（Proteus 模型丰富，例程多）。
- 其他：Git（可选）、文本编辑器/IDE（Cursor）。

---

## 2. 工程与目录（在 Cursor 中）
```
/stm32-pid-proteus
  ├─ Src/main.c            // 主循环、调度
  ├─ Src/pid.c/.h          // PID 算法（位置式/增量式）
  ├─ Src/motor.c/.h        // PWM 输出、测速输入（IC/Encoder）
  ├─ Src/usart.c/.h        // 串口初始化 & printf 重定向
  ├─ Src/system_stm32xx.c  // 时钟配置
  └─ CubeMX 工程或 Keil 工程文件
```
- 若用 CubeMX 生成：在 Cursor 继续编辑生成的 C 源文件即可，无需额外插件。

---

## 3. 外设与引脚规划（示例：STM32F103C8T6）
- 时钟：HSE 8MHz → PLL x9 → SYSCLK 72MHz。
- PWM：TIM1_CH1 (PA8) 输出占空比 → H 桥 EN/IN。
- 速度反馈（任选其一）：
  - Encoder 模式：TIM3_CH1/CH2 (PA6/PA7) 读取增量编码器。
  - 输入捕获：TIM2_CH1 (PA0) 读取霍尔/测速脉冲频率。
- 串口：USART1 TX/RX (PA9/PA10)，115200 8N1。
- 调度周期：1 ms 或 5 ms 定时中断执行 PID。

---

## 4. CubeMX 配置步骤（要点）
1) Clock：HSE on，PLL x9，APB1 36MHz，APB2 72MHz。  
2) GPIO：默认；PA9/PA10 复用推挽。  
3) TIM1：PWM 模式 1，通道 CH1，频率 20 kHz 左右，初值 0 占空比。  
4) TIM3（编码器模式）或 TIM2（输入捕获模式），采样频率确保能准确计数。  
5) USART1：115200 8N1，中断或轮询均可。  
6) SysTick / TIMx 周期：1 ms/5 ms 触发 PID 计算。  
7) 生成代码，后续在 Cursor 填充 `pid.c`、`motor.c`、`usart.c`、`main.c` 逻辑。

---

## 5. PID 逻辑设计
- 形式：位置式 PID（简洁，可读性高）。  
- 输入：`target_rpm`，`measured_rpm`。  
- 输出：`pwm_duty`（0~100%），带限幅 & 死区。  
- 积分抗饱和：积分限幅；当输出达上/下限且误差同向时暂停积分。  
- 调参顺序：先 Kp，后 Ki（消除稳态），再 Kd（抑制超调）。  
- 定时执行：在 1 ms/5 ms 定时中断内读取测速、计算 PID、更新 PWM；串口打印可放在主循环低频执行。

---

## 6. Proteus 电路搭建与连线
- 元件：STM32F103C8T6、L298N（或任意 H 桥）、DC Motor、12V 电源、Virtual Terminal、Ground、旁路电容。  
- 连线示例：  
  - PA8 (TIM1_CH1 PWM) → L298N EN/IN（按驱动需求接 ENA + IN1/IN2）。  
  - 电机 → 驱动输出端，驱动电源 12V。  
  - 速度反馈：  
    - 方案 A：DC Motor with Tacho 模型的测速输出 → STM32 PA0 (TIM2_CH1 输入捕获)。  
    - 方案 B：Rotary Encoder 模型 A/B 相 → STM32 PA6/PA7 (TIM3 Encoder)。  
  - 串口：PA9 (TX) / PA10 (RX) → Virtual Terminal。  
- HEX/ELF 加载：双击 MCU，设置 Program File 为编译生成的 `.hex` 或 `.elf`。

---

## 7. 串口输出格式（示例）
`printf("t=%.3fs, target=%d, rpm=%d, pwm=%d, P=%.2f I=%.2f D=%.2f\r\n");`

---

## 8. 详细落地步骤（按时间顺序）
1) 安装与准备（30-60 min）  
   - 安装 Proteus（含 Virtual Terminal/Logic Analyzer）。  
   - 安装 STM32CubeIDE 或 Keil。  
   - 在 Cursor 创建目录与基础文件骨架。  

2) CubeMX 生成初始化（30 min）  
   - 配置时钟、TIM1 PWM、TIM3 Encoder 或 TIM2 输入捕获、USART1。  
   - 生成工程，导入 IDE，确保能编译出空工程 HEX。  

3) 编写业务代码（Cursor 内完成，60-90 min）  
   - `pid.c/.h`：位置式 PID，带积分限幅、输出限幅、死区。  
   - `motor.c/.h`：PWM 设置函数、测速值计算（根据捕获/编码器计数换算 RPM）。  
   - `usart.c/.h`：初始化 + `int fputc` 重定向 printf。  
   - `main.c`：  
     - 全局目标转速 `target_rpm` 常量/可串口命令修改；  
     - 定时中断回调：读测速→PID→更新 PWM；  
     - 主循环：每 50~100 ms 打印一次状态。  

4) Proteus 画图与加载（45 min）  
   - 放置 MCU、H 桥、DC Motor、测速元件、Virtual Terminal。  
   - 连接 PWM、反馈脉冲、串口、12V 电源、地、旁路电容。  
   - 加载 HEX/ELF，设置时钟与复位引脚默认上拉。  

5) 调试与调参（60-90 min）  
   - 运行仿真，观察串口输出与 PWM 波形。  
   - 调 Kp：提升响应；  
   - 加 Ki：去稳态误差，注意积分限幅；  
   - 加 Kd：抑制超调；  
   - 用 Logic Analyzer 观察测速脉冲，必要时调整采样周期。  
   - 做负载扰动（在电机模型增加摩擦/负载步阶）验证恢复时间。  

6) 交付与演示（30 min）  
   - 保存 pdsprj、最终 HEX、源码。  
   - 记录最终 PID 参数与性能（上升时间、超调、稳态误差、恢复时间）。  
   - 录屏展示阶跃响应、扰动恢复、串口日志。  

---

## 9. 关键代码片段（可直接放入工程）
### PID 核心
```c
float pid_update(pid_t *p, float target, float meas) {
    float err = target - meas;
    p->integral += err * p->dt;
    if (p->integral > p->i_limit) p->integral = p->i_limit;
    if (p->integral < -p->i_limit) p->integral = -p->i_limit;
    float deriv = (err - p->prev_err) / p->dt;
    float out = p->kp * err + p->ki * p->integral + p->kd * deriv;
    if (out > p->out_limit) out = p->out_limit;
    if (out < 0) out = 0;            // 占空比下限
    p->prev_err = err;
    return out; // 占空比 0~100
}
```

### 串口 printf 重定向（Keil/CubeIDE 通用）
```c
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}
```

### 定时中断调度（伪代码）
```c
// 每 1 ms/5 ms 触发
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIMx_PID) {
        float rpm = motor_get_rpm();          // 从编码器/IC 换算
        float duty = pid_update(&pid, target_rpm, rpm);
        motor_set_pwm(duty);                  // 写 CCRx
        state_rpm = rpm; state_pwm = duty;
    }
}
```

---

## 10. 常见问题与排查
- PWM 无输出：检查 GPIO 复用推挽、TIM 使能、ARR/PSC 与 CCR 写入。  
- 速度测不准：确认编码器模式极性，或输入捕获滤波/预分频设置，采样周期是否过长。  
- 串口乱码：波特率匹配、虚拟终端设置、晶振与 PLL 配置。  
- PID 抖动/振荡：降低 Kp，增大采样周期滤波；加入简单一阶低通滤波测量值。  

---

## 11. 交付清单
- 源码：`pid.c/.h`, `motor.c/.h`, `usart.c/.h`, `main.c`。  
- 生成文件：最终稳定的 `.hex`/`.elf`。  
- 仿真：Proteus `.pdsprj`，含连线与参数。  
- 文档：最终 PID 参数和性能指标说明。  

