CubeMX 完整配置步骤

### 需求
这就是完整的 PWM+编码器测速+增量式PID 12V 电机恒速+串口输出 系统结构。

### 1. 选择芯片

打开 STM32CubeMX →
New Project → Board Selector / MCU Selector → STM32F103C8Tx

### 2. 时钟（Clock Configuration）

设置系统时钟为 72 MHz

HSE = 8 MHz（一般是外置晶振）

PLL 开启

PLLCLK = 72MHz

APB1 = 36MHz（定时器倍频后仍为 72MHz）

APB2 = 72MHz

### 3. 配置 PWM（控制电机）
👉“Timers” → TIM1 → Mode → PWM Generation CH1

CubeMX 中：

PA8 → TIM1_CH1（变成 PWM 图标）

PWM 模式：PWM Generation CH1

Prescaler: 71（得到 1MHz）

Counter Period: 999（PWM 频率 = 1kHz）

PWM1 模式

使能预装载（Auto-reload preload）

PWM 频率计算：
72 MHz / (71+1) = 1 MHz  
PWM freq = 1 MHz / (999+1) = 1 kHz

➕ 你后面调 PID 也可以把 PWM 频率改为 10kHz。
### 4. 配置编码器输入（测速）
👉选择：TIM4 → Encoder Mode

引脚自动占用：

PB6 → TIM4_CH1

PB7 → TIM4_CH2

CubeMX 选项：

Encoder Mode：TI12

Counter Period：65535

Prescaler：0

Input Capture Filter：3~10（滤波避免跳变）

Polarity：Rising Edge

这样 TIM4 会自动累加编码器脉冲数，用于测速。

### 5. 配置串口 UART
👉“Connectivity” → USART1 → Asynchronous

引脚：

PA9 → TX

PA10 → RX

参数：

115200 8-N-1（或你想要的 921600 / 1Mbps）

使能中断可选（一般只 TX 打印不需要）

### 6. 配置一个定时器作为 PID 定时任务（强烈建议）

PID 一般需要固定周期，比如 10ms。

使用 TIM2：

👉配置 TIM2 为：

Internal Clock

Prescaler: 7199（得到 10kHz）

Counter Period: 99（100 次 = 10ms）

Update Event / Interrupt：Enabled

也就是：

72MHz / 7200 = 10kHz
10kHz / 100 = 100Hz → 10ms


在 NVIC 中：

TIM2 global interrupt：Enable

这样我们可以在 HAL_TIM_PeriodElapsedCallback() 中运行 PID。

### 7. GPIO 其他

如果你用的是电机驱动（L298N / BTS7960 / VNH5019），需要额外引脚：

例如：

控制	引脚
DIR1	PB0
DIR2	PB1

CubeMX 设为 Output Push-Pull。



### CubeMX 配置

TIM1 PWM

TIM4 Encoder

TIM2 PID 定时器

USART1 串口

时钟 72MHz

main.c 启动外设

定时器 TIM2 每 10ms 做一次：

编码器测速

增量式PID

更新 PWM

目标速度和当前速度以及pid是要串口打印到vofa上位机进行调参（printf重定向）


