# STM32F103 PID 电机恒速控制系统

一个基于 STM32F103C8T6 的完整 PID 电机恒速控制解决方案，采用 PWM 控制、编码器反馈测速和增量式 PID 算法。

##  项目特性

- **PID 控制**：增量式 PID 算法，支持积分饱和保护、测量低通滤波、输出斜率限制
- **PWM 输出**：1kHz PWM 频率，PA8 端口驱动电机（TIM1）
- **编码器反馈**：20 脉冲/转的编码器，通过 TIM4 编码器模式测速
- **定时控制**：TIM2 提供 10ms 周期的 PID 控制任务调度
- **串口通信**：UART1 (115200 8-N-1) 用于 printf 调试和命令交互
- **死区补偿**：处理电机低转速死区，确保控制稳定性
- **实时监测**：通过串口 CSV 格式输出目标转速和实际转速，便于 VOFA 等工具可视化

##  硬件要求

- **MCU**：STM32F103CB (LQFP48)
- **时钟**：8MHz 外部晶振，72MHz 系统时钟
- **电机**：12V 直流电机（需外部驱动模块，如 L298N）
- **编码器**：编码器电机附带
- **其他**：USB 转串口模块（调试用）

##  引脚配置

| 功能 | 引脚 | 定时器 | 说明 |
|------|------|--------|------|
| PWM 输出 | PA8 | TIM1_CH1 | 1kHz，范围 0-999 |
| 编码器 CH1 | PB6 | TIM4_CH1 | 编码器 A 相 |
| 编码器 CH2 | PB7 | TIM4_CH2 | 编码器 B 相 |
| 电机方向 | PB0 | GPIO | 1=前进，0=反转 |
| 电机使能 | PB1 | GPIO | 预留 |
| UART TX | PA9 | USART1 | 调试输出 |
| UART RX | PA10 | USART1 | 数据接收 |

##  目录结构

```
motor_pid/
├── Core/
│   ├── Inc/                    # 头文件
│   │   ├── main.h
│   │   ├── pid.h              # PID 控制接口
│   │   ├── motor.h            # 电机驱动接口
│   │   ├── tim.h              # 定时器配置
│   │   └── usart.h            # 串口配置
│   └── Src/                    # 源代码文件
│       ├── main.c             # 主程序
│       ├── pid.c              # PID 算法实现
│       ├── motor.c            # 电机控制实现
│       ├── usart_cmd.c        # 串口命令解析
│       ├── tim.c              # 定时器初始化
│       └── ...                # 其他 HAL 初始化代码
├── Drivers/                    # STM32 HAL 驱动库
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── cmake/                      # CMake 构建配置
├── build/                      # 编译输出目录（不提交）
├── CMakeLists.txt             # CMake 构建脚本
├── PID_Motor.ioc              # STM32CubeMX 项目文件
└── README.md                  # 本文件
```


### 编码条件

- **CMake** >= 3.22
- **ARM GCC 工具链**：`arm-none-eabi-gcc`
- **STM32CubeMX**
- **STM32CubeIDE for Visual Studio Code插件**

### 编译

详见 编译.png

编译后生成的文件：
- `Debug/PID_Motor.elf` - ELF 可执行文件
- `Debug/PID_Motor.hex` - Intel HEX 格式
- `Debug/PID_Motor.bin` - 二进制格式

### 烧录
详见烧录.png






