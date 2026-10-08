# RoboMini 机甲杯 硬件高级任务

基于 STM32F103 + FreeRTOS 的串口DMA不定长收发、MPU6050六轴姿态解算与OLED显示工程。

## 1. 硬件清单

| 模块 | 型号 | 说明 |
|------|------|------|
| 主控 | STM32F103C8T6（最小系统板） | 72MHz，64KB Flash，20KB RAM |
| IMU | MPU6050 模块 | 六轴（三轴加速度+三轴陀螺仪），I2C接口 |
| OLED | 0.96寸 SSD1306 OLED模块 | 128×64，软件I2C（模拟I2C） |
| 串口通信 | USB-TTL 模块（CH340/CP2102） | 连接电脑串口助手 |

## 2. 引脚接线表

| STM32引脚 | 连接目标 | 功能 |
|-----------|----------|------|
| PA9 | USB-TTL RX | USART1_TX |
| PA10 | USB-TTL TX | USART1_RX |
| PB6 | MPU6050 SCL | I2C1_SCL |
| PB7 | MPU6050 SDA | I2C1_SDA |
| PB10 | OLED SCL | 软件I2C SCL |
| PB11 | OLED SDA | 软件I2C SDA |
| 3.3V | MPU6050 VCC / OLED VCC | 供电 |
| GND | 各模块 GND | 共地 |

> 注意：MPU6050 和 OLED 共用 3.3V 供电，必须共地。

## 3. CubeMX 配置说明

- **时钟**：HSE 8MHz → PLL ×9 → SYSCLK 72MHz（AHB=36MHz，因 PA0-PA11 默认映射）
- **USART1**：115200 8N1，TX/RX 均开启 DMA
  - RX：DMA1_Channel5，循环模式（CIRCULAR）
  - TX：DMA1_Channel4，正常模式（NORMAL）
  - 使能 USART1 全局中断
- **I2C1**：400kHz Fast Mode，PB6/PB7
- **DMA1**：使能 Channel4/Channel5 中断
- **FreeRTOS**：CMSIS v1，4个任务 + 1个消息队列 + 1个二值信号量
- **TIM4**：作为 HAL 时基（FreeRTOS 占用了 SysTick）

## 4. FreeRTOS 任务结构

| 任务名 | 优先级 | 栈大小 | 功能 |
|--------|--------|--------|------|
| defaultTask | Normal | 128字 | 初始化：MPU6050、OLED、Mahony滤波、陀螺仪零偏校准 |
| UartTask | Normal | 256字 | 启动DMA接收，IDLE中断通知后DMA原样回传 |
| ImuTask | AboveNormal | 512字 | 100Hz读取MPU6050原始数据，Mahony解算，发队列 |
| OledTask | BelowNormal | 256字 | 从队列取姿态数据，刷新OLED显示Roll/Pitch/Yaw |

**任务间通信：**
- **二值信号量 `UartRxSem`**：USART1 IDLE中断 → UartTask，通知收到一帧不定长数据
- **消息队列 `AttitudeQueue`**：ImuTask → OledTask，传递 `Attitude_t*` 姿态指针

## 5. 代码目录说明

```
high_task/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── mpu6050.h / mpu6050.c      # MPU6050 驱动（I2C读原始数据）
│   │   ├── imu_filter.h / imu_filter.c # Mahony姿态解算
│   │   ├── OLED.h / OLED.c / OLED_Font.h # OLED软件I2C驱动
│   │   ├── usart.h                     # USART1 + DMA 配置
│   │   ├── i2c.h                       # I2C1 配置
│   │   ├── dma.h / gpio.h / freertos.c # CubeMX生成
│   │   └── stm32f1xx_it.h             # 中断服务函数
│   └── Src/
│       ├── main.c                      # 入口、时钟配置
│       ├── freertos.c                   # 任务定义与实现
│       └── stm32f1xx_it.c             # 中断处理（IDLE中断）
├── Drivers/                            # HAL库 + CMSIS
├── MDK-ARM/                            # Keil5 工程文件
└── high_task.ioc                       # STM32CubeMX配置文件
```

## 6. 串口通信收发处理流程

1. UartTask 启动后调用 `HAL_UART_Receive_DMA()` 以循环模式接收数据到 `rx_buf[256]`
2. 使能 USART1 IDLE 空闲中断
3. 上位机发送任意长度数据 → DMA 自动写入 rx_buf
4. 一帧发送完毕后总线空闲 → 触发 IDLE 中断
5. 中断中计算接收长度：`len = 256 - __HAL_DMA_GET_COUNTER(hdma_usart1_rx)`
6. 中断释放二值信号量，唤醒 UartTask
7. UartTask 收到信号量后：
   - 停止 DMA
   - 调用 `HAL_UART_Transmit_DMA()` 将 rx_buf 原样发回
   - 等待发送完成（TC标志）
   - 重新启动 DMA 接收
8. 上位机收到回传数据，与发送内容逐字节一致

## 7. IMU 姿态解算说明

- **芯片**：MPU6050，I2C地址 0x68（7位）
- **量程**：加速度 ±2g（灵敏度 16384 LSB/g），陀螺仪 ±250°/s（灵敏度 131 LSB/(°/s)）
- **校准**：开机时静止采样200次，计算陀螺仪 X/Y/Z 零偏均值，后续每次读取减去零偏
- **融合算法**：Mahony互补滤波（Kp=2.0, Ki=0.01），采样率100Hz
- **输出**：Roll（绕X轴）、Pitch（绕Y轴）、Yaw（绕Z轴），单位角度

> 注意：MPU6050 没有磁力计，Yaw角为陀螺仪积分所得，长时间运行会有漂移，这是正常现象。

## 8. OLED 显示说明

- 0.96寸 SSD1306，128×64分辨率，8×16 ASCII字库
- 软件模拟I2C驱动（PB10=SCL, PB11=SDA）
- 显示内容：
  - 第1行：`IMU Attitude`
  - 第2行：`Roll:  xx.x`
  - 第3行：`Pitch: xx.x`
  - 第4行：`Yaw:   xx.x`

## 9. 编译烧录步骤

1. 用 STM32CubeMX 打开 `high_task.ioc` 确认配置，生成代码（一般不需要重新生成）
2. 用 Keil MDK5 打开 `MDK-ARM/high_task.uvprojx`
3. 选择编译目标（Debug），点击 Build（F7）
4. 用 ST-Link / J-Link 连接 SWD 接口（SWDIO=PA13, SWCLK=PA14）
5. 点击 Download（F8）烧录
6. 复位运行

## 10. 遇到的问题与解决方法

1. **串口DMA接收不定长数据**：使用 IDLE 空闲中断 + DMA循环接收方案，中断中读取DMA剩余计数器算出实际接收长度
2. **FreeRTOS下HAL时基冲突**：SysTick被FreeRTOS占用，将HAL时基改为TIM4
3. **Mahony滤波器采样率必须匹配**：`Mahony_Init()` 的参数必须与任务 `osDelay()` 实际频率一致，否则姿态发散
4. **OLED用软件I2C**：硬件I2C1已被MPU6050占用，OLED通过GPIO模拟I2C通信，不冲突
5. **中断中不做耗时操作**：IDLE中断只释放信号量，回传由UartTask完成，避免中断过长
