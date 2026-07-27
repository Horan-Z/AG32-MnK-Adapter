# AGENTS.md

面向在本仓库工作的 AI / 自动化编码代理的开发指南：项目是什么、代码如何组织、各模块内部怎么运转、构建方式、以及改动时必须遵守的约定。

## 项目简介

**AG32-MnK-Adapter**：键盘 + 鼠标 → Xbox 360 手柄（XInput）转换器。

- 主控为 AG32VF303（AGRV2K 系列，**RISC-V MCU + 可编程逻辑 fabric 单芯片**），配合自制 CH374T 转接板（PCB 工程见 `LCEDA project/`）。
- 两颗 **CH374T**（USB Host 控制器）分别挂键盘和鼠标，通过 **SPI** 接入逻辑 fabric。
- 逻辑 fabric 驱动 CH374T 完成 USB 枚举与中断端点轮询，解析出键鼠数据后，经 **AHB→APB** 总线以内存映射寄存器交给 MCU。
- MCU 把键鼠换算成 XInput 报文，通过 **TinyUSB** 以 Xbox 360 手柄身份（VID `0x045E` / PID `0x028E`）上报主机。

## 目录结构

```
├── adapter.ve              # 时钟配置 + 逻辑引脚映射（SYSCLK/BUSCLK/HSECLK、SPI/INT/LED 引脚）
├── platformio.ini          # 构建配置：MCU 固件 + 逻辑综合入口（ip_name / logic_dir / board_logic.ve）
├── src/                    # MCU 固件（C）
│   ├── main.c              # 唯一业务文件：ISR 取数 + 键鼠→XInput 换算 + TinyUSB 类驱动
│   ├── XInputPad.h         # XInput 20 字节报文结构体 ReportDataXinput
│   ├── descriptor_xinput.h # Xbox 360 设备/配置/字符串描述符
│   ├── usb_descriptors.c   # TinyUSB 描述符回调
│   └── tusb_config.h       # TinyUSB 配置（纯设备、无 HID/CDC/MSC 类）
├── logic/                  # 可编程逻辑（Verilog）
│   ├── analog_ip.v         # 顶层胶水：桥接、寄存器堆、W1C 状态、中断、例化两个 receiver
│   ├── ahb2apb.v           # AHB-Lite → APB 桥（厂商提供，一般不改）
│   ├── Kbd_Receiver.v      # 键盘接收器（CH374 SPI 主机 + USB 枚举 + 轮询 + 解析）
│   ├── Mouse_Receiver.v    # 鼠标接收器（与键盘结构对称）
│   ├── SPI_Master.v        # 通用 SPI 主机 IP（CPOL=0/CPHA=0，MSB first）
│   ├── SPI_Master_With_Single_CS.v  # SPI_Master + CS 控制 + 多字节计数
│   └── adapter.v           # 工具生成的 wrapper（连接硬核 MCU 总线 ↔ analog_ip），勿手改
└── LCEDA project/          # 立创 EDA 的 CH374T 转接板 PCB 工程（*.epro，二进制）
```

> `.gitignore`：`logic/*` 默认忽略，**只有 `logic/*.v` 入库**（且排除 `*routed.v`）。`logic/` 下的 `.bin/.inc/.qsf/.tcl/db/...` 均为构建产物。

## 端到端数据流

```
键盘/鼠标 (USB)
 └─ CH374T (USB Host)
     └─ SPI (CLK/MOSI/MISO/CS + INT#)          logic/SPI_Master*.v
         └─ CH374-SPI 协议层 (req/resp + task)  logic/*_Receiver.v 第三部分
             └─ USB 主状态机 (枚举→轮询→解析)    logic/*_Receiver.v 第五/六部分
                 └─ 内部数据总线 w_kbd_* / w_mouse_*
                     └─ 寄存器堆 + W1C 状态 + local_int[0]   logic/analog_ip.v
                         └─ ahb2apb 桥 → 映射到 0x6000_0000
                             └─ LOCAL_INT0 ISR 排空数据       src/main.c
                                 └─ build_xinput_report() 换算
                                     └─ TinyUSB IN 端点 → 主机
```

### 寄存器映射（逻辑侧 `analog_ip.v` 的 `ADDR_*` ↔ MCU 侧 `main.c` 的 `ADDR_*` 宏，基址 `0x60000000`）

| 偏移   | 名称     | 内容 |
|--------|----------|------|
| `0x00` | STATUS   | **W1C**（写 1 清零）：bit0=键盘更新，bit1=鼠标更新。非 0 时 `local_int[0]` 持续拉高 |
| `0x04` | KBD_D1   | `keys[31:0]`（keycode 0~3） |
| `0x08` | KBD_D2   | `{8'b0, keys[47:32]（keycode 4~5）, modifier[7:0]}` |
| `0x0C` | MOUSE_D1 | `{y[15:0], x[15:0]}`（有符号增量） |
| `0x10` | MOUSE_D2 | `{8'b0, wheel[7:0], buttons[15:0]}` |

握手语义：数据 `valid` 脉冲锁存数据并置位 status；MCU 在 ISR 里 `while(status != 0)` 循环读取，并把读到的 status **原样写回**触发 W1C。键盘寄存器是绝对状态（覆盖式），鼠标 x/y/wheel 由 MCU 侧累加。

## 逻辑侧详解（logic/）

### analog_ip.v — 胶水层

- 例化 `ahb2apb` 桥（APB 时钟 = `bus_clock` 40MHz）与两个 receiver。
- 寄存器堆 + W1C 状态机；数据 `valid` 优先于 W1C 写入（不会清掉刚到的新数据）。
- 上电软复位：`por_cnt` 计满 `0xFFFF` 个 bus clock（≈1.6ms）后才释放 `soft_rst_n` 给各子模块。
- `local_int[0] = (status != 0)`（电平中断）；`local_int[3:1]` 恒 0。
- `LED`：键鼠均无数据时点亮（调试指示）。
- 关键参数：`BUS_CLK_FREQ=40_000_000`、`SPI_CLKS_PER_HALF_BIT=2`（→ SCK = 10MHz）。

### Kbd_Receiver.v / Mouse_Receiver.v — 核心（两者结构对称，约 95% 相同）

文件按"部分"分段，注释为中文：

1. **参数与状态定义**：主状态机 `S_*`、子步骤约定 `OP_ERR=0xD / OP_RESET=0xE / OP_FINISH=0xF`、CH374 寄存器地址 `REG_*` / `RAM_*`。
2. **SPI Master IP 例化**：`SPI_Master_With_Single_CS`（一次 CS 最多 16 字节）。
3. **CH374-SPI 协议转换层**：把"寄存器级请求"翻译成 SPI 字节流。
   - 请求接口：`spi_req_{valid,wr,addr,len,wdata[8]}` → 应答 `spi_resp_valid` + `spi_resp_rdata[8]`。
   - 三个 task 封装：`spi_write_reg`（单字节写）、`spi_write_multi`（写 8 字节 Setup 包到 RAM）、`spi_read_reg`（读 n 字节）。
   - CH374 帧格式：`[addr][opcode 0x80 写/0xC0 读][data...]`。
4. **辅助逻辑**：1ms 脉冲 `ms_pulse`、毫秒延时 `delay_ms_cnt`、1.5s 超时看门狗 `timeout_flag`、`INT#` 两级同步。
5. **主状态机转移**（组合逻辑 `next_state`）+ 6. **输出逻辑**（时序，用 `op_step` 跑每个状态内的多步子流程；状态切换时 `op_step` 自动归零）。

**枚举流程**（上电或 `S_ERROR` 后重走）：

```
S_IDLE → S_PWR_WAIT(50ms) → S_CHIP_RST(0x4C 复位/0x44 解除) → S_HOST_CONFIG
 → S_DEV_DETECT(轮询 HUB_SETUP[3]) → S_DEV_DEBOUNCE(200ms) → S_DEV_SPEED_CHECK(测速+设低速)
 → S_DEV_BUS_RST → S_DEV_WAIT_STABLE → S_HUB_EN
 → Set_Address(1) 四阶段(SETUP/WAIT/IN/WAIT_IN，NAK 时 OP_RESET 重试) → S_ENUM_UPDATE_ADDR
 → Set_Configuration(1) 四阶段
 → [仅键盘] Set_Protocol(0) 四阶段（强制 boot 协议）
 → S_POLL_*
```

**轮询流程**：`S_POLL_WAIT(INTERVAL ms) → S_POLL_EP1_ISSUE(IN 令牌 0x91 + 按 toggle 发 DATA0/1) → S_POLL_EP1_WAIT_INT(等 INT#，读 USB_STATUS 判 PID：期望 DATA→读包 / NAK→下轮 / 非期望 DATA→丢弃 / 其他→S_ERROR) → S_POLL_EP1_READ(校验长度→读 RAM_HOST_RECV→解析输出)`。

- `ep1_in_toggle` 本地跟踪 DATA0/DATA1，`ep1_expected_pid` 据此过滤重复包。
- 任何阶段 `timeout_flag` 或异常 PID → `S_ERROR`：输出清零 + `valid` 脉冲一次，250ms 后回 `S_IDLE` 重新枚举。

**设备相关参数**（换不同键鼠时可能需要调）：

| | Kbd_Receiver | Mouse_Receiver |
|---|---|---|
| `REPORT_LEN`（长度校验） | 8 | 13 |
| `USEFUL_DATA_LEN`（实读字节） | 8 | 7 |
| `INTERVAL`（轮询周期） | 1ms | 1ms |
| 输出 | `o_kbd_modifiers[7:0]` + `o_kbd_keys[47:0]`（6 keycode） | `o_mouse_buttons[15:0]`、`o_mouse_x/y`（signed16）、`o_mouse_wheel`（signed8） |
| 解析位置 | `S_POLL_EP1_READ` 内位拼接 | 同左 |

### 通用 / 厂商模块（一般不改）

- `SPI_Master.v` / `SPI_Master_With_Single_CS.v`：通用 SPI 主机 IP，`CLKS_PER_HALF_BIT` 决定 SCK 频率。
- `ahb2apb.v`：厂商 AHB-Lite→APB 桥。
- `adapter.v`、`*.qsf`、`af_*.tcl`、`*.bin/.inc`：工具生成物。

## MCU 固件详解（src/main.c）

### 中断取数

`LOCAL_INT0_isr`：循环读 STATUS，按位分别解析 KBD_D1/D2（覆盖 `local_in` 的键码/修饰键）与 MOUSE_D1/D2（**累加** dx/dy/wheel，覆盖 buttons），最后把 status 原样写回清中断。`main()` 中通过 `INT_EnableIntLocal/EnableIRQ(LOCAL_INT0_IRQn, PLIC_MAX_PRIORITY)` 使能。

### 换算流水线（`build_xinput_report`，每次 USB 传输完成调用一次）

1. **修饰键**：Ctrl→B，Alt→DOWN，Shift→L3。
2. **按键 LUT**：`s_key_lut_btn[256]`（HID keycode → Xbox 按键位，如 Q→LB、Space→A、Tab/Esc→START）；`s_key_lut_wasd[256]`（WASD → 4bit 方向位）。
3. **左摇杆（移动）**：`s_lx_lut/s_ly_lut[16]` 把 WASD 方向位映射为摇杆值，含对角线归一化（±23170 ≈ 32767/√2）。
4. **右摇杆（视角）**：鼠标 dx/dy →
   - `apply_dynamic_curve()`：21 点分段 LUT 响应曲线，线性↔二次按 `sag_level` 混合；腰射（HIP，scale 40）/开镜（ADS，scale 25，右键按下时）两套参数，由 `update_mouse_curve()` 在启动时生成（当前 `sag_level=50`）。
   - EMA 平滑：定点 `<<8`，`EMA_SHIFT=1`。
   - 左键：RT=255 + 后坐力补偿 `recoil_offset=-250`（ADS 时 ×2）+ 抖动 `jitter=±990`（每 15 tick 翻向）。
   - `square_to_circle_int()`：方形→圆形映射（`isqrt` + 先乘后除，平方和用 `uint32_t` 防溢出）。
5. **舔包模式**（鼠标侧键 bit4 按住）：鼠标改映射到**左**摇杆（scale 180 + 独立 EMA），WASD 改到右摇杆，滚轮置满右摇杆 Y，左/右键→A/X。
6. **滚轮**：非 0 → Y 按键；MCU 侧每报告 ±1 衰减回 0。

### USB 侧

- 自定义 TinyUSB 类驱动 `xinput_driver`（经 `usbd_app_driver_get_cb` 注册）：`xinput_open` 从配置描述符抓取 IN/OUT 端点号；`xinput_xfer_cb` 在每次传输完成后于全局中断保护下做 `local_in` 快照（并清零 dx/dy、衰减 wheel）→ 构建报文 → 经 `usbd_edpt_claim/xfer/release` 回发。报告节奏由主机 IN 轮询驱动。
- `main()` 在枚举完成后发一帧全零初始报文。
- 报文格式：`XInputPad.h` 的 `ReportDataXinput`（20 字节，`rsize=0x14`）；描述符伪装 Xbox 360 手柄（`descriptor_xinput.h`）。

## 构建 / 烧录

PlatformIO，`platform = AgRV`，板 `agrv2k_303`，framework `agrv_sdk, agrv_tinyusb`。逻辑综合被集成进构建流程：`ip_name = analog_ip`、`logic_dir = logic`、`board_logic.ve = adapter.ve`、器件 `AGRV2KL48`。

```bash
pio run                 # 构建（default_envs = debug）
pio run -e release      # release 构建
pio run -t upload       # 烧录（cmsis-dap-openocd）
pio device monitor      # 串口监视
```

如需串口烧录 / RTT 监视，按 `platformio.ini` 注释替换 `setup_upload` / `setup_monitor`。

## 开发约定与注意事项

- **注释语言为中文**，请保持一致。
- 逻辑侧只提交 `logic/*.v`；手改只碰 `analog_ip.v` / `*_Receiver.v` / `SPI_Master*.v` / `ahb2apb.v`，**不要**改 `adapter.v` 及 `.qsf/.tcl/.bin/.inc` 等生成物。
- **寄存器映射双处同步**：`analog_ip.v` 的 `ADDR_*` 与 `main.c` 的 `ADDR_*` 宏（含位域布局）必须一致。
- **引脚三处对应**：`adapter.ve`（引脚名↔PIN 号）、`analog_ip.v`（端口名）、PCB（`LCEDA project/`）。
- **W1C 语义不可破坏**：ISR 必须把读到的 status 原样写回；改状态寄存器/ISR 时保证"数据 valid 优先于 W1C"。
- **两个 receiver 对称**：SPI 协议层、枚举、轮询、超时逻辑几乎逐行相同，改一处通常要同步另一处；差异仅在 Set_Protocol 状态群（仅键盘）、`REPORT_LEN/USEFUL_DATA_LEN`、`S_POLL_EP1_READ` 的解析位拼接、输出端口。
- **receiver 内新增 SPI 交互**遵循现有范式：状态内用 `op_step` 编号子步骤，`spi_req_ready && !spi_req_valid` 时发请求、`spi_resp_valid` 时收应答，完成置 `OP_FINISH`，异常置 `OP_ERR`/走 `timeout_flag`。
- **换键鼠设备**：确认其 boot 报文长度，调 `REPORT_LEN/USEFUL_DATA_LEN` 并核对解析拼接；低速/全速由枚举自动测速处理，无需改。
- 手感参数（键位 LUT、scale、sag、jitter、recoil）集中在 `main.c` 前部与 `build_xinput_report` 内，调参优先改这些常量。

## 常见改动速查

| 想做的事 | 改哪里 |
|---|---|
| 增删按键映射 | `main.c`：`s_key_lut_btn`（HID keycode → Xbox 按键位）、`s_key_lut_wasd` |
| 调鼠标灵敏度/曲线 | `main.c`：`XINPUT_MOUSE_TO_STICK_SCALE_{HIP,ADS,LOOT}`、`update_mouse_curve(sag_level)` |
| 调压枪/抖动/平滑 | `main.c` `build_xinput_report`：`recoil_offset`、`current_jitter_amp`、`EMA_SHIFT` |
| 改轮询周期 | `analog_ip.v` 例化处的 `INTERVAL`（经参数传入 receiver） |
| 改 SPI 频率 | `analog_ip.v` 的 `SPI_CLKS_PER_HALF_BIT`（SCK = BUSCLK / (4×该值)） |
| 换键鼠设备 | receiver 的 `REPORT_LEN/USEFUL_DATA_LEN` + `S_POLL_EP1_READ` 解析 |
| 改引脚 | `adapter.ve`（并核对 `analog_ip.v` 端口与 PCB） |
| 改寄存器接口 | `analog_ip.v` 寄存器堆 + `main.c` 宏与 ISR 解析，同步改 |
| 改 USB 身份/报文 | `descriptor_xinput.h`、`XInputPad.h`（注意主机按 VID/PID 缓存驱动） |
