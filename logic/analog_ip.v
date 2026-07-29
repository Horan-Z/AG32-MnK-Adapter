module analog_ip (
  output tri0        LED,
  input              i_kbd_CH374_INT_n,
  input              i_kbd_SPI_MISO,
  input              i_mouse_CH374_INT_n,
  input              i_mouse_SPI_MISO,
  output tri0        o_kbd_SPI_CS_n,
  output tri0        o_kbd_SPI_Clk,
  output tri0        o_kbd_SPI_MOSI,
  output tri0        o_mouse_SPI_CS_n,
  output tri0        o_mouse_SPI_Clk,
  output tri0        o_mouse_SPI_MOSI,
  input              sys_clock,
  input              bus_clock,
  input              resetn,
  input              stop,
  input       [1:0]  mem_ahb_htrans,
  input              mem_ahb_hready,
  input              mem_ahb_hwrite,
  input       [31:0] mem_ahb_haddr,
  input       [2:0]  mem_ahb_hsize,
  input       [2:0]  mem_ahb_hburst,
  input       [31:0] mem_ahb_hwdata,
  output tri1        mem_ahb_hreadyout,
  output tri0        mem_ahb_hresp,
  output tri0 [31:0] mem_ahb_hrdata,
  output tri0        slave_ahb_hsel,
  output tri1        slave_ahb_hready,
  input              slave_ahb_hreadyout,
  output tri0 [1:0]  slave_ahb_htrans,
  output tri0 [2:0]  slave_ahb_hsize,
  output tri0 [2:0]  slave_ahb_hburst,
  output tri0        slave_ahb_hwrite,
  output tri0 [31:0] slave_ahb_haddr,
  output tri0 [31:0] slave_ahb_hwdata,
  input              slave_ahb_hresp,
  input       [31:0] slave_ahb_hrdata,
  output tri0 [3:0]  ext_dma_DMACBREQ,
  output tri0 [3:0]  ext_dma_DMACLBREQ,
  output tri0 [3:0]  ext_dma_DMACSREQ,
  output tri0 [3:0]  ext_dma_DMACLSREQ,
  input       [3:0]  ext_dma_DMACCLR,
  input       [3:0]  ext_dma_DMACTC,
  output tri0 [3:0]  local_int
);

// --- 鼠标数据内部总线（原始字节透传，MCU 侧解析） ---
wire [63:0]          w_mouse_raw;
wire                 w_mouse_data_valid;

// --- 键盘数据内部总线 ---
wire [7:0]           w_kbd_modifiers;
wire [47:0]          w_kbd_keys;
wire                 w_kbd_data_valid;

parameter BUS_CLK_FREQ = 40_000_000;
parameter SPI_CLKS_PER_HALF_BIT = 2;
parameter ADDR_BITS = 16;	//真正使用到的地址位宽
parameter DATA_BITS = 32;	//真正使用到的数据位宽

//定义出：（ahb转apb后）apb的信号线
wire                 apb_psel;
wire                 apb_penable;
wire                 apb_pwrite;
wire [ADDR_BITS-1:0] apb_paddr;
wire [DATA_BITS-1:0] apb_pwdata;
wire [3:0]           apb_pstrb;
wire [2:0]           apb_pprot;
wire                 apb_pready  = 1'b1;
wire                 apb_pslverr = 1'b0;
wire [DATA_BITS-1:0] apb_prdata;

//apb的clock使用bus_clock。也可以使用其他clock。
//如果是慢速设备，请自行决定apb_clock的时钟频率。
assign apb_clock = bus_clock;

reg [15:0] por_cnt = 16'd0;
reg        soft_rst_n = 1'b0; // 初始为 0，即复位有效

always @(posedge bus_clock) begin
    if (por_cnt < 16'hFFFF) begin
        por_cnt <= por_cnt + 1'b1;
        soft_rst_n <= 1'b0; // 维持复位状态约 1.6ms (40MHz下)
    end else begin
        soft_rst_n <= resetn; // 脉冲结束后，将控制权交还给外部 resetn
    end
end

//关联ahb2apb模块。
//这里的#(ADDR_BITS, DATA_BITS) 会改写ahb2apb对应的宏的值。
ahb2apb #(ADDR_BITS, DATA_BITS) ahb2apb_inst(
  .reset        (!soft_rst_n                 ),
  .ahb_clock    (sys_clock                   ),
  .ahb_hmastlock(1'b0                        ),
  .ahb_htrans   (mem_ahb_htrans              ),
  .ahb_hsel     (1'b1                        ),
  .ahb_hready   (mem_ahb_hready              ),
  .ahb_hwrite   (mem_ahb_hwrite              ),
  .ahb_haddr    (mem_ahb_haddr[ADDR_BITS-1:0]),
  .ahb_hsize    (mem_ahb_hsize               ),
  .ahb_hburst   (mem_ahb_hburst              ),
  .ahb_hprot    (4'b0011                     ),
  .ahb_hwdata   (mem_ahb_hwdata              ),
  .ahb_hrdata   (mem_ahb_hrdata              ),
  .ahb_hreadyout(mem_ahb_hreadyout           ),
  .ahb_hresp    (mem_ahb_hresp               ),
  .apb_clock    (apb_clock                   ),
  .apb_psel     (apb_psel                    ),
  .apb_penable  (apb_penable                 ),
  .apb_pwrite   (apb_pwrite                  ),
  .apb_paddr    (apb_paddr                   ),
  .apb_pwdata   (apb_pwdata                  ),
  .apb_pstrb    (apb_pstrb                   ),
  .apb_pprot    (apb_pprot                   ),
  .apb_pready   (apb_pready                  ),
  .apb_pslverr  (apb_pslverr                 ),
  .apb_prdata   (apb_prdata                  )
);

//以上两部分，一组是mcu到ahb的信号(mem_ahb_xxxx)，一组是ahb转apb之后的信号（apb_xxxx)
//这里以下，可以直接使用apb_xxxx信号，遵循APB协议即可。

// ==========================================================
// 1. APB 寄存器地址映射
// ==========================================================
parameter ADDR_STATUS   = 12'h00; // 状态(W1C): bit0=键盘更新, bit1=鼠标更新
parameter ADDR_KBD_D1   = 12'h04; // 键盘数据1: { 32位Keys[31:0]}
parameter ADDR_KBD_D2   = 12'h08; // 键盘数据2: { 8位留空 | 16位Keys[47:32] | 8位修饰键 }
parameter ADDR_MOUSE_D1 = 12'h0C; // 鼠标数据1: raw 字节 0~3（MCU 按 profile 解析）
parameter ADDR_MOUSE_D2 = 12'h10; // 鼠标数据2: raw 字节 4~7（MCU 按 profile 解析）
parameter ADDR_MOUSE_CFG = 12'h14; // 鼠标配置(MCU可写): {8'b0, interval[7:0], useful_data_len[7:0], report_len[7:0]}

// ==========================================================
// 内部锁存与影子寄存器
// ==========================================================
reg [31:0] reg_status;
reg [31:0] reg_kbd_d1;
reg [31:0] reg_kbd_d2;
reg [31:0] reg_mouse_d1;
reg [31:0] reg_mouse_d2;

// MCU 可配置的鼠标参数（上电默认匹配当前已知鼠标）
reg [7:0]  cfg_mouse_report_len;
reg [7:0]  cfg_mouse_useful_data_len;
reg [7:0]  cfg_mouse_interval;

assign LED = (reg_kbd_d1 | reg_kbd_d2 | reg_mouse_d1 | reg_mouse_d2) == 32'b0;

// ==========================================================
// 数据锁存与 W1C(写1清零) 状态机 (加入影子锁存抗撕裂)
// ==========================================================
always @(posedge apb_clock) begin
    if (!resetn) begin
        reg_status   <= 32'b0;
        reg_kbd_d1   <= 32'b0;
        reg_kbd_d2   <= 32'b0;
        reg_mouse_d1 <= 32'b0;
        reg_mouse_d2 <= 32'b0;
        cfg_mouse_report_len      <= 8'd13;
        cfg_mouse_useful_data_len <= 8'd7;
        cfg_mouse_interval        <= 8'd1;
    end else begin
        // --- 1. 键盘数据锁存与状态更新 ---
        if (w_kbd_data_valid) begin
            reg_kbd_d1 <= w_kbd_keys[31:0];
            reg_kbd_d2 <= {8'h00, w_kbd_keys[47:32], w_kbd_modifiers};
            reg_status[0] <= 1'b1;
        end else if (apb_pwrite && apb_penable && apb_paddr[11:0] == ADDR_STATUS && apb_pwdata[0]) begin
            reg_status[0] <= 1'b0;
        end
        
        // --- 2. 鼠标数据处理（原始字节透传，MCU 侧按 profile 解析） ---
        // 步骤 A: 前端无条件捕获物理层脉冲，杜绝漏包
        if (w_mouse_data_valid) begin
            reg_mouse_d1 <= w_mouse_raw[31:0];
            reg_mouse_d2 <= w_mouse_raw[63:32];
            reg_status[1] <= 1'b1;
        end else if (apb_pwrite && apb_penable && apb_paddr[11:0] == ADDR_STATUS && apb_pwdata[1]) begin
            reg_status[1] <= 1'b0;
        end
        
        // --- 3. 鼠标配置寄存器写入（MCU 通过 APB 写入，receiver 下一轮询周期生效） ---
        if (apb_pwrite && apb_penable && apb_paddr[11:0] == ADDR_MOUSE_CFG) begin
            cfg_mouse_report_len       <= apb_pwdata[7:0];
            cfg_mouse_useful_data_len  <= apb_pwdata[15:8];
            cfg_mouse_interval         <= apb_pwdata[23:16];
        end
    end
end

// ==========================================================
// 3. APB 读操作 (组合逻辑地址译码)
// ==========================================================
reg [31:0] apb_read_data_out;

always @(*) begin
    if (!apb_pwrite && apb_penable) begin
        case (apb_paddr[11:0])
            ADDR_STATUS:   apb_read_data_out = reg_status;
            ADDR_KBD_D1:   apb_read_data_out = reg_kbd_d1;
            ADDR_KBD_D2:   apb_read_data_out = reg_kbd_d2;
            ADDR_MOUSE_D1: apb_read_data_out = reg_mouse_d1;
            ADDR_MOUSE_D2: apb_read_data_out = reg_mouse_d2;
            default:       apb_read_data_out = 32'b0;
        endcase
    end else begin
        apb_read_data_out = 32'b0;
    end
end

assign apb_prdata = apb_read_data_out;

// ==========================================================
// 4. 触发 MCU 高电平中断 (Local Int 0)
// ==========================================================
// 只要状态寄存器非空，中断线就一直拉高；配合 MCU 的 While 循环榨干数据
assign local_int[0] = (reg_status[1:0] != 2'b00);
assign local_int[3:1] = 3'b000; // 未使用的中断线拉低

// ********************************************************************
// ******************** 子模块例化 ************************************
// ********************************************************************

// 1. 例化鼠标接收器
Mouse_Receiver #(
    .SYS_CLK_FREQ           (BUS_CLK_FREQ),
    .SPI_CLKS_PER_HALF_BIT  (SPI_CLKS_PER_HALF_BIT)
) u_Mouse_Receiver (
    .clk                    (bus_clock),
    .rst_n                  (soft_rst_n),
    
    .o_SPI_Clk              (o_mouse_SPI_Clk),
    .i_SPI_MISO             (i_mouse_SPI_MISO),
    .o_SPI_MOSI             (o_mouse_SPI_MOSI),
    .o_SPI_CS_n             (o_mouse_SPI_CS_n),
    .i_CH374_INT_n          (i_mouse_CH374_INT_n),
    
    .i_cfg_report_len       (cfg_mouse_report_len),
    .i_cfg_useful_data_len  (cfg_mouse_useful_data_len),
    .i_cfg_interval         (cfg_mouse_interval),
    
    .o_mouse_raw            (w_mouse_raw),
    .o_mouse_data_valid     (w_mouse_data_valid)
);

// 2. 例化键盘接收器
Kbd_Receiver #(
    .SYS_CLK_FREQ           (BUS_CLK_FREQ),
    .SPI_CLKS_PER_HALF_BIT  (SPI_CLKS_PER_HALF_BIT)
) u_Kbd_Receiver (
    .clk                    (bus_clock),
    .rst_n                  (soft_rst_n),
    
    .o_SPI_Clk              (o_kbd_SPI_Clk),
    .i_SPI_MISO             (i_kbd_SPI_MISO),
    .o_SPI_MOSI             (o_kbd_SPI_MOSI),
    .o_SPI_CS_n             (o_kbd_SPI_CS_n),
    .i_CH374_INT_n          (i_kbd_CH374_INT_n),
    
    .o_kbd_modifiers        (w_kbd_modifiers),
    .o_kbd_keys             (w_kbd_keys),
    .o_kbd_data_valid       (w_kbd_data_valid)
);

endmodule
