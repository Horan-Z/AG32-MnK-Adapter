`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 适配说明：CH374T芯片、3.3V供电、无HUB、单鼠标设备
//////////////////////////////////////////////////////////////////////////////////
module Mouse_Receiver
#(
    // 系统参数
    parameter   SYS_CLK_FREQ            = 40_000_000,    // 系统时钟频率，单位Hz
    parameter   SPI_CLKS_PER_HALF_BIT   = 2,
    parameter   INTERVAL                = 8'd01,
    parameter   REPORT_LEN              = 8'd13,
    parameter   USEFUL_DATA_LEN         = 8'd7
)
(
    // 系统时钟与复位
    input                   clk,                    // 系统时钟50MHz
    input                   rst_n,                  // 系统复位，低电平有效
    
    // CH374T SPI接口
    output                  o_SPI_Clk,
    input                   i_SPI_MISO,
    output                  o_SPI_MOSI,
    output                  o_SPI_CS_n,
    
    input                   i_CH374_INT_n,
    
    output reg        [15:0] o_mouse_buttons,
    output reg signed [15:0] o_mouse_x,
    output reg signed [15:0] o_mouse_y,
    output reg signed [7:0]  o_mouse_wheel,
    output reg               o_mouse_data_valid
);

// ********************************************************************
// ******************** 第一部分：内部信号与参数定义 ********************
// ********************************************************************

// CH374主状态机定义
localparam  S_IDLE              = 8'h00;
localparam  S_PWR_WAIT          = 8'h01; // 上电等待 50ms
localparam  S_CHIP_RST          = 8'h02;
localparam  S_HOST_CONFIG       = 8'h03;
localparam  S_DEV_DETECT        = 8'h04;
localparam  S_DEV_DEBOUNCE      = 8'h05;
localparam  S_DEV_SPEED_CHECK   = 8'h06;
localparam  S_DEV_BUS_RST       = 8'h07;
localparam  S_DEV_WAIT_STABLE   = 8'h08;
localparam  S_HUB_EN            = 8'h09;

// ----- 精简枚举状态群 -----
localparam  S_ENUM_SET_ADDR_SETUP   = 8'h0A; // 发送 Set_Address 设定包
localparam  S_ENUM_SET_ADDR_WAIT    = 8'h0B; // 等待 Setup 传输完成
localparam  S_ENUM_SET_ADDR_IN      = 8'h0C; // 发送 IN 令牌(状态阶段)
localparam  S_ENUM_SET_ADDR_WAIT_IN = 8'h0D; // 等待 IN 传输完成
localparam  S_ENUM_UPDATE_ADDR      = 8'h0E; // 更新CH374内部的地址寄存器
localparam  S_ENUM_SET_CONF_SETUP   = 8'h0F; // 发送 Set_Configuration 设定包
localparam  S_ENUM_SET_CONF_WAIT    = 8'h10; // 等待 Setup 传输完成
localparam  S_ENUM_SET_CONF_IN      = 8'h11; // 发送 IN 令牌(状态阶段)
localparam  S_ENUM_SET_CONF_WAIT_IN = 8'h12; // 等待 IN 传输完成

// ----- 轮询状态群 -----
localparam  S_POLL_WAIT             = 8'h13; // 等待X ms定时器
localparam  S_POLL_EP1_ISSUE        = 8'h14; // 发送IN令牌+启动传输
localparam  S_POLL_EP1_WAIT_INT     = 8'h15; // 等待INT#低电平（非脉冲模式）
localparam  S_POLL_EP1_READ         = 8'h16; // 读取鼠标数据

localparam  S_ERROR                 = 8'hFF;

localparam  OP_ERR              = 4'hD;
localparam  OP_RESET            = 4'hE;
localparam  OP_FINISH           = 4'hF;
localparam  SPI_READ            = 1'b0;
localparam  SPI_WRITE           = 1'b1;

// CH374寄存器地址定义
localparam   REG_SYS_CTRL        = 8'h05;
localparam   REG_USB_SETUP       = 8'h06;
localparam   REG_HUB_SETUP       = 8'h02;
localparam   REG_INTER_EN        = 8'h07;
localparam   REG_INTER_FLAG      = 8'h09;
localparam   REG_USB_STATUS      = 8'h0A;
localparam   REG_USB_ADDR        = 8'h08;
localparam   REG_USB_LENGTH      = 8'h0B;
localparam   REG_USB_H_TOKEN     = 8'h0D;
localparam   REG_USB_H_CTRL      = 8'h0E;
localparam   RAM_HOST_TRAN       = 8'h40;
localparam   RAM_HOST_RECV       = 8'hC0;

localparam   MS_PULSE_CNT        = SYS_CLK_FREQ / 1000; // 1ms定时计数阈值

// SPI传输控制信号
reg         spi_req_valid;
reg         spi_req_wr;
reg  [7:0]  spi_req_addr;
reg  [7:0]  spi_req_len;
reg  [7:0]  spi_req_wdata[7:0];
wire        spi_req_ready;
wire        spi_resp_valid;
wire [7:0]  spi_resp_rdata[7:0];

// 系统辅助信号
reg  [15:0] timer_cnt;
reg         ms_pulse;
reg  [10:0] timeout_cnt;
reg         timeout_flag;
reg  [7:0]  delay_ms_cnt;
wire        delay_done = (delay_ms_cnt == 8'd0);

reg  [7:0]  curr_state;
reg  [7:0]  next_state;
reg         dev_is_low_speed;
reg  [3:0]  op_step;

reg         ep1_in_toggle;        // DATA0/DATA1 同步翻转标志
wire [3:0]  ep1_expected_pid = ep1_in_toggle ? 4'hB : 4'h3; // 期望 DATA1 / DATA0


// ********************************************************************
// ******************** 第二部分：SPI Master IP例化 ********************
// ********************************************************************

wire        spi_ip_tx_ready;
reg         spi_ip_tx_dv;
reg  [7:0]  spi_ip_tx_byte;
reg  [4:0]  spi_ip_tx_count;
wire        spi_ip_rx_dv;
wire [7:0]  spi_ip_rx_byte;
wire [4:0]  spi_ip_rx_count;

SPI_Master_With_Single_CS
#(
    .CLKS_PER_HALF_BIT  (SPI_CLKS_PER_HALF_BIT),
    .MAX_BYTES_PER_CS   (16),
    .CS_INACTIVE_CLKS   (5)
)
u_spi_master_mouse
(
    .i_Rst_L            (rst_n),
    .i_Clk              (clk),
    .i_TX_Count         (spi_ip_tx_count),
    .i_TX_Byte          (spi_ip_tx_byte),
    .i_TX_DV            (spi_ip_tx_dv),
    .o_TX_Ready         (spi_ip_tx_ready),
    .o_RX_Count         (spi_ip_rx_count),
    .o_RX_DV            (spi_ip_rx_dv),
    .o_RX_Byte          (spi_ip_rx_byte),
    .o_SPI_Clk          (o_SPI_Clk),
    .i_SPI_MISO         (i_SPI_MISO),
    .o_SPI_MOSI         (o_SPI_MOSI),
    .o_SPI_CS_n         (o_SPI_CS_n)
);

// ********************************************************************
// ******************** 第三部分：CH374-SPI协议转换层 *****************
// ********************************************************************

localparam  SPI_ST_IDLE   = 3'd0;
localparam  SPI_ST_ADDR   = 3'd1;
localparam  SPI_ST_OP     = 3'd2;
localparam  SPI_ST_DATA   = 3'd3;
localparam  SPI_ST_WAIT   = 3'd4;
localparam  SPI_ST_DONE   = 3'd5;

reg  [2:0]  spi_state;
reg  [4:0]  spi_byte_cnt;
reg  [7:0]  spi_rdata_buf[7:0];
reg         spi_resp_valid_reg;

assign spi_req_ready = (spi_state == SPI_ST_IDLE);
assign spi_resp_valid = spi_resp_valid_reg;
wire [7:0] ch374_opcode = spi_req_wr ? 8'h80 : 8'hC0;

reg spi_is_read_op;

always @(posedge clk) begin
    if(!rst_n) begin
        spi_state <= SPI_ST_IDLE;
        spi_ip_tx_dv <= 1'b0;
        spi_ip_tx_byte <= 8'd0;
        spi_ip_tx_count <= 5'd0;
        spi_byte_cnt <= 5'd0;
        spi_resp_valid_reg <= 1'b0;
        spi_is_read_op <= 1'b0;
    end else begin
        spi_ip_tx_dv <= 1'b0;
        spi_resp_valid_reg <= 1'b0;
        
        // 独立的数据捕获逻辑
        if (spi_ip_rx_dv && spi_is_read_op) begin
            if (spi_ip_rx_count >= 5'd2 && (spi_ip_rx_count - 5'd2) < 8) begin
                spi_rdata_buf[spi_ip_rx_count - 5'd2] <= spi_ip_rx_byte;
            end
        end
        
        case(spi_state)
            SPI_ST_IDLE: begin
                if(spi_req_valid) begin
                    spi_state <= SPI_ST_ADDR;
                    spi_ip_tx_count <= spi_req_len + 5'd2;
                    spi_byte_cnt <= 5'd0;
                    spi_is_read_op <= ~spi_req_wr;
                    if (~spi_req_wr) begin  // 读操作时清零缓冲区
                        spi_rdata_buf[0] <= 8'd0;
                        spi_rdata_buf[1] <= 8'd0;
                        spi_rdata_buf[2] <= 8'd0;
                        spi_rdata_buf[3] <= 8'd0;
                        spi_rdata_buf[4] <= 8'd0;
                        spi_rdata_buf[5] <= 8'd0;
                        spi_rdata_buf[6] <= 8'd0;
                        spi_rdata_buf[7] <= 8'd0;
                    end
                end
            end
            SPI_ST_ADDR: begin
                if(spi_ip_tx_ready && !spi_ip_tx_dv) begin
                    spi_ip_tx_byte <= spi_req_addr;
                    spi_ip_tx_dv <= 1'b1;
                    spi_state <= SPI_ST_OP;
                end
            end
            SPI_ST_OP: begin
                if(spi_ip_tx_ready && !spi_ip_tx_dv) begin
                    spi_ip_tx_byte <= ch374_opcode;
                    spi_ip_tx_dv <= 1'b1;
                    spi_state <= SPI_ST_DATA;
                end
            end
            SPI_ST_DATA: begin
                if(spi_ip_tx_ready && !spi_ip_tx_dv && spi_byte_cnt < spi_req_len) begin
                    spi_ip_tx_byte <= spi_req_wr ? spi_req_wdata[spi_byte_cnt] : 8'hFF;
                    spi_ip_tx_dv <= 1'b1;
                    spi_byte_cnt <= spi_byte_cnt + 5'd1;
                end else if(spi_byte_cnt == spi_req_len) begin
                    spi_state <= SPI_ST_WAIT;
                end
            end
            SPI_ST_WAIT: begin
                if(o_SPI_CS_n) begin 
                    spi_state <= SPI_ST_DONE;
                end
            end
            SPI_ST_DONE: begin
                spi_resp_valid_reg <= 1'b1;
                spi_state <= SPI_ST_IDLE;
            end
            default: spi_state <= SPI_ST_IDLE;
        endcase
    end
end

genvar i;
generate
    for(i = 0; i < USEFUL_DATA_LEN; i = i + 1) begin : gen_mouse_data
        assign spi_resp_rdata[i] = spi_rdata_buf[i];
    end
endgenerate

// ********************************************************************
// ******************** 第四部分：系统辅助模块与Task *******************
// ********************************************************************

// 1ms 脉冲生成
always @(posedge clk) begin
    if(!rst_n) begin
        timer_cnt <= 16'd0;
        ms_pulse <= 1'b0;
    end else if(timer_cnt == MS_PULSE_CNT - 1) begin
        timer_cnt <= 16'd0;
        ms_pulse <= 1'b1;
    end else begin
        timer_cnt <= timer_cnt + 16'd1;
        ms_pulse <= 1'b0;
    end
end

// 超时监控
always @(posedge clk) begin
    if(!rst_n) begin
        timeout_cnt <= 11'd0;
        timeout_flag <= 1'b0;
    end else if(curr_state != next_state || curr_state == S_ERROR || curr_state == S_POLL_WAIT || curr_state == S_DEV_DETECT || curr_state == S_POLL_EP1_WAIT_INT) begin
        timeout_cnt <= 11'd0;
        timeout_flag <= 1'b0;
    end else if(ms_pulse) begin
        if(timeout_cnt >= 11'd1500) begin // 1.5s超时
            timeout_flag <= 1'b1;
        end else begin
            timeout_cnt <= timeout_cnt + 11'd1;
            timeout_flag <= 1'b0;
        end
    end
end

// 中断同步信号
reg         ch374_int_n_sync1;
reg         ch374_int_n_sync2;
wire        ch374_int_low = ~ch374_int_n_sync2; // 同步后的低电平有效标志

always @(posedge clk) begin
    if(!rst_n) begin
        ch374_int_n_sync1 <= 1'b1;
        ch374_int_n_sync2 <= 1'b1;
    end else begin
        ch374_int_n_sync1 <= i_CH374_INT_n; // 第一级：可能亚稳态，但只在内部
        ch374_int_n_sync2 <= ch374_int_n_sync1; // 第二级：几乎100%稳定
    end
end

// ========== SPI 操作 Task ==========

// SPI 单字节写寄存器
task spi_write_reg;
    input [7:0] addr;
    input [7:0] data;
    begin
        spi_req_wr   <= SPI_WRITE;
        spi_req_addr <= addr;
        spi_req_len  <= 8'd1;
        spi_req_wdata[0] <= data;
        spi_req_valid <= 1'b1;
    end
endtask

// SPI 多字节写 (用于写Setup包到RAM)
task spi_write_multi;
    input [7:0]  addr;
    input [63:0] data; 
    begin
        spi_req_wr   <= SPI_WRITE;
        spi_req_addr <= addr;
        spi_req_len  <= 8'd8;
        {spi_req_wdata[7], spi_req_wdata[6], spi_req_wdata[5], spi_req_wdata[4],
         spi_req_wdata[3], spi_req_wdata[2], spi_req_wdata[1], spi_req_wdata[0]} <= data;
        spi_req_valid <= 1'b1;
    end
endtask

// SPI 读寄存器/内存
task spi_read_reg;
    input [7:0] addr;
    input [7:0] len;
    begin
        spi_req_wr   <= SPI_READ;
        spi_req_addr <= addr;
        spi_req_len  <= len;
        spi_req_valid <= 1'b1;
    end
endtask

// ********************************************************************
// ******************** 第五部分：主状态机转移逻辑 ********************
// ********************************************************************

always @(posedge clk) begin
    if(!rst_n) curr_state <= S_IDLE;
    else curr_state <= next_state;
end

always @(*) begin
    case(curr_state)
        S_IDLE:              next_state = S_PWR_WAIT;
        S_PWR_WAIT:          next_state = (delay_done && op_step == OP_FINISH) ? S_CHIP_RST             :  S_PWR_WAIT;
        S_CHIP_RST:          next_state = (delay_done && op_step == OP_FINISH) ? S_HOST_CONFIG          : (timeout_flag ? S_ERROR : S_CHIP_RST);
        S_HOST_CONFIG:       next_state = (delay_done && op_step == OP_FINISH) ? S_DEV_DETECT           : (timeout_flag ? S_ERROR : S_HOST_CONFIG);
        S_DEV_DETECT:        next_state = (delay_done && op_step == OP_FINISH) ? S_DEV_DEBOUNCE         :  S_DEV_DETECT;
        S_DEV_DEBOUNCE:      next_state = (delay_done && op_step == OP_FINISH) ? S_DEV_SPEED_CHECK      : ((op_step == OP_ERR) ? S_DEV_DETECT : (timeout_flag ? S_ERROR : S_DEV_DEBOUNCE));
        S_DEV_SPEED_CHECK:   next_state = (delay_done && op_step == OP_FINISH) ? S_DEV_BUS_RST          : (timeout_flag ? S_ERROR : S_DEV_SPEED_CHECK);
        S_DEV_BUS_RST:       next_state = (delay_done && op_step == OP_FINISH) ? S_DEV_WAIT_STABLE      : (timeout_flag ? S_ERROR : S_DEV_BUS_RST);
        S_DEV_WAIT_STABLE:   next_state = (delay_done && op_step == OP_FINISH) ? S_HUB_EN               : (timeout_flag ? S_ERROR : S_DEV_WAIT_STABLE);
        S_HUB_EN:            next_state = (delay_done && op_step == OP_FINISH) ? S_ENUM_SET_ADDR_SETUP  : (timeout_flag ? S_ERROR : S_HUB_EN);
        
        // ----- 枚举状态群：Set Address -----
        S_ENUM_SET_ADDR_SETUP:   next_state = (op_step == OP_FINISH) ? S_ENUM_SET_ADDR_WAIT             : (timeout_flag ? S_ERROR : S_ENUM_SET_ADDR_SETUP);
        S_ENUM_SET_ADDR_WAIT:    next_state = (op_step == OP_FINISH) ? S_ENUM_SET_ADDR_IN               : (timeout_flag ? S_ERROR : S_ENUM_SET_ADDR_WAIT);
        S_ENUM_SET_ADDR_IN:      next_state = (op_step == OP_FINISH) ? S_ENUM_SET_ADDR_WAIT_IN          : (timeout_flag ? S_ERROR : S_ENUM_SET_ADDR_IN);
        S_ENUM_SET_ADDR_WAIT_IN: next_state = (op_step == OP_FINISH) ? S_ENUM_UPDATE_ADDR               : ((op_step == OP_RESET) ? S_ENUM_SET_ADDR_IN : (timeout_flag ? S_ERROR : S_ENUM_SET_ADDR_WAIT_IN));
        // ----- 更新地址寄存器 -----
        S_ENUM_UPDATE_ADDR:      next_state = (op_step == OP_FINISH) ? S_ENUM_SET_CONF_SETUP            : (timeout_flag ? S_ERROR : S_ENUM_UPDATE_ADDR);

        // ----- 枚举状态群：Set Configuration -----
        S_ENUM_SET_CONF_SETUP:   next_state = (op_step == OP_FINISH) ? S_ENUM_SET_CONF_WAIT             : (timeout_flag ? S_ERROR : S_ENUM_SET_CONF_SETUP);
        S_ENUM_SET_CONF_WAIT:    next_state = (op_step == OP_FINISH) ? S_ENUM_SET_CONF_IN               : (timeout_flag ? S_ERROR : S_ENUM_SET_CONF_WAIT);
        S_ENUM_SET_CONF_IN:      next_state = (op_step == OP_FINISH) ? S_ENUM_SET_CONF_WAIT_IN          : (timeout_flag ? S_ERROR : S_ENUM_SET_CONF_IN);
        S_ENUM_SET_CONF_WAIT_IN: next_state = (op_step == OP_FINISH) ? S_POLL_WAIT : ((op_step == OP_RESET) ? S_ENUM_SET_CONF_IN : (timeout_flag ? S_ERROR : S_ENUM_SET_CONF_WAIT_IN));
        
        // ----- 轮询状态群转移逻辑 -----
        S_POLL_WAIT:             next_state = (delay_done && op_step == OP_FINISH)  ? S_POLL_EP1_ISSUE      : S_POLL_WAIT;
        S_POLL_EP1_ISSUE:        next_state = (op_step == OP_FINISH)                ? S_POLL_EP1_WAIT_INT   : (timeout_flag ? S_POLL_WAIT : S_POLL_EP1_ISSUE);
        S_POLL_EP1_WAIT_INT:     next_state = (op_step == OP_ERR || timeout_flag)   ? S_ERROR               :
                                              (op_step == OP_RESET)                 ? S_POLL_WAIT           : 
                                              (op_step == OP_FINISH)                ? S_POLL_EP1_READ       :  S_POLL_EP1_WAIT_INT;
        S_POLL_EP1_READ:         next_state = (op_step == OP_FINISH)                ? S_POLL_WAIT           : (timeout_flag ? S_POLL_WAIT : S_POLL_EP1_READ);
        
        S_ERROR:             next_state = (delay_done && op_step == 4'd1) ? S_IDLE : S_ERROR;
        default:             next_state = S_IDLE;
    endcase
end

// ********************************************************************
// ******************** 第六部分：主状态机输出逻辑 ********************
// ********************************************************************

always @(posedge clk) begin
    if(!rst_n) begin
        spi_req_valid <= 1'b0;
        spi_req_wr <= 1'b0;
        spi_req_addr <= 8'd0;
        spi_req_len <= 8'd0;
        op_step <= 4'd0;
        ep1_in_toggle <= 1'b0;
        dev_is_low_speed <= 1'b0;
        o_mouse_buttons <= 16'd0;
        o_mouse_x <= 16'd0;
        o_mouse_y <= 16'd0;
        o_mouse_wheel <= 8'd0;
        o_mouse_data_valid <= 1'b0;
        delay_ms_cnt <= 8'd0;
    end else begin
        spi_req_valid      <= 1'b0; // 默认拉低，只有在调用task时拉高
        o_mouse_data_valid <= 1'b0;
        
        if (ms_pulse && delay_ms_cnt > 0) begin
            delay_ms_cnt <= delay_ms_cnt - 8'd1;
        end
        
        case(curr_state)
            S_PWR_WAIT: begin
                if (op_step == 4'd0) begin
                    delay_ms_cnt <= 8'd50; // 等待 50ms
                    op_step <= OP_FINISH;
                end
            end

            S_CHIP_RST: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    // 1. 发送带复位的指令 0x4C (Bit 3 = 1)
                    spi_write_reg(REG_SYS_CTRL, 8'h4C);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    delay_ms_cnt <= 8'd50; // 等待 50ms 让复位充分生效
                    op_step <= 4'd2;
                end else if(delay_done && spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    // 2. 发送解除复位的指令 0x44 (Bit 3 = 0, 保持 OE 和 POWER 配置)
                    spi_write_reg(REG_SYS_CTRL, 8'h44);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    delay_ms_cnt <= 8'd10; // 退出复位后稍作稳定
                    op_step <= OP_FINISH;
                end
            end
            
            S_HOST_CONFIG: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_write_reg(REG_USB_SETUP, 8'hC0);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_HUB_SETUP, 8'h00);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= 4'd4;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd4) begin
                    spi_write_reg(REG_INTER_EN, 8'hF1);
                    op_step <= 4'd5;
                end else if(spi_resp_valid && op_step == 4'd5) begin
                    op_step <= OP_FINISH;
                end
            end 
            
            S_DEV_DETECT: begin
                if(delay_done && spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_HUB_SETUP, 8'd1);
                    delay_ms_cnt <= 8'd10;
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= spi_resp_rdata[0][3] ? OP_FINISH : 4'd0;
                end
            end

            S_DEV_DEBOUNCE: begin
                if (op_step == 4'd0) begin
                    delay_ms_cnt <= 8'd200; // 等待 200ms 消除物理插拔抖动
                    op_step <= 4'd1;
                end else if (delay_done && spi_req_ready && !spi_req_valid && op_step == 4'd1) begin
                    spi_read_reg(REG_HUB_SETUP, 8'd1); // 再次读状态确认
                    op_step <= 4'd2;
                end else if (spi_resp_valid && op_step == 4'd2) begin
                    // 依然为 1，防抖通过(OP_FINISH)；如果变成 0，视作抖动干扰，抛出错误标志(OP_ERR)退回检测状态
                    op_step <= spi_resp_rdata[0][3] ? OP_FINISH : OP_ERR;
                end
            end
            
            S_DEV_SPEED_CHECK: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_INTER_FLAG, 8'd1);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    dev_is_low_speed <= ~spi_resp_rdata[0][7];
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_HUB_SETUP, dev_is_low_speed ? 8'h04 : 8'h00);
                    delay_ms_cnt <= 8'd20;
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= OP_FINISH;
                end
            end
            
            S_DEV_BUS_RST: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_write_reg(REG_HUB_SETUP, dev_is_low_speed ? 8'h06 : 8'h02);
                    delay_ms_cnt <= 8'd20; 
                    op_step <= 4'd1;
                end else if(delay_done && spi_req_ready && !spi_req_valid && op_step == 4'd1) begin
                    spi_write_reg(REG_HUB_SETUP, dev_is_low_speed ? 8'h04 : 8'h00);
                    delay_ms_cnt <= 8'd10;
                    op_step <= 4'd2;
                end else if(spi_resp_valid && op_step == 4'd2) begin
                    op_step <= OP_FINISH;
                end
            end
            
            S_DEV_WAIT_STABLE: begin
                if(delay_done && spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_HUB_SETUP, 8'd1);
                    delay_ms_cnt <= 8'd10;
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= spi_resp_rdata[0][3] ? OP_FINISH : 4'd0;
                end
            end
            
            S_HUB_EN: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_HUB_SETUP, 8'd1);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_HUB_SETUP, spi_resp_rdata[0] | 8'h01);
                    delay_ms_cnt <= 8'd100;
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= OP_FINISH;
                end
            end

            // ---------------- USB枚举：Set Address (分配地址 1) ----------------
            S_ENUM_SET_ADDR_SETUP: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    // 步骤1：写入8字节标准Setup包。注意位宽补全为 64'd0 
                    // 字节顺序从高到低拼接：wdata[7], wdata[6]...wdata[0]
                    spi_write_multi(RAM_HOST_TRAN, {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h01, 8'h05, 8'h00});
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    // 步骤2：设置设备速度
                    spi_write_reg(REG_USB_SETUP, dev_is_low_speed ? 8'hE0 : 8'hC0);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= 4'd4;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd4) begin
                    // 步骤3：设置发送长度
                    spi_write_reg(REG_USB_LENGTH, 8'd8);
                    op_step <= 4'd5;
                end else if(spi_resp_valid && op_step == 4'd5) begin
                    op_step <= 4'd6;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd6) begin
                    // 步骤3：下发 SETUP 令牌到 EP0 (PID=0xD, EP=0x0 -> 0xD0) 
                    spi_write_reg(REG_USB_H_TOKEN, 8'hD0);
                    op_step <= 4'd7;
                end else if(spi_resp_valid && op_step == 4'd7) begin
                    op_step <= 4'd8;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd8) begin
                    // 步骤4：启动传输，SETUP 必须用 DATA0 (0x08) 
                    spi_write_reg(REG_USB_H_CTRL, 8'h08);
                    op_step <= 4'd9;
                end else if(spi_resp_valid && op_step == 4'd9) begin
                    op_step <= OP_FINISH;
                end
            end
            
            // 通用等待中断状态逻辑
            S_ENUM_SET_ADDR_WAIT, S_ENUM_SET_CONF_WAIT: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_INTER_FLAG, 8'd1);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    if(spi_resp_rdata[0][0] == 1'b1) op_step <= 4'd2; // 有中断
                    else op_step <= 4'd0; // 没中断，继续等
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_INTER_FLAG, 8'h1F); // 写1清中断
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= OP_FINISH; // 直接成功，进入下一步
                end
            end
            
            S_ENUM_SET_ADDR_WAIT_IN, S_ENUM_SET_CONF_WAIT_IN: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_read_reg(REG_INTER_FLAG, 8'd1);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    if(spi_resp_rdata[0][0] == 1'b1) op_step <= 4'd2;
                    else op_step <= 4'd0;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_read_reg(REG_USB_STATUS, 8'd1); // 必须先读状态！
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    if ((spi_resp_rdata[0] & 8'h0F) == 4'hA) op_step <= 4'd4; // 1010=NAK
                    else op_step <= 4'd5; // 成功
                end else if(spi_req_ready && !spi_req_valid && (op_step == 4'd4 || op_step == 4'd5)) begin
                    spi_write_reg(REG_INTER_FLAG, 8'h1F); // 后清中断
                    op_step <= op_step + 4'd2;
                end else if(spi_resp_valid && (op_step == 4'd6 || op_step == 4'd7)) begin
                    op_step <= (op_step == 4'd6) ? OP_RESET : OP_FINISH; // 产生分歧路径
                end
            end
            
            S_ENUM_SET_ADDR_IN, S_ENUM_SET_CONF_IN: begin
                // 控制传输状态阶段：下发 IN 令牌
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_write_reg(REG_USB_H_TOKEN, 8'h90);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_USB_H_CTRL, 8'h88); // 启动且 TOG=DATA1
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= OP_FINISH;
                end
            end
            
            S_ENUM_UPDATE_ADDR: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    spi_write_reg(REG_USB_ADDR, 8'h01);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= OP_FINISH;
                end
            end

            // ---------------- USB枚举：Set Configuration (配置 1) ----------------
            S_ENUM_SET_CONF_SETUP: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    // 步骤1：写入 Set_Configuration(1) 包 
                    spi_write_multi(RAM_HOST_TRAN, {8'h00, 8'h00, 8'h00, 8'h00, 8'h00, 8'h01, 8'h09, 8'h00});
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    spi_write_reg(REG_USB_LENGTH, 8'd8);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= 4'd4;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd4) begin
                    spi_write_reg(REG_USB_H_TOKEN, 8'hD0); // SETUP 令牌
                    op_step <= 4'd5;
                end else if(spi_resp_valid && op_step == 4'd5) begin
                    op_step <= 4'd6;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd6) begin
                    spi_write_reg(REG_USB_H_CTRL, 8'h08); // 启动传输 
                    op_step <= 4'd7;
                end else if(spi_resp_valid && op_step == 4'd7) begin
                    op_step <= OP_FINISH;
                end
            end
            
            // ---------------- 非脉冲模式适配的轮询逻辑 ----------------
            S_POLL_WAIT: begin
                if(op_step == 4'd0) begin
                    delay_ms_cnt <= INTERVAL;
                    op_step <= OP_FINISH;
                end
            end
            
            S_POLL_EP1_ISSUE: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    // 步骤1：下发 IN 令牌到 EP1 (PID=0x9, EP=0x1 -> 0x91)
                    spi_write_reg(REG_USB_H_TOKEN, 8'h91);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    op_step <= 4'd2;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    // 步骤2：启动传输，根据Toggle选择DATA0/DATA1
                    spi_write_reg(REG_USB_H_CTRL, ep1_in_toggle ? 8'h88 : 8'h08);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    op_step <= OP_FINISH;
                end
            end
            
            S_POLL_EP1_WAIT_INT: begin
                if (ch374_int_low && op_step == 4'd0) begin
                    op_step <= 4'd1;
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd1) begin
                    spi_read_reg(REG_USB_STATUS, 8'd1);
                    op_step <= 4'd2;
                end else if(spi_resp_valid && op_step == 4'd2) begin
                    // 仅接受“期望的”DATA PID，另一种DATA视为重复包/错toggle，丢弃
                    if ((spi_resp_rdata[0] & 8'h0F) == ep1_expected_pid) begin
                        op_step <= 4'd3; // [有效] 期望 DATA0/DATA1
                    end else if ((spi_resp_rdata[0] & 8'h0F) == 4'hA) begin
                        op_step <= 4'd4; // [正常] NAK
                    end else if (((spi_resp_rdata[0] & 8'h0F) == 4'h3) || ((spi_resp_rdata[0] & 8'h0F) == 4'hB)) begin
                        op_step <= 4'd4; // [丢弃] 非期望DATA(重复包/错toggle)，按NAK路径回轮询
                    end else begin
                        op_step <= 4'd5; // [异常] 其他PID
                    end
                end else if(spi_req_ready && !spi_req_valid && (op_step == 4'd3 || op_step == 4'd4 || op_step == 4'd5)) begin
                    spi_write_reg(REG_INTER_FLAG, 8'h1F); // 写1清中断，CH374规定中断必须清
                    // 将分支引流到不同的下一步
                    if (op_step == 4'd3) op_step <= 4'd6;      
                    else if (op_step == 4'd4) op_step <= 4'd7; 
                    else op_step <= 4'd8;                      
                end else if(spi_resp_valid && (op_step == 4'd6 || op_step == 4'd7 || op_step == 4'd8)) begin
                    if (op_step == 4'd6) op_step <= OP_FINISH;      // 有效数据，去 S_POLL_EP1_READ 读包
                    else if (op_step == 4'd7) op_step <= OP_RESET;  // NAK，去 S_POLL_WAIT 等待下一回合
                    else op_step <= OP_ERR;                         // [新增] 方案二：抛出错误标志，强制触发状态机跳转到 S_ERROR
                end 
            end
            
            S_POLL_EP1_READ: begin
                if(spi_req_ready && !spi_req_valid && op_step == 4'd0) begin
                    // 步骤1：读取当前 USB 传输的接收长度
                    spi_read_reg(REG_USB_LENGTH, 8'd1);
                    op_step <= 4'd1;
                end else if(spi_resp_valid && op_step == 4'd1) begin
                    // 步骤2：长度校验
                    if (spi_resp_rdata[0] == REPORT_LEN) begin
                        op_step <= 4'd2;
                    end else begin
                        // 长度不吻合，直接丢弃
                        ep1_in_toggle <= ~ep1_in_toggle;
                        op_step <= OP_FINISH;
                    end
                end else if(spi_req_ready && !spi_req_valid && op_step == 4'd2) begin
                    // 步骤3：读取实际的 13 字节负载数据
                    spi_read_reg(RAM_HOST_RECV, USEFUL_DATA_LEN);
                    op_step <= 4'd3;
                end else if(spi_resp_valid && op_step == 4'd3) begin
                    o_mouse_buttons <= {spi_resp_rdata[1], spi_resp_rdata[0]};
                    o_mouse_x       <= {spi_resp_rdata[3], spi_resp_rdata[2]};
                    o_mouse_y       <= {spi_resp_rdata[5], spi_resp_rdata[4]};
                    o_mouse_wheel   <=  spi_resp_rdata[6];
                    o_mouse_data_valid <= 1'b1;
                    ep1_in_toggle <= ~ep1_in_toggle;
                    op_step <= OP_FINISH;
                end
            end
            
            S_ERROR: begin
                if (op_step == 4'd0) begin
                    delay_ms_cnt <= 8'd250;
                    op_step <= 4'd1;

                    o_mouse_buttons <= 16'd0;
                    o_mouse_x <= 16'd0;
                    o_mouse_y <= 16'd0;
                    o_mouse_wheel <= 8'd0;
                    o_mouse_data_valid <= 1'b1;
                end
                dev_is_low_speed <= 1'b0;
                ep1_in_toggle <= 1'b0;
            end
        endcase
        
        if (curr_state != next_state) begin
            op_step <= 4'd0;
        end
    end
end

endmodule