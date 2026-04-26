#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "tusb.h"
#include "device/usbd_pvt.h"
#include "XInputPad.h"
#include "bsp/board_api.h"
#include "AltaRiscv.h"
#include "interrupt.h"

#define MMIO_BASE     0x60000000
#define ADDR_STATUS   (MMIO_BASE + 0x00)
#define ADDR_KBD_D1   (MMIO_BASE + 0x04)
#define ADDR_KBD_D2   (MMIO_BASE + 0x08)
#define ADDR_MOUSE_D1 (MMIO_BASE + 0x0C)
#define ADDR_MOUSE_D2 (MMIO_BASE + 0x10)

#define XINPUT_MOUSE_TO_STICK_SCALE 40
#define XINPUT_MOUSE_TO_STICK_SCALE_LOOT 180

#define XINPUT_STICK_MAX  32767
#define XINPUT_STICK_MIN -32767
#define XINPUT_STICK_DIAG  23170
#define XINPUT_STICK_DIAG_N -23170

#define MAX_DX 820
#define NUM_INTERVALS 20
#define DX_STEP (MAX_DX / NUM_INTERVALS) // 41
#define MAX_OUT 32767

#define XBOX_BUTTON_UP     (1 << 0)
#define XBOX_BUTTON_DOWN   (1 << 1)
#define XBOX_BUTTON_LEFT   (1 << 2)
#define XBOX_BUTTON_RIGHT  (1 << 3)
#define XBOX_BUTTON_START  (1 << 4)
#define XBOX_BUTTON_BACK   (1 << 5)
#define XBOX_BUTTON_L3     (1 << 6)
#define XBOX_BUTTON_R3     (1 << 7)
#define XBOX_BUTTON_LB     (1 << 8)
#define XBOX_BUTTON_RB     (1 << 9)
#define XBOX_BUTTON_GUIDE  (1 << 10)
#define XBOX_BUTTON_A      (1 << 12)
#define XBOX_BUTTON_B      (1 << 13)
#define XBOX_BUTTON_X      (1 << 14)
#define XBOX_BUTTON_Y      (1 << 15)

static const uint32_t s_key_lut_btn[256] = {
    [0x14] = XBOX_BUTTON_LB,                    // Q
    [0x08] = XBOX_BUTTON_X,                     // E
    [0x15] = XBOX_BUTTON_X,                     // R
    [0x09] = XBOX_BUTTON_R3,                    // F
    [0x0A] = XBOX_BUTTON_RIGHT,                 // G
    [0x17] = XBOX_BUTTON_RIGHT,                 // T
    [0x10] = XBOX_BUTTON_BACK,                  // M
    [0x19] = XBOX_BUTTON_LEFT,                  // V
    [0x1D] = XBOX_BUTTON_LB | XBOX_BUTTON_RB,   // Z
    [0x1E] = XBOX_BUTTON_Y,                     // 1
    [0x1F] = XBOX_BUTTON_Y,                     // 2
    [0x20] = XBOX_BUTTON_Y,                     // 3
    [0x2C] = XBOX_BUTTON_A,                     // Space
    [0x2B] = XBOX_BUTTON_START,                 // Tab
    [0x29] = XBOX_BUTTON_START,                 // Esc
    [0x52] = XBOX_BUTTON_UP,                    // Up
    [0x51] = XBOX_BUTTON_DOWN,                  // Down
    [0x50] = XBOX_BUTTON_LEFT,                  // Left
    [0x4F] = XBOX_BUTTON_RIGHT,                 // Right
};

static const uint8_t s_key_lut_wasd[256] = {
    [0x1A] = 1u << 0,   // W
    [0x04] = 1u << 1,   // A
    [0x16] = 1u << 2,   // S
    [0x07] = 1u << 3,   // D
};

static const int16_t s_lx_lut[16] = {
    0, 0, XINPUT_STICK_MIN, XINPUT_STICK_DIAG_N, 0, 0, XINPUT_STICK_DIAG_N, XINPUT_STICK_MIN, 
    XINPUT_STICK_MAX, XINPUT_STICK_DIAG, 0, 0, XINPUT_STICK_DIAG, XINPUT_STICK_MAX, 0, 0
};

static const int16_t s_ly_lut[16] = {
    0, XINPUT_STICK_MAX, 0, XINPUT_STICK_DIAG, XINPUT_STICK_MIN, 0, XINPUT_STICK_DIAG_N, 0, 
    0, XINPUT_STICK_DIAG, 0, XINPUT_STICK_MAX, XINPUT_STICK_DIAG_N, 0, XINPUT_STICK_MIN, 0
};

typedef struct {
    uint8_t kbd_modifier;
    uint8_t kbd_keycode[6];
    uint16_t mouse_buttons;
    int32_t mouse_dx;
    int32_t mouse_dy;
    int16_t mouse_wheel;
} raw_input_state_t;

static volatile raw_input_state_t local_in;

static ReportDataXinput send_pkt __attribute__((aligned(4)));;

static uint8_t endpoint_in = 0;
static uint8_t endpoint_out = 0;

// 全局的动态查表数组，共 21 个锚点 (0 到 20)
static int16_t s_dynamic_curve_lut[NUM_INTERVALS + 1];

// sag_level: 0 (无下陷，纯线性) 到 100 (最大下陷，抛物线)
void update_mouse_curve(int32_t sag_level) {
    // 限制参数范围
    if (sag_level < 0) sag_level = 0;
    if (sag_level > 100) sag_level = 100;

    for (int i = 0; i <= NUM_INTERVALS; i++) {
        int32_t x = i * DX_STEP;
        
        // 1. 计算纯线性值 (相当于 dx * 40)
        int32_t y_lin = (x * MAX_OUT) / MAX_DX; // 接近 x * 40
        if (y_lin > MAX_OUT) y_lin = MAX_OUT;
        
        // 2. 计算二次方曲线值 (y = x^2 * MAX_OUT / MAX_DX^2)
        // 使用 64 位整数防止 x^2 * MAX_OUT 溢出
        int64_t x64 = x;
        int64_t max_dx64 = MAX_DX;
        int32_t y_quad = (int32_t)((x64 * x64 * MAX_OUT) / (max_dx64 * max_dx64));
        
        // 3. 根据 sag_level 混合这两种模型
        int32_t y_final = y_lin + ((y_quad - y_lin) * sag_level) / 100;
        
        s_dynamic_curve_lut[i] = (int16_t)y_final;
    }
}

static inline int16_t apply_dynamic_curve(int32_t raw_delta) {
    int32_t abs_d = (raw_delta > 0) ? raw_delta : -raw_delta;
    
    // 封顶，超过 820 直接输出最大值
    if (abs_d >= MAX_DX) {
        return (raw_delta > 0) ? MAX_OUT : -MAX_OUT;
    }
    
    // 计算所在区间和余数
    int32_t index = abs_d / DX_STEP; 
    int32_t rem = abs_d % DX_STEP;
    
    // 获取区间两端的锚点
    int32_t y0 = s_dynamic_curve_lut[index];
    int32_t y1 = s_dynamic_curve_lut[index + 1];
    
    // 整数线性插值
    int16_t mapped_val = (int16_t)(y0 + ((y1 - y0) * rem) / DX_STEP);
    
    return (raw_delta > 0) ? mapped_val : -mapped_val;
}

static inline int16_t clamp_s16(int32_t v) {
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline uint32_t isqrt(uint32_t n) {
    uint32_t root = 0;
    uint32_t bit = 1UL << 30; // 假设是 32位 整数
    
    while (bit > n) bit >>= 2;
    
    while (bit != 0) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

static inline void square_to_circle_int(int16_t x, int16_t y, int16_t *out_x, int16_t *out_y) {
    // 拦截原点，防止后续除以 0
    if (x == 0 && y == 0) {
        *out_x = 0; 
        *out_y = 0;
        return;
    }

    // 求绝对值 (转为 32位 防止 -32768 取绝对值溢出)
    int32_t ax = (x > 0) ? x : -(int32_t)x;
    int32_t ay = (y > 0) ? y : -(int32_t)y;
    int32_t max_side = (ax > ay) ? ax : ay;

    // 计算平方和 
    // 注意：一定要用 uint32_t！因为 (-32768)^2 + (-32768)^2 = 2,147,483,648
    // 这个值刚好比 signed int32_t 的最大值 (2,147,483,647) 大 1，用有符号会溢出。
    uint32_t xx = (uint32_t)ax * ax;
    uint32_t yy = (uint32_t)ay * ay;
    uint32_t mag = isqrt(xx + yy);

    // 核心转换：利用“先乘后除”替代浮点小数的 rescale
    // 原逻辑: x * (max_side / mag) -> 新逻辑: (x * max_side) / mag
    // 32768 * 32768 = 1,073,741,824，在 int32_t 范围内，不会溢出
    *out_x = (int16_t)(((int32_t)x * max_side) / (int32_t)mag);
    *out_y = (int16_t)(((int32_t)y * max_side) / (int32_t)mag);
}

static void build_xinput_report(const raw_input_state_t *in, ReportDataXinput *out) {

    out->rsize = 0x14; // 20 字节
    
    uint16_t buttons = 0;
    int16_t temp_x = 0, temp_y = 0, recoil_offset = 0;
    uint8_t wasd = 0;

    static uint8_t jitter_tick = 0;
    static int16_t current_jitter_amp = 990;
    int16_t jitter = 0;

    if(jitter_tick >= 15) {
        current_jitter_amp = -current_jitter_amp;
        jitter_tick = 0;
    } else {
        jitter_tick++;
    }

    if (in->kbd_modifier & ((1u << 0) | (1u << 4))) buttons |= XBOX_BUTTON_B;   // Ctrl
    if (in->kbd_modifier & ((1u << 2) | (1u << 3))) buttons |= XBOX_BUTTON_DOWN;// Alt
    if (in->kbd_modifier & ((1u << 1) | (1u << 5))) buttons |= XBOX_BUTTON_L3;  // Shift

    for (uint8_t i = 0; i < 6; i++) {
        uint8_t kc = in->kbd_keycode[i];
        if(kc == 0) continue;
        buttons |= s_key_lut_btn[kc];
        wasd    |= s_key_lut_wasd[kc];
    }

    if(in->mouse_buttons & (1u << 4)) { // 特殊鼠标侧键逻辑
        out->r_x = s_lx_lut[wasd & 0x0F];
        out->r_y = s_ly_lut[wasd & 0x0F];
        if(in->mouse_wheel < 0) out->r_y = -32767;
        if(in->mouse_wheel > 0) out->r_y = 32767;
        square_to_circle_int(clamp_s16(in->mouse_dx * XINPUT_MOUSE_TO_STICK_SCALE_LOOT),
                         clamp_s16(-in->mouse_dy * XINPUT_MOUSE_TO_STICK_SCALE_LOOT),
                         &temp_x, &temp_y);
        out->l_x = temp_x;
        out->l_y = temp_y;
        if (in->mouse_buttons & (1u << 0)) buttons |= XBOX_BUTTON_A;
        if (in->mouse_buttons & (1u << 1)) buttons |= XBOX_BUTTON_X;
    } else { // 正常视角映射
        out->l_x = s_lx_lut[wasd & 0x0F];
        out->l_y = s_ly_lut[wasd & 0x0F];
        if (in->mouse_buttons & (1u << 0)) { out->rt = 255; recoil_offset = -200; jitter = current_jitter_amp;}
        if (in->mouse_buttons & (1u << 1)) { out->lt = 255; recoil_offset *= 4;} 
        if (in->mouse_buttons & (1u << 2)) buttons |= XBOX_BUTTON_RB;
        if (in->mouse_buttons & (1u << 3)) buttons |= XBOX_BUTTON_UP;
        if (in->mouse_wheel != 0)          buttons |= XBOX_BUTTON_Y;
        int16_t curve_x = apply_dynamic_curve(in->mouse_dx);
        int16_t curve_y = apply_dynamic_curve(-in->mouse_dy); // 注意反转
        square_to_circle_int(clamp_s16(curve_x + jitter),
                             clamp_s16(curve_y + recoil_offset),
                             &temp_x, &temp_y);
        out->r_x = temp_x;
        out->r_y = temp_y;
    }
    
    // 直接赋给 uint16_t 的 buttons
    out->buttons = buttons;
}

void LOCAL_INT0_isr(void) {
    uint32_t status;
    int8_t raw_wheel_inc = 0; 
    
    // 只要有状态，就一直处理，防漏拍
    while ((status = *((volatile uint32_t *)ADDR_STATUS)) != 0) {

        __sync_synchronize();
        
        // 键盘数据有更新
        if (status & 0x01) { 
            uint32_t k1 = *((volatile uint32_t *)ADDR_KBD_D1);
            uint32_t k2 = *((volatile uint32_t *)ADDR_KBD_D2);
            
            // 解析 D1 (前 4 个按键)
            local_in.kbd_keycode[0] = (uint8_t)(k1 & 0xFF);
            local_in.kbd_keycode[1] = (uint8_t)((k1 >> 8) & 0xFF);
            local_in.kbd_keycode[2] = (uint8_t)((k1 >> 16) & 0xFF);
            local_in.kbd_keycode[3] = (uint8_t)((k1 >> 24) & 0xFF);
            
            // 解析 D2 (后 2 个按键 + Modifier)
            local_in.kbd_modifier   = (uint8_t)(k2 & 0xFF);
            local_in.kbd_keycode[4] = (uint8_t)((k2 >> 8) & 0xFF);
            local_in.kbd_keycode[5] = (uint8_t)((k2 >> 16) & 0xFF);
        }
        
        // 鼠标数据有更新
        if (status & 0x02) { 
            uint32_t m1 = *((volatile uint32_t *)ADDR_MOUSE_D1);
            uint32_t m2 = *((volatile uint32_t *)ADDR_MOUSE_D2);
            
            // 解析 D1 (X 和 Y 位移，需要累加)
            local_in.mouse_dx += (int16_t)(m1 & 0xFFFF);
            local_in.mouse_dy += (int16_t)((m1 >> 16) & 0xFFFF);
            
            // 解析 D2 (按钮和滚轮)
            local_in.mouse_buttons = (uint16_t)(m2 & 0xFFFF);
            raw_wheel_inc = (int8_t)((m2 >> 16) & 0xFF);
            local_in.mouse_wheel += (int16_t)raw_wheel_inc << 5;
        }

        __sync_synchronize();

        // 把读出来的 status 原封不动写回去，触发 FPGA 的 W1C 清 0
        *((volatile uint32_t *)ADDR_STATUS) = status;
    }
}

int main(void) {
  board_init();

  update_mouse_curve(40);

  INT_EnableIntLocal(LOCAL_INT0_IRQn);
  INT_EnableIRQ(LOCAL_INT0_IRQn, PLIC_MAX_PRIORITY);

  tusb_init();
  bool first_packet_sent = false;
  memset((void*)&local_in, 0, sizeof(local_in));
  
  while (1) {
    tud_task();

    if (!first_packet_sent && tud_ready() && (endpoint_in != 0)) {
        first_packet_sent = true;

        memset(&send_pkt, 0, sizeof(send_pkt));
        send_pkt.rsize = 0x14;

        usbd_edpt_claim(0, endpoint_in);
        if (!usbd_edpt_busy(0, endpoint_in)) {
            usbd_edpt_xfer(0, endpoint_in, (uint8_t*)&send_pkt, sizeof(send_pkt));
        }
        usbd_edpt_release(0, endpoint_in);
    }
  }
  return 0;
}

static void xinput_init(void) { }
static void xinput_reset(uint8_t __unused rhport) { }
static uint16_t xinput_open(uint8_t __unused rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len) {
  // ... 保持你的原始代码逻辑 ...
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + itf_desc->bNumEndpoints*sizeof(tusb_desc_endpoint_t) + 16;
  TU_VERIFY(max_len >= drv_len, 0);

  uint8_t const * p_desc = tu_desc_next(itf_desc);
  uint8_t found_endpoints = 0;
  while ( (found_endpoints < itf_desc->bNumEndpoints) && (drv_len <= max_len)  )
  {
    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;
    if ( TUSB_DESC_ENDPOINT == tu_desc_type(desc_ep) )
    {
      TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

      if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN )
      {
        endpoint_in = desc_ep->bEndpointAddress;
      }else
      {
        endpoint_out = desc_ep->bEndpointAddress;
      }
      found_endpoints += 1;
    }
    p_desc = tu_desc_next(p_desc);
  }
  return drv_len;
}

static bool xinput_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) { return true; }
static bool xinput_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) { 
    (void) result;
    (void) xferred_bytes;

    // 检查：设备准备好了吗？
    if (!tud_ready()) {
        return true;
    }

    raw_input_state_t snapshot;

    INT_DisableIntGlobal();
    snapshot.kbd_modifier = local_in.kbd_modifier;
    snapshot.kbd_keycode[0] = local_in.kbd_keycode[0];
    snapshot.kbd_keycode[1] = local_in.kbd_keycode[1];
    snapshot.kbd_keycode[2] = local_in.kbd_keycode[2];
    snapshot.kbd_keycode[3] = local_in.kbd_keycode[3];
    snapshot.kbd_keycode[4] = local_in.kbd_keycode[4];
    snapshot.kbd_keycode[5] = local_in.kbd_keycode[5];
    snapshot.mouse_buttons = local_in.mouse_buttons;
    snapshot.mouse_dx      = local_in.mouse_dx;
    snapshot.mouse_dy      = local_in.mouse_dy;
    snapshot.mouse_wheel   = local_in.mouse_wheel;
    
    local_in.mouse_dx = 0;
    local_in.mouse_dy = 0;
    if(local_in.mouse_wheel > 0) local_in.mouse_wheel--;
    if(local_in.mouse_wheel < 0) local_in.mouse_wheel++;

    memset(&send_pkt, 0, sizeof(send_pkt));
    
    INT_EnableIntGlobal();

    build_xinput_report(&snapshot, &send_pkt);

    usbd_edpt_claim(rhport, ep_addr);
    if (!usbd_edpt_busy(rhport, ep_addr)) {
        usbd_edpt_xfer(rhport, ep_addr, (uint8_t*)&send_pkt, sizeof(send_pkt));
    }
    usbd_edpt_release(rhport, ep_addr);

    return true;
}

static usbd_class_driver_t const xinput_driver ={
    .init             = xinput_init,
    .reset            = xinput_reset,
    .open             = xinput_open,
    .control_xfer_cb  = xinput_control_xfer_cb, 
    .xfer_cb          = xinput_xfer_cb,
    .sof              = NULL
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    *driver_count = 1;
    return &xinput_driver;
}