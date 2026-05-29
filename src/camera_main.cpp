#include <Arduino.h>
#include <M5UnitGLASS2.h>
#include <esp_camera.h>
#include <FastLED.h>
#include <math.h>

// --- NeoPixel Configuration ---
#define LED_PIN     38
#define NUM_LEDS    8
CRGB leds[NUM_LEDS];

// --- Camera Pins for Atom S3R CAM ---
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    21
#define SIOD_GPIO_NUM    12
#define SIOC_GPIO_NUM    9
#define Y9_GPIO_NUM      13
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      17
#define Y6_GPIO_NUM      4
#define Y5_GPIO_NUM      48
#define Y4_GPIO_NUM      46
#define Y3_GPIO_NUM      42
#define Y2_GPIO_NUM      3
#define VSYNC_GPIO_NUM   10
#define HREF_GPIO_NUM    14
#define PCLK_GPIO_NUM    40
#define POWER_GPIO_NUM   18 

#define IMG_WIDTH 320
#define IMG_HEIGHT 240
#define CENTER_X 153 
#define CENTER_Y 115
#define RADIUS 84
#define BITS_COUNT 36
#define POINTS_PER_BIT 10
#define TOTAL_POINTS (BITS_COUNT * POINTS_PER_BIT)
#define ENABLE_PREPROCESS 1
#define ROI_MARGIN 10

M5UnitGLASS2 display;

int lot_number = 0;
uint32_t serial_number = 0;
bool decoded_once = false;
uint32_t last_success_time = 0;
int marker_pos = -1;

uint8_t led_r = 255, led_g = 255, led_b = 255, led_brightness = 150;
bool led_on = true;

uint8_t *r_proc_buf = NULL;
uint8_t *g_proc_buf = NULL;
uint8_t *b_proc_buf = NULL;
uint8_t *tmp_proc_buf = NULL;

float sin_lut[TOTAL_POINTS];
float cos_lut[TOTAL_POINTS];
int r_samples[TOTAL_POINTS], g_samples[TOTAL_POINTS], b_samples[TOTAL_POINTS];
int r_sums[BITS_COUNT], g_sums[BITS_COUNT], b_sums[BITS_COUNT];
uint8_t r_bits[BITS_COUNT], g_bits[BITS_COUNT], b_bits[BITS_COUNT];
float r_thr = 0, g_thr = 0, b_thr = 0;
float phase_offset = 0;

bool thr_auto_mode = true;
int thr_manual_r = 128, thr_manual_g = 128, thr_manual_b = 128;

void initLut() {
    for (int i = 0; i < TOTAL_POINTS; i++) {
        float rad = (i * 1.0f) * PI / 180.0f;
        sin_lut[i] = sin(rad);
        cos_lut[i] = cos(rad);
    }
}

bool initCamera() {
    pinMode(POWER_GPIO_NUM, OUTPUT);
    digitalWrite(POWER_GPIO_NUM, LOW);
    delay(500);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_RGB565;
    config.frame_size = FRAMESIZE_QVGA; // 320x240
    config.fb_count = 2; 
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.jpeg_quality = 12; // Not used
    config.grab_mode = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) return false;

    sensor_t *s = esp_camera_sensor_get();
    if (s) s->set_hmirror(s, 0);
    return true;
}

void applyGaussian3x3(uint8_t *buf, int x_start, int y_start, int w, int h) {
    memcpy(tmp_proc_buf, buf, IMG_WIDTH * IMG_HEIGHT);
    for (int y = y_start + 1; y < y_start + h - 1; y++) {
        for (int x = x_start + 1; x < x_start + w - 1; x++) {
            int sum = 
                tmp_proc_buf[(y-1)*IMG_WIDTH + (x-1)] * 1 + tmp_proc_buf[(y-1)*IMG_WIDTH + x] * 2 + tmp_proc_buf[(y-1)*IMG_WIDTH + (x+1)] * 1 +
                tmp_proc_buf[y*IMG_WIDTH + (x-1)]     * 2 + tmp_proc_buf[y*IMG_WIDTH + x]     * 4 + tmp_proc_buf[y*IMG_WIDTH + (x+1)]     * 2 +
                tmp_proc_buf[(y+1)*IMG_WIDTH + (x-1)] * 1 + tmp_proc_buf[(y+1)*IMG_WIDTH + x] * 2 + tmp_proc_buf[(y+1)*IMG_WIDTH + (x+1)] * 1;
            buf[y * IMG_WIDTH + x] = sum >> 4;
        }
    }
}

void applyMaxFilter3x3(uint8_t *buf, int x_start, int y_start, int w, int h) {
    memcpy(tmp_proc_buf, buf, IMG_WIDTH * IMG_HEIGHT);
    for (int y = y_start + 1; y < y_start + h - 1; y++) {
        for (int x = x_start + 1; x < x_start + w - 1; x++) {
            uint8_t max_val = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    uint8_t val = tmp_proc_buf[(y + dy) * IMG_WIDTH + (x + dx)];
                    if (val > max_val) max_val = val;
                }
            }
            buf[y * IMG_WIDTH + x] = max_val;
        }
    }
}

void processImage(camera_fb_t *fb) {
    int x_start = CENTER_X - RADIUS - ROI_MARGIN;
    int y_start = CENTER_Y - RADIUS - ROI_MARGIN;
    int roi_w = (RADIUS + ROI_MARGIN) * 2;
    int roi_h = roi_w;
    
    if (x_start < 0) x_start = 0;
    if (y_start < 0) y_start = 0;
    if (x_start + roi_w > IMG_WIDTH) roi_w = IMG_WIDTH - x_start;
    if (y_start + roi_h > IMG_HEIGHT) roi_h = IMG_HEIGHT - y_start;

    for (int y = y_start; y < y_start + roi_h; y++) {
        for (int x = x_start; x < x_start + roi_w; x++) {
            size_t idx = (y * IMG_WIDTH + x) * 2;
            uint16_t pixel = (fb->buf[idx] << 8) | fb->buf[idx+1];
            uint8_t r = (pixel >> 11) & 0x1F;
            uint8_t g = (pixel >> 5) & 0x3F;
            uint8_t b = pixel & 0x1F;
            r_proc_buf[y * IMG_WIDTH + x] = (r << 3) | (r >> 2);
            g_proc_buf[y * IMG_WIDTH + x] = (g << 2) | (g >> 4);
            b_proc_buf[y * IMG_WIDTH + x] = (b << 3) | (b >> 2);
        }
    }

    if (ENABLE_PREPROCESS) {
        applyGaussian3x3(r_proc_buf, x_start, y_start, roi_w, roi_h);
        applyGaussian3x3(g_proc_buf, x_start, y_start, roi_w, roi_h);
        applyGaussian3x3(b_proc_buf, x_start, y_start, roi_w, roi_h);
        // 最大値フィルタ（膨張）を無効化（谷が潰れて検出できなくなる原因のため）
        applyMaxFilter3x3(r_proc_buf, x_start, y_start, roi_w, roi_h);
        applyMaxFilter3x3(g_proc_buf, x_start, y_start, roi_w, roi_h);
        applyMaxFilter3x3(b_proc_buf, x_start, y_start, roi_w, roi_h);
    }

    for (int i = 0; i < TOTAL_POINTS; i++) {
        int x = CENTER_X + (int)(cos_lut[i] * RADIUS);
        int y = CENTER_Y + (int)(sin_lut[i] * RADIUS);
        if (x >= 0 && x < IMG_WIDTH && y >= 0 && y < IMG_HEIGHT) {
            r_samples[i] = r_proc_buf[y * IMG_WIDTH + x];
            g_samples[i] = g_proc_buf[y * IMG_WIDTH + x];
            b_samples[i] = b_proc_buf[y * IMG_WIDTH + x];
        } else {
            r_samples[i] = 0;
            g_samples[i] = 0;
            b_samples[i] = 0;
        }
    }

    // --- 白色区切り(White Divider)を用いた位相オフセットの検出 ---
    // R,G,Bすべてが明るい箇所が周期的に並んでいることを利用し、36周期成分を解析
    float sum_cos = 0, sum_sin = 0;
    for (int i = 0; i < TOTAL_POINTS; i++) {
        // 白色の区切りはR,G,Bすべてが明るい箇所。合計値を指標にする。
        float w = r_samples[i] + g_samples[i] + b_samples[i];
        float angle = 2.0f * PI * i / ((float)TOTAL_POINTS / BITS_COUNT); // 周期10
        sum_cos += w * cos(angle);
        sum_sin += w * sin(angle);
    }
    // 36周期成分の位相を求め、オフセットを算出
    float phase = atan2(sum_sin, sum_cos);
    phase_offset = phase * ((float)TOTAL_POINTS / BITS_COUNT) / (2.0f * PI);
    if (phase_offset < 0) phase_offset += ((float)TOTAL_POINTS / BITS_COUNT);

    // --- 各ビットのサンプリング ---
    // 1ビット10ポイントのうち、最も低い5ポイントの平均をとる。
    // これにより、位相が多少ずれて白い区切り線(高輝度)を拾っても、
    // ビット本来の暗い部分(0)を優先的に評価できる。
    uint32_t r_sum_total = 0, g_sum_total = 0, b_sum_total = 0;

    for (int k = 0; k < BITS_COUNT; k++) {
        // 区切りの位置が phase_offset + k*10
        int start_i = (int)round(phase_offset + k * 10.0f);
        std::vector<int> r_win, g_win, b_win;
        
        for (int w = 0; w < 10; w++) {
            int idx = (start_i + w + TOTAL_POINTS) % TOTAL_POINTS;
            r_win.push_back(r_samples[idx]);
            g_win.push_back(g_samples[idx]);
            b_win.push_back(b_samples[idx]);
        }
        
        std::sort(r_win.begin(), r_win.end());
        std::sort(g_win.begin(), g_win.end());
        std::sort(b_win.begin(), b_win.end());

        uint32_t r_sec = 0, g_sec = 0, b_sec = 0;
        for (int i = 0; i < 5; i++) {
            r_sec += r_win[i];
            g_sec += g_win[i];
            b_sec += b_win[i];
        }
        
        r_sums[k] = r_sec;
        g_sums[k] = g_sec;
        b_sums[k] = b_sec;
        r_sum_total += r_sec;
        g_sum_total += g_sec;
        b_sum_total += b_sec;
    }

    int window_size = 5; // 評価に使用したサンプル数
    if (thr_auto_mode) {
        r_thr = (float)r_sum_total / (BITS_COUNT * window_size);
    } else {
        r_thr = thr_manual_r;
    }

    for (int k = 0; k < BITS_COUNT; k++) {
        r_bits[k] = ((float)r_sums[k] / window_size > r_thr) ? 1 : 0;
    }

    marker_pos = -1;
    for (int i = 0; i < BITS_COUNT; i++) {
        if (r_bits[i] == 1 && r_bits[(i + 1) % BITS_COUNT] == 1 && 
            r_bits[(i + 2) % BITS_COUNT] == 1 && r_bits[(i + 3) % BITS_COUNT] == 1) {
            marker_pos = i;
            break;
        }
    }

    if (thr_auto_mode) {
        if (marker_pos != -1) {
            // マーカーが検出された場合、マーカー部分（4ビット）を除外してGとBの閾値を再計算する
            uint32_t g_sum_recalc = 0;
            uint32_t b_sum_recalc = 0;
            int valid_bits = 0;

            for (int k = 0; k < BITS_COUNT; k++) {
                // マーカー領域内かどうかを判定
                bool is_marker_area = false;
                for (int m = 0; m < 4; m++) {
                    if (k == (marker_pos + m) % BITS_COUNT) {
                        is_marker_area = true;
                        break;
                    }
                }

                if (!is_marker_area) {
                    g_sum_recalc += g_sums[k];
                    b_sum_recalc += b_sums[k];
                    valid_bits++;
                }
            }

            if (valid_bits > 0) {
                g_thr = (float)g_sum_recalc / (valid_bits * window_size);
                b_thr = (float)b_sum_recalc / (valid_bits * window_size);
            }
        } else {
            g_thr = (float)g_sum_total / (BITS_COUNT * window_size);
            b_thr = (float)b_sum_total / (BITS_COUNT * window_size);
        }
    } else {
        g_thr = thr_manual_g;
        b_thr = thr_manual_b;
    }

    // GとBのビットを判定
    for (int k = 0; k < BITS_COUNT; k++) {
        g_bits[k] = ((float)g_sums[k] / window_size > g_thr) ? 1 : 0;
        b_bits[k] = ((float)b_sums[k] / window_size > b_thr) ? 1 : 0;
    }

    if (marker_pos != -1) {
        lot_number = 0;
        for (int i = 0; i < 8; i++) {
            lot_number |= (g_bits[(marker_pos + 4 + i) % BITS_COUNT] << (7 - i));
        }
        serial_number = 0;
        for (int i = 0; i < 24; i++) {
            serial_number |= ((uint32_t)g_bits[(marker_pos + 12 + i) % BITS_COUNT] << (23 - i));
        }
        decoded_once = true;
        last_success_time = millis();
    }

    // Output Debug
    Serial.println("--- DEBUG START ---");
    
    Serial.print("DATA,R_BITS,");
    for (int i = 0; i < BITS_COUNT; i++) Serial.print(r_bits[i]);
    Serial.println();

    Serial.print("DATA,G_BITS,");
    for (int i = 0; i < BITS_COUNT; i++) Serial.print(g_bits[i]);
    Serial.println();

    Serial.print("DATA,B_BITS,");
    for (int i = 0; i < BITS_COUNT; i++) Serial.print(b_bits[i]);
    Serial.println();

    Serial.print("DATA,R_SUMS,");
    for (int i = 0; i < BITS_COUNT; i++) {
        Serial.print(r_sums[i]);
        if(i < BITS_COUNT - 1) Serial.print(",");
    }
    Serial.println();

    Serial.print("DATA,B_SUMS,");
    for (int i = 0; i < BITS_COUNT; i++) {
        Serial.print(b_sums[i]);
        if(i < BITS_COUNT - 1) Serial.print(",");
    }
    Serial.println();

    Serial.print("DATA,R_FILT_SAMP,");
    for (int i = 0; i < TOTAL_POINTS; i++) {
        Serial.print(r_samples[i]);
        if(i < TOTAL_POINTS - 1) Serial.print(",");
    }
    Serial.println();

    Serial.print("DATA,G_FILT_SAMP,");
    for (int i = 0; i < TOTAL_POINTS; i++) {
        Serial.print(g_samples[i]);
        if(i < TOTAL_POINTS - 1) Serial.print(",");
    }
    Serial.println();

    Serial.print("DATA,B_FILT_SAMP,");
    for (int i = 0; i < TOTAL_POINTS; i++) {
        Serial.print(b_samples[i]);
        if(i < TOTAL_POINTS - 1) Serial.print(",");
    }
    Serial.println();

    Serial.printf("DATA,R_THRESHOLD,%.2f\n", r_thr);
    Serial.printf("DATA,G_THRESHOLD,%.2f\n", g_thr);
    Serial.printf("DATA,B_THRESHOLD,%.2f\n", b_thr);
    Serial.printf("DATA,PHASE_OFFSET,%.2f\n", phase_offset);
    Serial.printf("DATA,MARKER_POS,%d\n", marker_pos);
    
    if (marker_pos != -1) {
        Serial.printf("DATA,DECODE,%d,%u\n", lot_number, serial_number);
    }
    
    Serial.println("--- DEBUG END ---");
}

inline uint8_t getGray(uint16_t rgb) {
    uint8_t r = ((rgb >> 11) & 0x1F) << 3;
    uint8_t g = ((rgb >> 5) & 0x3F) << 2;
    uint8_t b = (rgb & 0x1F) << 3;
    return (uint8_t)(0.299f * r + 0.587f * g + 0.114f * b);
}

void updateUI(camera_fb_t * fb) {
    display.startWrite();
    display.clear();

    if (fb && fb->format == PIXFORMAT_RGB565) {
        int preview_x = 64;
        int preview_y = 8;
        int scale = 5;
        for (int y = 0; y < 48; y++) {
            for (int x = 0; x < 64; x++) {
                size_t pixel_index = ((y * scale) * IMG_WIDTH + (x * scale)) * 2;
                uint16_t pixel = (fb->buf[pixel_index] << 8) | (fb->buf[pixel_index + 1]);
                uint8_t val = getGray(pixel);
                display.drawPixel(preview_x + x, preview_y + y, (val > 120) ? WHITE : BLACK);
            }
        }
        display.drawRect(preview_x - 1, preview_y - 1, 66, 50, WHITE);
        
        // サンプリング位置の円をプレビュー上に描画
        int preview_cx = preview_x + (CENTER_X / scale);
        int preview_cy = preview_y + (CENTER_Y / scale);
        int preview_r = RADIUS / scale;
        display.drawCircle(preview_cx, preview_cy, preview_r, WHITE);
    }

    display.setCursor(0, 0);
    display.println("CAM READY");
    
    if (decoded_once && (millis() - last_success_time < 1500)) {
        display.setCursor(0, 24);
        display.printf("LOT:%03d\n", lot_number);
        display.printf("SN:%08u\n", serial_number);
        display.setCursor(0, 48);
        display.println("[OK]");
    } else {
        display.setCursor(0, 32);
        display.println("WAITING...");
    }

    display.display();
    display.endWrite();
}

char rx_buf[128];
int rx_idx = 0;

void handleCommands() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n') {
            rx_buf[rx_idx] = 0;
            String cmd = String(rx_buf);
            cmd.trim();
            if (cmd.startsWith("L,")) {
                int on_val, r, g, b, br;
                if (sscanf(cmd.c_str(), "L,%d,%d,%d,%d,%d", &on_val, &r, &g, &b, &br) >= 5) {
                    led_on = (on_val != 0);
                    led_r = r; led_g = g; led_b = b; led_brightness = br;
                    
                    if (led_on) {
                        fill_solid(leds, NUM_LEDS, CRGB(led_r, led_g, led_b));
                        FastLED.setBrightness(led_brightness);
                    } else {
                        fill_solid(leds, NUM_LEDS, CRGB::Black);
                    }
                    FastLED.show();
                }
            } else if (cmd.startsWith("T,")) {
                int auto_val, r, g, b;
                if (sscanf(cmd.c_str(), "T,%d,%d,%d,%d", &auto_val, &r, &g, &b) >= 4) {
                    thr_auto_mode = (auto_val != 0);
                    thr_manual_r = r;
                    thr_manual_g = g;
                    thr_manual_b = b;
                }
            }
            rx_idx = 0;
        } else if (c != '\r') {
            if (rx_idx < sizeof(rx_buf) - 1) {
                rx_buf[rx_idx++] = c;
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);

    display.init(2, 1);
    display.setRotation(1);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clear();
    display.println("Init Camera...");
    display.display();

    r_proc_buf = (uint8_t *)ps_malloc(IMG_WIDTH * IMG_HEIGHT);
    g_proc_buf = (uint8_t *)ps_malloc(IMG_WIDTH * IMG_HEIGHT);
    b_proc_buf = (uint8_t *)ps_malloc(IMG_WIDTH * IMG_HEIGHT);
    tmp_proc_buf = (uint8_t *)ps_malloc(IMG_WIDTH * IMG_HEIGHT);
    
    initLut();
    initCamera();

    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    fill_solid(leds, NUM_LEDS, CRGB::White);
    FastLED.show();
}

void loop() {
    handleCommands();

    static uint32_t last_frame_time = 0;
    if (millis() - last_frame_time > 100) { // 10 FPS
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            processImage(fb);
            updateUI(fb);
            esp_camera_fb_return(fb);
            last_frame_time = millis();
        }
    }
    delay(1);
}
