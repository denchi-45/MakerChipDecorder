#include <M5Unified.h>
#include <USBHostSerial.h>
#include <usb/vcp.hpp>
#include <vector>
#include <string>
#include <sstream>
#include <SD_MMC.h>

namespace esp_usb {
class Esp32S3CDC : public CdcAcmDevice {
public:
    Esp32S3CDC(uint16_t pid, const cdc_acm_host_device_config_t *dev_config, uint8_t interface_idx = 0) {
        esp_err_t err = this->open(vid, pid, interface_idx, dev_config);
        if (err != ESP_OK) {
            throw err;
        }
    }
    static constexpr uint16_t vid = 0x303A;
    static constexpr std::array<uint16_t, 5> pids = { 0x1001, 0x1002, 0x4001, 0x0002, 0x3000 };
};
}

USBHostSerial usbSerial;

// --- Parameters ---
#define BITS_COUNT 36
#define POINTS_PER_BIT 10
#define TOTAL_POINTS (BITS_COUNT * POINTS_PER_BIT)

// --- UI Layout for 1280x720 ---
#define GRAPH_X 40
#define GRAPH_Y 120
#define GRAPH_W 1200
#define GRAPH_H 400

// Colors mapping manually via RGB888 to fix incorrect macro interpretation on ST7123
#define C_RED      m5gfx::color888(255, 0, 0)
#define C_GREEN    m5gfx::color888(0, 255, 0)
#define C_BLUE     m5gfx::color888(60, 140, 255) // Lighter blue for better visibility
#define C_YELLOW   m5gfx::color888(255, 255, 0)
#define C_WHITE    m5gfx::color888(255, 255, 255)
#define C_BLACK    m5gfx::color888(0, 0, 0)
#define C_DARKGREY m5gfx::color888(40, 40, 40)

int r_samples[TOTAL_POINTS] = {0};
int g_samples[TOTAL_POINTS] = {0};
int b_samples[TOTAL_POINTS] = {0};
uint8_t r_bits_arr[BITS_COUNT] = {0};
uint8_t g_bits_arr[BITS_COUNT] = {0};
uint8_t b_bits_arr[BITS_COUNT] = {0};
int marker_pos = -1;
float r_thr = 0, g_thr = 0, b_thr = 0;
float phase_offset = 0;
int lot = 0; uint32_t sn = 0;
bool decoded_once = false;

int led_br = 150, led_r = 255, led_g = 255, led_b = 255;
bool led_on = true;

bool thr_auto = true;
int thr_manual_r = 128, thr_manual_g = 128, thr_manual_b = 128;
bool show_r = true, show_g = true, show_b = true;

uint32_t save_anim_time = 0;
bool initial_sync_done = false;
bool sd_available = false;

M5Canvas graphCanvas(&M5.Display);

void loadSettings() {
    if (!sd_available) return;
    File f = SD_MMC.open("/settings.txt", FILE_READ);
    if (!f) return;

    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        int eq = line.indexOf('=');
        if (eq != -1) {
            String key = line.substring(0, eq);
            String val = line.substring(eq + 1);
            
            if (key == "led_on") led_on = val.toInt();
            else if (key == "led_br") led_br = val.toInt();
            else if (key == "led_r") led_r = val.toInt();
            else if (key == "led_g") led_g = val.toInt();
            else if (key == "led_b") led_b = val.toInt();
            else if (key == "thr_auto") thr_auto = val.toInt();
            else if (key == "thr_manual_r") thr_manual_r = val.toInt();
            else if (key == "thr_manual_g") thr_manual_g = val.toInt();
            else if (key == "thr_manual_b") thr_manual_b = val.toInt();
            else if (key == "show_r") show_r = val.toInt();
            else if (key == "show_g") show_g = val.toInt();
            else if (key == "show_b") show_b = val.toInt();
        }
    }
    f.close();
}

void saveSettings() {
    if (!sd_available) {
        // Attempt to re-initialize if not available (hot-plug support)
        if (SD_MMC.begin("/sdcard", false)) {
            if (SD_MMC.cardType() != CARD_NONE) {
                sd_available = true;
            }
        }
    }
    if (!sd_available) return;
    
    File f = SD_MMC.open("/settings.txt", FILE_WRITE);
    if (!f) return;
    
    f.printf("led_on=%d\n", led_on ? 1 : 0);
    f.printf("led_br=%d\n", led_br);
    f.printf("led_r=%d\n", led_r);
    f.printf("led_g=%d\n", led_g);
    f.printf("led_b=%d\n", led_b);
    f.printf("thr_auto=%d\n", thr_auto ? 1 : 0);
    f.printf("thr_manual_r=%d\n", thr_manual_r);
    f.printf("thr_manual_g=%d\n", thr_manual_g);
    f.printf("thr_manual_b=%d\n", thr_manual_b);
    f.printf("show_r=%d\n", show_r ? 1 : 0);
    f.printf("show_g=%d\n", show_g ? 1 : 0);
    f.printf("show_b=%d\n", show_b ? 1 : 0);
    f.close();
}

void sendCommand() {
    char buf[128];
    int len = snprintf(buf, sizeof(buf), "L,%d,%d,%d,%d,%d\n", led_on ? 1 : 0, led_r, led_g, led_b, led_br);
    usbSerial.write((uint8_t*)buf, len);
    
    // Slight delay to ensure buffer processing
    delay(5);
    
    len = snprintf(buf, sizeof(buf), "T,%d,%d,%d,%d\n", thr_auto ? 1 : 0, thr_manual_r, thr_manual_g, thr_manual_b);
    usbSerial.write((uint8_t*)buf, len);
}

// UI Helpers
void drawButton(int x, int y, int w, int h, bool state, const char* label, uint32_t color_on) {
    M5.Display.fillRoundRect(x, y, w, h, 12, state ? color_on : C_DARKGREY);
    M5.Display.drawRoundRect(x, y, w, h, 12, C_WHITE);
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(C_WHITE, state ? color_on : C_DARKGREY);
    M5.Display.drawString(label, x + w / 2, y + h / 2);
}

void drawHSlider(int x, int y, int w, int h, int val, const char* label, uint32_t color) {
    M5.Display.drawRect(x, y, w, h, C_WHITE);
    int bar_w = (val * (w - 4) / 255);
    if (bar_w > 0) M5.Display.fillRect(x + 2, y + 2, bar_w, h - 4, color);
    if (bar_w < w - 4) M5.Display.fillRect(x + 2 + bar_w, y + 2, w - 4 - bar_w, h - 4, C_BLACK);
    
    M5.Display.setTextDatum(middle_center);
    M5.Display.setTextColor(C_WHITE);
    M5.Display.drawString(label, x + w / 2 + 1, y + h / 2 + 1);
    M5.Display.drawString(label, x + w / 2 - 1, y + h / 2 - 1);
    M5.Display.setTextColor(C_BLACK);
    M5.Display.drawString(label, x + w / 2, y + h / 2);
}

void drawGraph() {
    graphCanvas.fillSprite(C_DARKGREY);
    if (show_r) graphCanvas.drawFastHLine(0, GRAPH_H - (r_thr * GRAPH_H / 255), GRAPH_W, C_RED);
    if (show_g) graphCanvas.drawFastHLine(0, GRAPH_H - (g_thr * GRAPH_H / 255), GRAPH_W, C_GREEN);
    if (show_b) graphCanvas.drawFastHLine(0, GRAPH_H - (b_thr * GRAPH_H / 255), GRAPH_W, C_BLUE);

    for (int i = 0; i < TOTAL_POINTS - 1; i++) {
        int x1 = (i * GRAPH_W / TOTAL_POINTS);
        int x2 = ((i + 1) * GRAPH_W / TOTAL_POINTS);
        if (show_r) graphCanvas.drawLine(x1, GRAPH_H - (r_samples[i] * GRAPH_H / 255), x2, GRAPH_H - (r_samples[i+1] * GRAPH_H / 255), C_RED);
        if (show_g) graphCanvas.drawLine(x1, GRAPH_H - (g_samples[i] * GRAPH_H / 255), x2, GRAPH_H - (g_samples[i+1] * GRAPH_H / 255), C_GREEN);
        if (show_b) graphCanvas.drawLine(x1, GRAPH_H - (b_samples[i] * GRAPH_H / 255), x2, GRAPH_H - (b_samples[i+1] * GRAPH_H / 255), C_BLUE);
    }
    
    // Original 10-point width box
    if (marker_pos >= 0) {
        for (int m = 0; m < 4; m++) {
            float pos = fmod(marker_pos * 10.0f + phase_offset + m * 10.0f, TOTAL_POINTS);
            int mx = (int)(pos * GRAPH_W / TOTAL_POINTS);
            int mw = (10 * GRAPH_W / TOTAL_POINTS);

            graphCanvas.drawRect(mx, 0, mw, GRAPH_H, C_YELLOW);
            graphCanvas.drawRect(mx + 1, 1, mw - 2, GRAPH_H - 2, C_YELLOW);
            
            if (mx + mw > GRAPH_W) {
                int over = (mx + mw) - GRAPH_W;
                graphCanvas.drawRect(0, 0, over, GRAPH_H, C_YELLOW);
                graphCanvas.drawRect(1, 1, over - 2, GRAPH_H - 2, C_YELLOW);
            }
        }
    }
    graphCanvas.pushSprite(GRAPH_X, GRAPH_Y);
}

void drawHeader() {
    M5.Display.fillRect(0, 0, 1280, 100, C_BLACK);
    
    M5.Display.setFont(&fonts::FreeSansBold18pt7b);
    M5.Display.setTextColor(C_GREEN, C_BLACK);
    M5.Display.setCursor(40, 40);
    if (decoded_once) {
        M5.Display.printf("LOT: %03d   SN: %08u", lot, sn);
        
        // Debug: Print raw 36-bit G sequence aligned to marker_pos
        M5.Display.setFont(&fonts::FreeMono12pt7b);
        M5.Display.setCursor(40, 80);
        M5.Display.setTextColor(C_WHITE, C_BLACK);
        M5.Display.print("RAW(G): ");
        if (marker_pos >= 0) {
            for (int i = 0; i < BITS_COUNT; i++) {
                int idx = (marker_pos + i) % BITS_COUNT;
                if (i < 4) M5.Display.setTextColor(C_RED, C_BLACK); // Highlight marker region in red
                else M5.Display.setTextColor(C_GREEN, C_BLACK);     // Data in green
                M5.Display.print(g_bits_arr[idx] ? "1" : "0");
                if (i == 3 || i == 11) M5.Display.print(" "); // Spacers for readability
            }
        }
    } else {
        M5.Display.printf("WAITING FOR CHIP...");
    }

    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    bool is_saving = (save_anim_time > 0);

    // SD Status indicator
    if (sd_available) {
        M5.Display.setTextColor(C_GREEN);
        M5.Display.drawString("SD OK", 620, 50);
    } else {
        M5.Display.setTextColor(C_RED);
        M5.Display.drawString("NO SD", 620, 50);
    }

    drawButton(720, 20, 100, 60, is_saving, is_saving ? "SAVED" : "SAVE", is_saving ? C_GREEN : C_YELLOW);
    drawButton(850, 20, 100, 60, show_r, "R", C_RED);
    drawButton(980, 20, 100, 60, show_g, "G", C_GREEN);
    drawButton(1110, 20, 100, 60, show_b, "B", C_BLUE);
}

void drawAllUI() {
    M5.Display.setFont(&fonts::FreeSansBold12pt7b);
    
    // Row 1: LED Controls (Y=520)
    drawButton(20, 520, 140, 70, led_on, led_on ? "LED ON" : "LED OFF", C_GREEN);
    drawHSlider(180, 520, 300, 70, led_br, "BRIGHT", C_WHITE);
    drawHSlider(500, 520, 240, 70, led_r, "L-R", C_RED);
    drawHSlider(760, 520, 240, 70, led_g, "L-G", C_GREEN);
    drawHSlider(1020, 520, 240, 70, led_b, "L-B", C_BLUE);

    // Row 2: Threshold Controls (Y=620)
    drawButton(20, 620, 140, 70, thr_auto, thr_auto ? "AUTO" : "MAN", C_YELLOW);
    drawHSlider(180, 620, 340, 70, thr_manual_r, "THR R", C_RED);
    drawHSlider(540, 620, 340, 70, thr_manual_g, "THR G", C_GREEN);
    drawHSlider(900, 620, 340, 70, thr_manual_b, "THR B", C_BLUE);
}

void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    
    Serial.begin(115200); 

    // SD MMC Init for Tab5 (P4)
    if (SD_MMC.setPins(43, 44, 39, 40, 41, 42)) {
        if (SD_MMC.begin("/sdcard", false)) {
            if (SD_MMC.cardType() != CARD_NONE) {
                sd_available = true;
                loadSettings();
            }
        }
    }

    // Enable USB Host power for Tab5
    M5.Power.setExtOutput(true);
    M5.Power.setUsbOutput(true);
    delay(500); 
    
    esp_usb::VCP::register_driver<esp_usb::Esp32S3CDC>();
    usbSerial.begin(115200, 1, 0, 8); // match new baud rate
    
    M5.Display.setRotation(1); // Usually sets landscape mapping, Tab5 is natively 1280x720
    M5.Display.fillScreen(C_BLACK);

    // Large canvas: allocate in PSRAM
    graphCanvas.setPsram(true); 
    graphCanvas.createSprite(GRAPH_W, GRAPH_H);

    drawHeader();
    drawAllUI();
}

std::vector<int> parseCSVInts(const String& s, int startIdx) {
    std::vector<int> res;
    int idx = startIdx;
    while (idx < s.length()) {
        int next = s.indexOf(',', idx);
        if (next == -1) {
            res.push_back(s.substring(idx).toInt());
            break;
        } else {
            res.push_back(s.substring(idx, next).toInt());
            idx = next + 1;
        }
    }
    return res;
}

void processDebugLine(const String& line) {
    if (line.startsWith("DATA,R_FILT_SAMP,")) {
        auto vals = parseCSVInts(line, 17);
        for (int i = 0; i < min((int)vals.size(), TOTAL_POINTS); i++) r_samples[i] = vals[i];
    } else if (line.startsWith("DATA,G_FILT_SAMP,")) {
        auto vals = parseCSVInts(line, 17);
        for (int i = 0; i < min((int)vals.size(), TOTAL_POINTS); i++) g_samples[i] = vals[i];
    } else if (line.startsWith("DATA,B_FILT_SAMP,")) {
        auto vals = parseCSVInts(line, 17);
        for (int i = 0; i < min((int)vals.size(), TOTAL_POINTS); i++) b_samples[i] = vals[i];
    } else if (line.startsWith("DATA,R_BITS,")) {
        String bits = line.substring(12);
        for (int i = 0; i < min((int)bits.length(), BITS_COUNT); i++) r_bits_arr[i] = bits[i] == '1' ? 1 : 0;
    } else if (line.startsWith("DATA,G_BITS,")) {
        String bits = line.substring(12);
        for (int i = 0; i < min((int)bits.length(), BITS_COUNT); i++) g_bits_arr[i] = bits[i] == '1' ? 1 : 0;
    } else if (line.startsWith("DATA,B_BITS,")) {
        String bits = line.substring(12);
        for (int i = 0; i < min((int)bits.length(), BITS_COUNT); i++) b_bits_arr[i] = bits[i] == '1' ? 1 : 0;
    } else if (line.startsWith("DATA,R_THRESHOLD,")) {
        r_thr = line.substring(17).toFloat();
        if (thr_auto) thr_manual_r = (int)r_thr;
    } else if (line.startsWith("DATA,G_THRESHOLD,")) {
        g_thr = line.substring(17).toFloat();
        if (thr_auto) thr_manual_g = (int)g_thr;
    } else if (line.startsWith("DATA,B_THRESHOLD,")) {
        b_thr = line.substring(17).toFloat();
        if (thr_auto) thr_manual_b = (int)b_thr;
    } else if (line.startsWith("DATA,PHASE_OFFSET,")) {
        phase_offset = line.substring(18).toFloat();
    } else if (line.startsWith("DATA,MARKER_POS,")) {
        marker_pos = line.substring(16).toInt();
    } else if (line.startsWith("DATA,DECODE,")) {
        int comma = line.indexOf(',', 12);
        if (comma != -1) {
            lot = line.substring(12, comma).toInt();
            sn = line.substring(comma + 1).toInt();
            decoded_once = true;
        }
    } else if (line == "--- DEBUG END ---") {
        if (!initial_sync_done) {
            sendCommand();
            initial_sync_done = true;
        }
        M5.Display.startWrite();
        drawHeader();
        drawGraph();
        drawAllUI();
        M5.Display.endWrite();
    }
}

String readBuf = "";

void handleUSB() {
    while (usbSerial.available()) {
        char c = usbSerial.read();
        if (c == '\n') {
            readBuf.trim();
            processDebugLine(readBuf);
            readBuf = "";
        } else if (c != '\r') {
            readBuf += c;
            // Prevent OOM
            if (readBuf.length() > 2048) readBuf = "";
        }
    }
}

void loop() {
    M5.update();
    
    // Save animation reset
    if (save_anim_time > 0 && millis() - save_anim_time > 500) {
        save_anim_time = 0;
        M5.Display.startWrite();
        drawHeader();
        M5.Display.endWrite();
    }

    // --- Touch UI ---
    static bool was_touched = false;
    bool is_touched = (M5.Touch.getCount() > 0);
    bool just_pressed = (is_touched && !was_touched);
    was_touched = is_touched;

    if (is_touched) {
        auto t = M5.Touch.getDetail();
        int touch_x = t.x;
        int touch_y = t.y;
        bool ui_changed = false;

        if (just_pressed) {
            // Row 1: LED ON/OFF Button Area
            if (touch_x >= 10 && touch_x <= 170 && touch_y >= 510 && touch_y <= 600) {
                led_on = !led_on;
                ui_changed = true;
            } 
            // Row 2: THR AUTO/MAN Button Area
            else if (touch_x >= 10 && touch_x <= 170 && touch_y >= 610 && touch_y <= 700) {
                thr_auto = !thr_auto;
                ui_changed = true;
            }
            // Header Area
            else if (touch_y >= 10 && touch_y <= 90) {
                if (touch_x >= 710 && touch_x <= 830) {
                    saveSettings();
                    save_anim_time = millis();
                    ui_changed = true; // Trigger redraw
                }
                else if (touch_x >= 840 && touch_x <= 960) { show_r = !show_r; ui_changed = true; }
                else if (touch_x >= 970 && touch_x <= 1090) { show_g = !show_g; ui_changed = true; }
                else if (touch_x >= 1100 && touch_x <= 1220) { show_b = !show_b; ui_changed = true; }
            }
        }
        
        auto checkSlider = [&](int x, int y, int w, int h, int &val, bool is_thr) {
            if (touch_x >= x - 15 && touch_x <= x + w + 15 && touch_y >= y - 15 && touch_y <= y + h + 15) {
                int new_val = constrain((touch_x - x) * 255 / w, 0, 255);
                if (new_val != val) {
                    val = new_val;
                    if (is_thr) thr_auto = false;
                    ui_changed = true;
                }
            }
        };

        // Row 1 Sliders
        checkSlider(180, 520, 300, 70, led_br, false);
        checkSlider(500, 520, 240, 70, led_r, false);
        checkSlider(760, 520, 240, 70, led_g, false);
        checkSlider(1020, 520, 240, 70, led_b, false);

        // Row 2 Sliders
        checkSlider(180, 620, 340, 70, thr_manual_r, true);
        checkSlider(540, 620, 340, 70, thr_manual_g, true);
        checkSlider(900, 620, 340, 70, thr_manual_b, true);

        if (ui_changed) {
            M5.Display.startWrite();
            drawHeader();
            drawGraph();
            drawAllUI();
            M5.Display.endWrite();
            sendCommand();
        }
    }

    // --- USB ---
    handleUSB();
    delay(1);
}
