/*
  ESP32 + Ma trận LED P5 outdoor 64x32 (four-scan 1/8, chỉ A,B,C)
  Nhận dữ liệu qua MQTT (HiveMQ public broker) để hiển thị:
    - Ảnh tĩnh (RGB565 raw)
    - GIF động (có xác nhận từng khung qua topic gif/ack để chống mất gói)
    - Đồng hồ (digital/analog, có ngày dương lịch + âm lịch)
    - Kết hợp: đồng hồ bên trái + ảnh bên phải
    - Xoay hướng màn hình (nằm/đứng) + hiệu ứng chuyển cảnh (fade)
    - Lưu trạng thái/dữ liệu hiển thị vào flash (Preferences + LittleFS)
      để khi ESP32 reboot thật (mất điện, brownout...) sẽ tự khôi phục
      lại đúng nội dung cuối cùng thay vì về mặc định.
    - Chống retained-message cũ: chỉ áp dụng lệnh đổi nội dung nếu web
      vừa gửi "tín hiệu sống" trong vài giây gần đây (topic /live).

  Thư viện cần cài (Library Manager):
    - ESP32-HUB75-MatrixPanel-I2S-DMA
    - ESP32-VirtualMatrixPanel-I2S-DMA
    - PubSubClient (Nick O'Leary)
    - ArduinoJson (Benoit Blanchon) - bản 6.x hoặc 7.x
    - Preferences và LittleFS đã có sẵn trong ESP32 Arduino core

  QUAN TRỌNG: Tools -> Partition Scheme -> chọn loại có "spiffs"
  (ví dụ "Default 4MB with spiffs"), nếu không LittleFS sẽ không mount được
  và phần khôi phục ảnh/GIF sau reset sẽ không hoạt động.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <ESP32-VirtualMatrixPanel-I2S-DMA.h>
#include <time.h>
#include <math.h>
#include <functional>
#include <Preferences.h>
#include <LittleFS.h>

// ================== WIFI ==================
const char* WIFI_SSID = "677 5G";
const char* WIFI_PASS = "10101010";

// ================== MQTT ==================
const char* MQTT_HOST = "broker.hivemq.com";
const int   MQTT_PORT = 1883;

#define TOPIC_PREFIX "hcm_led_matrix_9f3a2b"

String T_MODE       = String(TOPIC_PREFIX) + "/mode";
String T_IMAGE      = String(TOPIC_PREFIX) + "/image";
String T_GIF_META   = String(TOPIC_PREFIX) + "/gif/meta";
String T_GIF_FRAME  = String(TOPIC_PREFIX) + "/gif/frame";
String T_GIF_DONE   = String(TOPIC_PREFIX) + "/gif/done";
String T_GIF_ACK    = String(TOPIC_PREFIX) + "/gif/ack";
String T_CLOCK_CFG  = String(TOPIC_PREFIX) + "/clock/config";
String T_ORIENT     = String(TOPIC_PREFIX) + "/orient";
String T_TRANS      = String(TOPIC_PREFIX) + "/transition";
String T_BRIGHTNESS = String(TOPIC_PREFIX) + "/brightness";
String T_SPLIT_CFG  = String(TOPIC_PREFIX) + "/split/config";
String T_SPLIT_IMG  = String(TOPIC_PREFIX) + "/split/image";
String T_LAYOUT_CFG = String(TOPIC_PREFIX) + "/layout/config";
String T_LAYOUT_IMG = String(TOPIC_PREFIX) + "/layout/image";
String T_LAYOUT_GIF_META  = String(TOPIC_PREFIX) + "/layout/gif/meta";
String T_LAYOUT_GIF_FRAME = String(TOPIC_PREFIX) + "/layout/gif/frame";
String T_LAYOUT_GIF_DONE  = String(TOPIC_PREFIX) + "/layout/gif/done";
String T_LIVE_PING  = String(TOPIC_PREFIX) + "/live"; // ===== MỚI =====

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ================== PANEL LED (giữ nguyên cấu hình gốc) ==================
#define PANEL_RES_X 64
#define PANEL_RES_Y 32
#define NUM_ROWS 1
#define NUM_COLS 1
#define PANEL_CHAIN (NUM_ROWS * NUM_COLS)
#define TONG_SO_DIEM_ANH (PANEL_RES_X * PANEL_RES_Y) // không đổi dù xoay hướng nào

#define R1_PIN  25
#define G1_PIN  26
#define B1_PIN  27
#define R2_PIN  14
#define G2_PIN  12
#define B2_PIN  13
#define A_PIN   23
#define B_PIN   19
#define C_PIN   5
#define D_PIN   -1
#define E_PIN   -1
#define LAT_PIN 4
#define OE_PIN  15
#define CLK_PIN 16

MatrixPanel_I2S_DMA *dma_display = nullptr;
VirtualMatrixPanel  *matrix      = nullptr;

// ================== HƯỚNG MÀN HÌNH ==================
int huongManHinh = 0;     // 0 = nằm ngang (64x32), 1 = đứng (32x64)
int W_hienTai = PANEL_RES_X;
int H_hienTai = PANEL_RES_Y;

// ================== HIỆU ỨNG CHUYỂN CẢNH ==================
int thoiGianChuyenCanh = 300; // ms
uint8_t doSangHienTai = 40;

// ================== TRẠNG THÁI HIỂN THỊ ==================
enum CheDoHienThi { CHEDO_ANH, CHEDO_GIF, CHEDO_DONGHO, CHEDO_SPLIT, CHEDO_LAYOUT };
CheDoHienThi cheDoHienTai = CHEDO_DONGHO;

// ---- Bộ đệm ảnh tĩnh ----
uint16_t bufferAnh[TONG_SO_DIEM_ANH];

// ---- Bộ đệm GIF ----
#define MAX_GIF_FRAMES 40
uint16_t* gifFrames[MAX_GIF_FRAMES] = { nullptr };
bool gifFrameDaNhan[MAX_GIF_FRAMES] = { false };
int soFrameGif = 0;
int gifDelayMs = 100;
int gifFrameHienTai = 0;
uint32_t gifLastUpdate = 0;
bool gifSanSang = false;
bool gifVuaBatDau = false; // để áp hiệu ứng chuyển cảnh ở khung đầu tiên

// ---- Cấu hình đồng hồ ----
String kieuDongHo = "digital";
uint16_t mauDongHo = 0x07E0;      // xanh lá mặc định
uint16_t mauNenDongHo = 0x0000;   // đen
bool hienNgayDuong = true;
bool hienNgayAm = true;
uint32_t lastClockDraw = 0;

// ---- Cấu hình chế độ kết hợp ----
String kieuDongHoSplit = "digital";
uint16_t mauDongHoSplit = 0x07E0;
uint16_t* splitAnhBuf = nullptr; // ảnh cho nửa phải, kích thước (W_hienTai - W_hienTai/2) * H_hienTai
int splitAnhRong = 0;

// ---- Cau hinh bo cuc tu do ----
struct LayoutClockCfg {
  bool enabled = true;
  int x = 2, y = 2, w = 30, h = 20;
  String style = "boxed";
  uint16_t color = 0x07E0;
  uint16_t bgColor = 0x0000;
  bool showDate = true;
  bool showLunar = false;
} layoutClock;

struct LayoutTextCfg {
  bool enabled = false;
  int x = 0, y = 24, w = 24, h = 8;
  String content = "";
  uint16_t color = 0xFFFF;
  uint16_t bgColor = 0x0000;
  String align = "center";
  String effect = "static";
  int speed = 4;
} layoutText;

struct LayoutImageCfg {
  bool enabled = false;
  bool isGif = false;
  int x = 36, y = 2, w = 24, h = 20;
} layoutImage;

uint16_t* layoutImageBuf = nullptr;
int layoutImageW = 0;
int layoutImageH = 0;

#define MAX_LAYOUT_GIF_FRAMES 60
uint16_t* layoutGifFrames[MAX_LAYOUT_GIF_FRAMES] = { nullptr };
bool layoutGifDaNhan[MAX_LAYOUT_GIF_FRAMES] = { false };
int layoutGifSoFrame = 0;
int layoutGifDelayMs = 100;
int layoutGifFrameHienTai = 0;
uint32_t layoutGifLastUpdate = 0;
bool layoutGifSanSang = false;
int layoutGifW = 0;
int layoutGifH = 0;

int layoutTextOffsetX = 0;
int layoutTextOffsetY = 0;
uint32_t layoutTextLastStep = 0;
bool layoutTextBlinkOn = true;
int layoutTextDirX = -1;
int layoutTextTypedChars = 0;

// =========================================================
// ===== CHỐNG RETAINED MESSAGE CŨ: "TÍN HIỆU SỐNG" =====
//   Web chỉ gửi topic /live khi có người thao tác thật.
//   ESP32 chỉ áp dụng lệnh đổi nội dung nếu tín hiệu sống
//   vừa đến trong vài giây gần đây. Tin nhắn retained mà
//   broker tự phát lại lúc reconnect sẽ KHÔNG có tín hiệu
//   sống đi kèm nên bị bỏ qua.
// =========================================================
unsigned long lanCuoiNhanTinHieuSong = 0;
const unsigned long CUA_SO_TIN_HIEU_SONG_MS = 4000; // 4 giay

bool laLenhConTuoi() {
  return (millis() - lanCuoiNhanTinHieuSong) <= CUA_SO_TIN_HIEU_SONG_MS;
}

// =========================================================
// LƯU / KHÔI PHỤC TRẠNG THÁI VÀO FLASH
//   - Preferences (NVS): cấu hình nhỏ (mode, orient, clock, layout config...)
//   - LittleFS: dữ liệu nhị phân lớn (ảnh, khung GIF...)
//   Chỉ ghi flash khi dữ liệu ĐÃ NHẬN XONG (không ghi từng khung lẻ tẻ)
//   để tránh mòn flash khi nhận GIF liên tục.
// =========================================================
Preferences prefs;

#define FS_IMG_PATH        "/img.bin"
#define FS_SPLIT_IMG_PATH  "/split_img.bin"
#define FS_LAYOUT_IMG_PATH "/layout_img.bin"

String duongDanKhungGif(int idx)       { return "/gif_" + String(idx) + ".bin"; }
String duongDanKhungLayoutGif(int idx) { return "/lgif_" + String(idx) + ".bin"; }

bool ghiBufferVaoFlash(const String& path, uint16_t* buf, size_t soPhanTu) {
  if (buf == nullptr || soPhanTu == 0) return false;
  File f = LittleFS.open(path, "w");
  if (!f) { Serial.println("Loi mo file ghi: " + path); return false; }
  size_t viet = f.write((uint8_t*)buf, soPhanTu * sizeof(uint16_t));
  f.close();
  return viet == soPhanTu * sizeof(uint16_t);
}

bool docBufferTuFlash(const String& path, uint16_t* buf, size_t soPhanTu) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = (f.size() == soPhanTu * sizeof(uint16_t));
  if (ok) f.read((uint8_t*)buf, soPhanTu * sizeof(uint16_t));
  f.close();
  return ok;
}

void luuCauHinhChung() {
  prefs.begin("ledcfg", false);
  prefs.putInt("mode", (int)cheDoHienTai);
  prefs.putInt("orient", huongManHinh);
  prefs.putUChar("bright", doSangHienTai);
  prefs.putInt("trans", thoiGianChuyenCanh);

  prefs.putString("clkStyle", kieuDongHo);
  prefs.putUShort("clkColor", mauDongHo);
  prefs.putUShort("clkBg", mauNenDongHo);
  prefs.putBool("clkDate", hienNgayDuong);
  prefs.putBool("clkLunar", hienNgayAm);

  prefs.putString("spStyle", kieuDongHoSplit);
  prefs.putUShort("spColor", mauDongHoSplit);
  prefs.putInt("spRong", splitAnhRong);

  prefs.putBool("lcEn", layoutClock.enabled);
  prefs.putInt("lcX", layoutClock.x);   prefs.putInt("lcY", layoutClock.y);
  prefs.putInt("lcW", layoutClock.w);   prefs.putInt("lcH", layoutClock.h);
  prefs.putString("lcStyle", layoutClock.style);
  prefs.putUShort("lcColor", layoutClock.color);
  prefs.putUShort("lcBg", layoutClock.bgColor);
  prefs.putBool("lcDate", layoutClock.showDate);
  prefs.putBool("lcLunar", layoutClock.showLunar);

  prefs.putBool("ltEn", layoutText.enabled);
  prefs.putInt("ltX", layoutText.x);   prefs.putInt("ltY", layoutText.y);
  prefs.putInt("ltW", layoutText.w);   prefs.putInt("ltH", layoutText.h);
  prefs.putString("ltContent", layoutText.content);
  prefs.putUShort("ltColor", layoutText.color);
  prefs.putUShort("ltBg", layoutText.bgColor);
  prefs.putString("ltAlign", layoutText.align);
  prefs.putString("ltEffect", layoutText.effect);
  prefs.putInt("ltSpeed", layoutText.speed);

  prefs.putBool("liEn", layoutImage.enabled);
  prefs.putBool("liGif", layoutImage.isGif);
  prefs.putInt("liX", layoutImage.x); prefs.putInt("liY", layoutImage.y);
  prefs.putInt("liW", layoutImage.w); prefs.putInt("liH", layoutImage.h);
  prefs.putInt("liBufW", layoutImageW);
  prefs.putInt("liBufH", layoutImageH);

  prefs.putInt("lgFrames", layoutGifSoFrame);
  prefs.putInt("lgDelay", layoutGifDelayMs);
  prefs.putInt("lgW", layoutGifW);
  prefs.putInt("lgH", layoutGifH);

  prefs.putInt("gifFrames", soFrameGif);
  prefs.putInt("gifDelay", gifDelayMs);
  prefs.end();
}

void taiCauHinhChung() {
  prefs.begin("ledcfg", true); // read-only
  cheDoHienTai = (CheDoHienThi)prefs.getInt("mode", CHEDO_DONGHO);
  huongManHinh = prefs.getInt("orient", 0);
  doSangHienTai = prefs.getUChar("bright", 40);
  thoiGianChuyenCanh = prefs.getInt("trans", 300);

  kieuDongHo = prefs.getString("clkStyle", "digital");
  mauDongHo = prefs.getUShort("clkColor", 0x07E0);
  mauNenDongHo = prefs.getUShort("clkBg", 0x0000);
  hienNgayDuong = prefs.getBool("clkDate", true);
  hienNgayAm = prefs.getBool("clkLunar", true);

  kieuDongHoSplit = prefs.getString("spStyle", "digital");
  mauDongHoSplit = prefs.getUShort("spColor", 0x07E0);
  splitAnhRong = prefs.getInt("spRong", 0);

  layoutClock.enabled = prefs.getBool("lcEn", true);
  layoutClock.x = prefs.getInt("lcX", 2);   layoutClock.y = prefs.getInt("lcY", 2);
  layoutClock.w = prefs.getInt("lcW", 30);  layoutClock.h = prefs.getInt("lcH", 20);
  layoutClock.style = prefs.getString("lcStyle", "boxed");
  layoutClock.color = prefs.getUShort("lcColor", 0x07E0);
  layoutClock.bgColor = prefs.getUShort("lcBg", 0x0000);
  layoutClock.showDate = prefs.getBool("lcDate", true);
  layoutClock.showLunar = prefs.getBool("lcLunar", false);

  layoutText.enabled = prefs.getBool("ltEn", false);
  layoutText.x = prefs.getInt("ltX", 0);   layoutText.y = prefs.getInt("ltY", 24);
  layoutText.w = prefs.getInt("ltW", 24);  layoutText.h = prefs.getInt("ltH", 8);
  layoutText.content = prefs.getString("ltContent", "");
  layoutText.color = prefs.getUShort("ltColor", 0xFFFF);
  layoutText.bgColor = prefs.getUShort("ltBg", 0x0000);
  layoutText.align = prefs.getString("ltAlign", "center");
  layoutText.effect = prefs.getString("ltEffect", "static");
  layoutText.speed = prefs.getInt("ltSpeed", 4);

  layoutImage.enabled = prefs.getBool("liEn", false);
  layoutImage.isGif = prefs.getBool("liGif", false);
  layoutImage.x = prefs.getInt("liX", 36); layoutImage.y = prefs.getInt("liY", 2);
  layoutImage.w = prefs.getInt("liW", 24); layoutImage.h = prefs.getInt("liH", 20);
  layoutImageW = prefs.getInt("liBufW", 0);
  layoutImageH = prefs.getInt("liBufH", 0);

  layoutGifSoFrame = prefs.getInt("lgFrames", 0);
  layoutGifDelayMs = prefs.getInt("lgDelay", 100);
  layoutGifW = prefs.getInt("lgW", 0);
  layoutGifH = prefs.getInt("lgH", 0);

  soFrameGif = prefs.getInt("gifFrames", 0);
  gifDelayMs = prefs.getInt("gifDelay", 100);
  prefs.end();
}

void luuAnhTinhVaoFlash() { ghiBufferVaoFlash(FS_IMG_PATH, bufferAnh, TONG_SO_DIEM_ANH); }
bool taiAnhTinhTuFlash()  { return docBufferTuFlash(FS_IMG_PATH, bufferAnh, TONG_SO_DIEM_ANH); }

void luuGifVaoFlash() {
  if (soFrameGif <= 0) return;
  int soLoi = 0;
  for (int i = 0; i < soFrameGif; i++) {
    if (gifFrames[i]) {
      if (!ghiBufferVaoFlash(duongDanKhungGif(i), gifFrames[i], TONG_SO_DIEM_ANH)) soLoi++;
    }
  }
  Serial.printf("Da ghi GIF vao flash: %d/%d frame loi\n", soLoi, soFrameGif);
}
bool taiGifTuFlash() {
  if (soFrameGif <= 0) return false;
  int soFrameCanTai = min(soFrameGif, MAX_GIF_FRAMES);
  xoaBufferGif(); // hàm này reset soFrameGif về 0 nên phải gán lại
  soFrameGif = soFrameCanTai;
  bool coDuKhung = true;
  for (int i = 0; i < soFrameGif; i++) {
    gifFrames[i] = (uint16_t*)calloc(TONG_SO_DIEM_ANH, sizeof(uint16_t));
    if (!docBufferTuFlash(duongDanKhungGif(i), gifFrames[i], TONG_SO_DIEM_ANH)) coDuKhung = false;
    gifFrameDaNhan[i] = true;
  }
  gifSanSang = coDuKhung;
  gifFrameHienTai = 0;
  return coDuKhung;
}

void luuAnhSplitVaoFlash() {
  if (splitAnhBuf != nullptr && splitAnhRong > 0) {
    ghiBufferVaoFlash(FS_SPLIT_IMG_PATH, splitAnhBuf, (size_t)splitAnhRong * H_hienTai);
  }
}
bool taiAnhSplitTuFlash() {
  if (splitAnhRong <= 0) return false;
  size_t tong = (size_t)splitAnhRong * H_hienTai;
  if (splitAnhBuf != nullptr) free(splitAnhBuf);
  splitAnhBuf = (uint16_t*)malloc(tong * sizeof(uint16_t));
  if (splitAnhBuf == nullptr) return false;
  return docBufferTuFlash(FS_SPLIT_IMG_PATH, splitAnhBuf, tong);
}

void luuLayoutImageVaoFlash() {
  if (layoutImageBuf != nullptr && layoutImageW > 0 && layoutImageH > 0) {
    ghiBufferVaoFlash(FS_LAYOUT_IMG_PATH, layoutImageBuf, (size_t)layoutImageW * layoutImageH);
  }
}
bool taiLayoutImageTuFlash() {
  if (layoutImageW <= 0 || layoutImageH <= 0) return false;
  size_t tong = (size_t)layoutImageW * layoutImageH;
  if (layoutImageBuf != nullptr) free(layoutImageBuf);
  layoutImageBuf = (uint16_t*)malloc(tong * sizeof(uint16_t));
  if (layoutImageBuf == nullptr) return false;
  return docBufferTuFlash(FS_LAYOUT_IMG_PATH, layoutImageBuf, tong);
}

void luuLayoutGifVaoFlash() {
  if (layoutGifSoFrame <= 0 || layoutGifW <= 0 || layoutGifH <= 0) return;
  size_t diem = (size_t)layoutGifW * layoutGifH;
  for (int i = 0; i < layoutGifSoFrame; i++) {
    if (layoutGifFrames[i]) ghiBufferVaoFlash(duongDanKhungLayoutGif(i), layoutGifFrames[i], diem);
  }
}
bool taiLayoutGifTuFlash() {
  if (layoutGifSoFrame <= 0 || layoutGifW <= 0 || layoutGifH <= 0) return false;
  int soFrameCanTai = min(layoutGifSoFrame, MAX_LAYOUT_GIF_FRAMES);
  int w = layoutGifW, h = layoutGifH, delayMs = layoutGifDelayMs;
  xoaLayoutGif(); // reset các biến, phải gán lại sau
  layoutGifSoFrame = soFrameCanTai;
  layoutGifW = w; layoutGifH = h; layoutGifDelayMs = delayMs;
  size_t diem = (size_t)w * h;
  bool coDuKhung = true;
  for (int i = 0; i < layoutGifSoFrame; i++) {
    layoutGifFrames[i] = (uint16_t*)calloc(diem, sizeof(uint16_t));
    if (!docBufferTuFlash(duongDanKhungLayoutGif(i), layoutGifFrames[i], diem)) coDuKhung = false;
    layoutGifDaNhan[i] = true;
  }
  layoutGifSanSang = coDuKhung;
  layoutGifFrameHienTai = 0;
  return coDuKhung;
}

// =========================================================
// TIỆN ÍCH MÀU
// =========================================================
uint16_t hexColorToRGB565(const String& hexStr) {
  long v = strtol(hexStr.c_str() + 1, nullptr, 16);
  uint8_t r = (v >> 16) & 0xFF, g = (v >> 8) & 0xFF, b = v & 0xFF;
  return matrix->color565(r, g, b);
}

// =========================================================
// XOAY MÀN HÌNH
// =========================================================
void capNhatHuongManHinh(int huong) {
  huongManHinh = huong;
  matrix->setRotation(huong); // Adafruit_GFX: 0 hoặc 1 (90 độ)
  W_hienTai = matrix->width();
  H_hienTai = matrix->height();
  Serial.printf("Huong man hinh: %d -> %dx%d\n", huong, W_hienTai, H_hienTai);
}

// =========================================================
// HIỆU ỨNG CHUYỂN CẢNH (fade tối -> vẽ nội dung mới -> fade sáng)
// =========================================================
void chuyenCanh(std::function<void()> veNoiDungMoi) {
  if (thoiGianChuyenCanh <= 0) { veNoiDungMoi(); return; }
  const int buoc = 10;
  int moiBuoc = max(2, thoiGianChuyenCanh / (buoc * 2));
  for (int i = buoc; i >= 0; i--) { dma_display->setBrightness8((doSangHienTai * i) / buoc); delay(moiBuoc); }
  veNoiDungMoi();
  for (int i = 0; i <= buoc; i++) { dma_display->setBrightness8((doSangHienTai * i) / buoc); delay(moiBuoc); }
}

// =========================================================
// GIẢI PHÓNG BUFFER GIF CŨ
// =========================================================
void xoaBufferGif() {
  for (int i = 0; i < MAX_GIF_FRAMES; i++) {
    if (gifFrames[i] != nullptr) { free(gifFrames[i]); gifFrames[i] = nullptr; }
    gifFrameDaNhan[i] = false;
  }
  soFrameGif = 0;
  gifFrameHienTai = 0;
  gifSanSang = false;
}

void xoaLayoutImage() {
  if (layoutImageBuf != nullptr) {
    free(layoutImageBuf);
    layoutImageBuf = nullptr;
  }
  layoutImageW = 0;
  layoutImageH = 0;
}

void xoaLayoutGif() {
  for (int i = 0; i < MAX_LAYOUT_GIF_FRAMES; i++) {
    if (layoutGifFrames[i] != nullptr) {
      free(layoutGifFrames[i]);
      layoutGifFrames[i] = nullptr;
    }
    layoutGifDaNhan[i] = false;
  }
  layoutGifSoFrame = 0;
  layoutGifDelayMs = 100;
  layoutGifFrameHienTai = 0;
  layoutGifLastUpdate = 0;
  layoutGifSanSang = false;
  layoutGifW = 0;
  layoutGifH = 0;
}

int gioiHan(int v, int minV, int maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

// =========================================================
// VẼ 1 BUFFER RGB565 LÊN MATRIX (dùng kích thước hiện tại theo hướng)
// =========================================================
void veBufferLenMatrix(uint16_t* buf, int wBuf, int hBuf, int xOffset = 0) {
  for (int y = 0; y < hBuf; y++) {
    for (int x = 0; x < wBuf; x++) {
      matrix->drawPixel(x + xOffset, y, buf[y * wBuf + x]);
    }
  }
}

// =========================================================
// CHUYỂN ĐỔI ÂM LỊCH (thuật toán Hồ Ngọc Đức, múi giờ VN +7)
// =========================================================
long jdFromDate(int dd, int mm, int yy) {
  long a = (14 - mm) / 12, y = yy + 4800 - a, m = mm + 12 * a - 3;
  long jd = dd + (153 * m + 2) / 5 + 365 * y + y / 4 - y / 100 + y / 400 - 32045;
  if (jd < 2299161) jd = dd + (153 * m + 2) / 5 + 365 * y + y / 4 - 32083;
  return jd;
}
double NewMoon(int k) {
  double T = k / 1236.85, T2 = T * T, T3 = T2 * T, dr = PI / 180;
  double Jd1 = 2415020.75933 + 29.53058868 * k + 0.0001178 * T2 - 0.000000155 * T3;
  Jd1 += 0.00033 * sin((166.56 + 132.87 * T - 0.009173 * T2) * dr);
  double M = 359.2242 + 29.10535608 * k - 0.0000333 * T2 - 0.00000347 * T3;
  double Mpr = 306.0253 + 385.81691806 * k + 0.0107306 * T2 + 0.00001236 * T3;
  double F = 21.2964 + 390.67050646 * k - 0.0016528 * T2 - 0.00000239 * T3;
  double C1 = (0.1734 - 0.000393 * T) * sin(M * dr) + 0.0021 * sin(2 * dr * M);
  C1 -= 0.4068 * sin(Mpr * dr) + 0.0161 * sin(dr * 2 * Mpr);
  C1 -= 0.0004 * sin(dr * 3 * Mpr);
  C1 += 0.0104 * sin(dr * 2 * F) - 0.0051 * sin(dr * (M + Mpr));
  C1 -= 0.0074 * sin(dr * (M - Mpr)) + 0.0004 * sin(dr * (2 * F + M));
  C1 -= 0.0004 * sin(dr * (2 * F - M)) - 0.0006 * sin(dr * (2 * F + Mpr));
  C1 += 0.0010 * sin(dr * (2 * F - Mpr)) + 0.0005 * sin(dr * (2 * Mpr + M));
  double deltat = (T < -11) ? (0.001 + 0.000839 * T + 0.0002261 * T2 - 0.00000845 * T3 - 0.000000081 * T * T3)
                             : (-0.000278 + 0.000265 * T + 0.000262 * T2);
  return Jd1 + C1 - deltat;
}
double SunLongitude(double jdn) {
  double T = (jdn - 2451545.0) / 36525, T2 = T * T, dr = PI / 180;
  double M = 357.52910 + 35999.05030 * T - 0.0001559 * T2 - 0.00000048 * T * T2;
  double L0 = 280.46645 + 36000.76983 * T + 0.0003032 * T2;
  double DL = (1.914600 - 0.004817 * T - 0.000014 * T2) * sin(dr * M);
  DL += (0.019993 - 0.000101 * T) * sin(dr * 2 * M) + 0.000290 * sin(dr * 3 * M);
  double L = (L0 + DL) * dr;
  L -= PI * 2 * (floor(L / (PI * 2)));
  return L;
}
int getSunLongitude(long dayNumber, int tz) { return (int)(SunLongitude(dayNumber - 0.5 - tz / 24.0) / PI * 6); }
long getNewMoonDay(int k, int tz) { return (long)floor(NewMoon(k) + 0.5 + tz / 24.0); }
long getLunarMonth11(int yy, int tz) {
  double off = jdFromDate(31, 12, yy) - 2415021.076998695;
  int k = (int)floor(off / 29.530588853);
  long nm = getNewMoonDay(k, tz);
  if (getSunLongitude(nm, tz) >= 9) nm = getNewMoonDay(k - 1, tz);
  return nm;
}
int getLeapMonthOffset(long a11, int tz) {
  int k = (int)floor((a11 - 2415021.076998695) / 29.530588853 + 0.5);
  int last = 0, i = 1, arc = getSunLongitude(getNewMoonDay(k + i, tz), tz);
  do { last = arc; i++; arc = getSunLongitude(getNewMoonDay(k + i, tz), tz); } while (arc != last && i < 14);
  return i - 1;
}
void convertSolar2Lunar(int dd, int mm, int yy, int tz, int &ld, int &lm, int &ly, bool &leap) {
  long dayNumber = jdFromDate(dd, mm, yy);
  int k = (int)floor((dayNumber - 2415021.076998695) / 29.530588853);
  long monthStart = getNewMoonDay(k + 1, tz);
  if (monthStart > dayNumber) monthStart = getNewMoonDay(k, tz);
  long a11 = getLunarMonth11(yy, tz), b11 = a11;
  int lunarY;
  if (a11 >= monthStart) { lunarY = yy; a11 = getLunarMonth11(yy - 1, tz); }
  else { lunarY = yy + 1; b11 = getLunarMonth11(yy + 1, tz); }
  ld = dayNumber - monthStart + 1;
  long diff = (monthStart - a11) / 29;
  bool leapFlag = false;
  int lunarM = diff + 11;
  if (b11 - a11 > 365) {
    int leapMonthDiff = getLeapMonthOffset(a11, tz);
    if (diff >= leapMonthDiff) { lunarM = diff + 10; if (diff == leapMonthDiff) leapFlag = true; }
  }
  if (lunarM > 12) lunarM -= 12;
  if (lunarM >= 11 && diff < 4) lunarY -= 1;
  lm = lunarM; ly = lunarY; leap = leapFlag;
}

// =========================================================
// VẼ ĐỒNG HỒ (dùng chung cho chế độ toàn màn hình và nửa trái ở chế độ Split)
// =========================================================
void veDongHoTrongVung(int xOff, int yOff, int wVung, int hVung, const String& kieu, uint16_t mau,
                        uint16_t mauNen, bool veNen, bool veNgayDuong, bool veNgayAm) {
  struct tm tinfo;
  if (!getLocalTime(&tinfo)) return;
  if (wVung < 4 || hVung < 4) return;

  if (veNen) {
    for (int y = 0; y < hVung; y++)
      for (int x = 0; x < wVung; x++)
        matrix->drawPixel(xOff + x, yOff + y, mauNen);
  }

  if (kieu == "analog") {
    int cx = xOff + wVung / 2, cy = yOff + hVung / 2;
    int r = min(wVung, hVung) / 2 - 1;
    if (r < 2) return;
    for (int i = 0; i < 12; i++) {
      float goc = i * 30 * PI / 180.0;
      matrix->drawPixel(cx + sin(goc) * r, cy - cos(goc) * r, mau);
    }
    float gioGoc  = ((tinfo.tm_hour % 12) + tinfo.tm_min / 60.0) * 30 * PI / 180.0;
    float phutGoc = (tinfo.tm_min + tinfo.tm_sec / 60.0) * 6 * PI / 180.0;
    float giayGoc = tinfo.tm_sec * 6 * PI / 180.0;
    matrix->drawLine(cx, cy, cx + sin(gioGoc) * r * 0.5, cy - cos(gioGoc) * r * 0.5, mau);
    matrix->drawLine(cx, cy, cx + sin(phutGoc) * r * 0.75, cy - cos(phutGoc) * r * 0.75, mau);
    matrix->drawLine(cx, cy, cx + sin(giayGoc) * r * 0.9, cy - cos(giayGoc) * r * 0.9, matrix->color565(255, 0, 0));
  } else {
    bool chamNhay = (tinfo.tm_sec % 2 == 0);
    char gio[6];
    sprintf(gio, "%02d%c%02d", tinfo.tm_hour, chamNhay ? ':' : ' ', tinfo.tm_min);
    matrix->setTextColor(mau);
    int yTiep = yOff + 2;
    auto tinhTextSizeHopLe = [&](int soKyTu, int maxSize, int topPadding, int sidePadding) {
      for (int s = maxSize; s >= 1; s--) {
        int wCan = soKyTu * 6 * s;
        int hCan = 8 * s;
        if (wCan <= max(1, wVung - sidePadding * 2) && hCan <= max(1, hVung - topPadding - 2)) return s;
      }
      return 1;
    };

    if (kieu == "ring") {
      int cx = xOff + wVung / 2, cy = yOff + hVung / 2;
      int r = min(wVung, hVung) / 2 - 1;
      if (r < 2) return;
      matrix->drawCircle(cx, cy, r, mau);
      int ts = tinhTextSizeHopLe(5, 2, 1, 1);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      int textH = 8 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + max(0, (hVung - textH) / 2));
      matrix->print(gio);
      return;
    }

    if (kieu == "monthsplit" && wVung >= 24 && hVung >= 16) {
      int leftW = max(10, wVung / 2);
      int rightW = wVung - leftW;
      matrix->drawRect(xOff, yOff, wVung, hVung, mau);
      matrix->drawFastVLine(xOff + leftW, yOff + 1, max(1, hVung - 2), mau);

      int ts = (leftW >= 28 && hVung >= 18) ? 2 : 1;
      int textW = 5 * 6 * ts;
      int textH = 8 * ts;
      matrix->setTextSize(ts);
      matrix->setCursor(xOff + max(0, (leftW - textW) / 2), yOff + max(0, (hVung - textH) / 2));
      matrix->print(gio);

      int yy = tinfo.tm_year + 1900;
      int mm = tinfo.tm_mon + 1;
      int dd = tinfo.tm_mday;
      int dim = 31;
      if (mm == 4 || mm == 6 || mm == 9 || mm == 11) dim = 30;
      if (mm == 2) {
        bool leap = ((yy % 4 == 0 && yy % 100 != 0) || (yy % 400 == 0));
        dim = leap ? 29 : 28;
      }
      int dow = (jdFromDate(1, mm, yy) + 1) % 7; // 0 = CN
      int idx = dow + dd - 1;
      int col = idx % 7;
      int row = idx / 7;
      int cellW = max(1, (rightW - 2) / 7);
      int cellH = max(1, (hVung - 4) / 6);
      int bx = xOff + leftW + 1 + col * cellW;
      int by = yOff + 2 + row * cellH;
      matrix->drawRect(bx, by, max(1, cellW), max(1, cellH), mau);

      if (veNgayAm && hVung >= 24) {
        int ad, am, ay; bool nhuan;
        convertSolar2Lunar(dd, mm, yy, 7, ad, am, ay, nhuan);
        char amStr[8];
        sprintf(amStr, "A%02d/%02d", ad, am);
        int maxKyTu = max(1, leftW / 6);
        String amLich = String(amStr);
        if ((int)amLich.length() > maxKyTu) amLich = amLich.substring(0, maxKyTu);
        matrix->setTextSize(1);
        matrix->setCursor(xOff + max(0, (leftW - (int)amLich.length() * 6) / 2), yOff + hVung - 8);
        matrix->print(amLich);
      }
      return;
    }

    if (kieu == "minimal") {
      int ts = tinhTextSizeHopLe(5, 2, 1, 1);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      int textH = 8 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + max(1, (hVung - textH) / 2));
      matrix->print(gio);
      return;
    }

    if (kieu == "neon") {
      matrix->drawRect(xOff, yOff + 2, wVung, max(8, hVung - 4), mau);
      matrix->drawFastHLine(xOff + 1, yOff + 1, max(1, wVung - 2), mau);
      int ts = tinhTextSizeHopLe(5, 2, 4, 1);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + 4);
      matrix->print(gio);
      yTiep = yOff + 4 + 8 * ts + 1;
    } else if (kieu == "metro") {
      matrix->drawFastVLine(xOff + 1, yOff + 2, max(1, hVung - 4), mau);
      matrix->drawFastVLine(xOff + wVung - 2, yOff + 2, max(1, hVung - 4), mau);
      int ts = tinhTextSizeHopLe(5, 2, 3, 2);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + 3);
      matrix->print(gio);
      yTiep = yOff + 3 + 8 * ts + 1;
    } else if (kieu == "boxed") {
      matrix->drawRect(xOff, yOff, wVung, hVung, mau);
      int ts = tinhTextSizeHopLe(5, 2, 3, 1);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + 3);
      matrix->print(gio);
      yTiep = yOff + 3 + 8 * ts + 1;
    } else if (kieu == "compact") {
      char compactLine[9];
      sprintf(compactLine, "%02d:%02d %02d", tinfo.tm_hour, tinfo.tm_min, tinfo.tm_sec);
      int ts = tinhTextSizeHopLe((int)strlen(compactLine), 1, 1, 1);
      matrix->setTextSize(ts);
      int textW = (int)strlen(compactLine) * 6 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + max(0, (hVung - 8 * ts) / 2));
      matrix->print(compactLine);
      yTiep = yOff + hVung;
    } else {
      int ts = tinhTextSizeHopLe(5, 2, 2, 1);
      matrix->setTextSize(ts);
      int textW = 5 * 6 * ts;
      matrix->setCursor(xOff + max(0, (wVung - textW) / 2), yOff + 2);
      matrix->print(gio);
      yTiep = yOff + 2 + 8 * ts + 1;
    }

    bool conCho2Dong = (hVung - (yTiep - yOff) >= 16);
    bool conCho1Dong = (hVung - (yTiep - yOff) >= 8);
    bool uuTienAm = (veNgayAm && (!veNgayDuong || (veNgayDuong && !conCho2Dong && ((tinfo.tm_sec / 2) % 2 == 1))));

    if (veNgayDuong && conCho1Dong && !uuTienAm) {
      char ngay[12];
      if (wVung >= 40) sprintf(ngay, "%02d/%02d/%04d", tinfo.tm_mday, tinfo.tm_mon + 1, tinfo.tm_year + 1900);
      else sprintf(ngay, "%02d/%02d", tinfo.tm_mday, tinfo.tm_mon + 1);
      int maxKyTu = max(1, wVung / 6);
      String ngayStr = String(ngay);
      if ((int)ngayStr.length() > maxKyTu) ngayStr = ngayStr.substring(0, maxKyTu);
      matrix->setTextSize(1);
      matrix->setCursor(xOff + max(0, (wVung - (int)ngayStr.length() * 6) / 2), yTiep);
      matrix->print(ngayStr);
      yTiep += 8;
    }

    if (veNgayAm && (conCho2Dong || (conCho1Dong && uuTienAm) || (hVung - (yTiep - yOff) >= 8))) {
      int ad, am, ay; bool nhuan;
      convertSolar2Lunar(tinfo.tm_mday, tinfo.tm_mon + 1, tinfo.tm_year + 1900, 7, ad, am, ay, nhuan);
      char amStr[12];
      sprintf(amStr, "AL%02d/%02d%s", ad, am, nhuan ? "n" : "");
      int maxKyTu = max(1, wVung / 6);
      String amLich = String(amStr);
      if ((int)amLich.length() > maxKyTu) amLich = amLich.substring(0, maxKyTu);
      matrix->setTextSize(1);
      matrix->setCursor(xOff + max(0, (wVung - (int)amLich.length() * 6) / 2), yTiep);
      matrix->print(amLich);
    }
  }
}

void veDongHo() {
  veDongHoTrongVung(0, 0, W_hienTai, H_hienTai, kieuDongHo, mauDongHo, mauNenDongHo, true, hienNgayDuong, hienNgayAm);
}

void veManHinhKetHop() {
  matrix->fillScreen(matrix->color565(0, 0, 0));
  int rongDongHo = W_hienTai / 2;
  int rongAnh = W_hienTai - rongDongHo;
  veDongHoTrongVung(0, 0, rongDongHo, H_hienTai, kieuDongHoSplit, mauDongHoSplit, 0, false, false, false);
  if (splitAnhBuf != nullptr && splitAnhRong == rongAnh) {
    veBufferLenMatrix(splitAnhBuf, rongAnh, H_hienTai, rongDongHo);
  }
}

void veChuTrongVung(int xOff, int yOff, int wVung, int hVung, const String& noiDung, uint16_t mau,
                    uint16_t mauNen, const String& canLe, const String& effect, int speed) {
  for (int y = 0; y < hVung; y++) {
    for (int x = 0; x < wVung; x++) {
      matrix->drawPixel(xOff + x, yOff + y, mauNen);
    }
  }

  String dong = noiDung;
  dong.replace("\r", " ");
  int newlinePos = dong.indexOf('\n');
  if (newlinePos >= 0) dong = dong.substring(0, newlinePos);

  int textSize = (wVung >= 40 && hVung >= 14) ? 2 : 1;
  int kyTuRong = 6 * textSize;
  int kyTuCao = 8 * textSize;
  int sucChua = max(1, wVung / kyTuRong);
  if (effect == "static" && (int)dong.length() > sucChua) dong = dong.substring(0, sucChua);

  int textWidth = dong.length() * kyTuRong;
  int cursorX = xOff + max(0, (wVung - textWidth) / 2);
  int cursorY = yOff + max(0, (hVung - kyTuCao) / 2);

  int stepDelay = max(25, 210 - gioiHan(speed, 1, 10) * 18);
  if (effect == "scroll-left") {
    if (layoutTextOffsetX == 0) layoutTextOffsetX = wVung;
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay) {
      layoutTextLastStep = millis();
      layoutTextOffsetX--;
      if (layoutTextOffsetX < -textWidth) layoutTextOffsetX = wVung;
    }
    cursorX = xOff + layoutTextOffsetX;
  } else if (effect == "scroll-right") {
    if (layoutTextOffsetX == 0) layoutTextOffsetX = -textWidth;
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay) {
      layoutTextLastStep = millis();
      layoutTextOffsetX++;
      if (layoutTextOffsetX > wVung) layoutTextOffsetX = -textWidth;
    }
    cursorX = xOff + layoutTextOffsetX;
  } else if (effect == "scroll-up") {
    if (layoutTextOffsetY == 0) layoutTextOffsetY = hVung;
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay) {
      layoutTextLastStep = millis();
      layoutTextOffsetY--;
      if (layoutTextOffsetY < -kyTuCao) layoutTextOffsetY = hVung;
    }
    cursorX = xOff + max(0, (wVung - textWidth) / 2);
    cursorY = yOff + layoutTextOffsetY;
  } else if (effect == "bounce") {
    if (layoutTextOffsetX == 0 && layoutTextDirX == -1) layoutTextOffsetX = max(0, wVung - textWidth);
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay) {
      layoutTextLastStep = millis();
      layoutTextOffsetX += layoutTextDirX;
      if (layoutTextOffsetX <= 0) {
        layoutTextOffsetX = 0;
        layoutTextDirX = 1;
      } else if (layoutTextOffsetX >= max(0, wVung - textWidth)) {
        layoutTextOffsetX = max(0, wVung - textWidth);
        layoutTextDirX = -1;
      }
    }
    cursorX = xOff + layoutTextOffsetX;
  } else if (effect == "blink") {
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay * 2) {
      layoutTextLastStep = millis();
      layoutTextBlinkOn = !layoutTextBlinkOn;
    }
    if (!layoutTextBlinkOn) return;
  } else if (effect == "pulse") {
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay * 2) {
      layoutTextLastStep = millis();
      layoutTextBlinkOn = !layoutTextBlinkOn;
    }
    matrix->setTextColor(layoutTextBlinkOn ? mau : matrix->color565(32, 32, 32));
  } else if (effect == "typewriter") {
    if (millis() - layoutTextLastStep >= (uint32_t)stepDelay * 2) {
      layoutTextLastStep = millis();
      layoutTextTypedChars++;
      if (layoutTextTypedChars > (int)dong.length()) layoutTextTypedChars = 1;
    }
    dong = dong.substring(0, max(1, min(layoutTextTypedChars, (int)dong.length())));
    textWidth = dong.length() * kyTuRong;
    cursorX = xOff + max(0, (wVung - textWidth) / 2);
  } else {
    if (canLe == "left") cursorX = xOff;
    else if (canLe == "right") cursorX = xOff + max(0, wVung - textWidth);
  }

  matrix->setTextSize(textSize);
  if (effect != "pulse") matrix->setTextColor(mau);
  matrix->setCursor(cursorX, cursorY);
  matrix->print(dong);
}

void veBoCucTuDo() {
  matrix->fillScreen(matrix->color565(0, 0, 0));

  if (layoutImage.enabled && layoutImage.isGif && layoutGifSanSang &&
      layoutGifSoFrame > 0 && layoutGifFrames[layoutGifFrameHienTai] != nullptr &&
      layoutGifW == layoutImage.w && layoutGifH == layoutImage.h) {
    uint16_t* frame = layoutGifFrames[layoutGifFrameHienTai];
    for (int y = 0; y < layoutGifH; y++) {
      for (int x = 0; x < layoutGifW; x++) {
        int dx = layoutImage.x + x;
        int dy = layoutImage.y + y;
        if (dx >= 0 && dx < W_hienTai && dy >= 0 && dy < H_hienTai) {
          matrix->drawPixel(dx, dy, frame[y * layoutGifW + x]);
        }
      }
    }
  } else if (layoutImage.enabled && !layoutImage.isGif &&
             layoutImageBuf != nullptr && layoutImageW == layoutImage.w && layoutImageH == layoutImage.h) {
    for (int y = 0; y < layoutImageH; y++) {
      for (int x = 0; x < layoutImageW; x++) {
        int dx = layoutImage.x + x;
        int dy = layoutImage.y + y;
        if (dx >= 0 && dx < W_hienTai && dy >= 0 && dy < H_hienTai) {
          matrix->drawPixel(dx, dy, layoutImageBuf[y * layoutImageW + x]);
        }
      }
    }
  }

  if (layoutClock.enabled) {
    veDongHoTrongVung(layoutClock.x, layoutClock.y, layoutClock.w, layoutClock.h, layoutClock.style,
                      layoutClock.color, layoutClock.bgColor, true, layoutClock.showDate, layoutClock.showLunar);
  }

  if (layoutText.enabled && layoutText.content.length() > 0) {
    veChuTrongVung(layoutText.x, layoutText.y, layoutText.w, layoutText.h, layoutText.content,
                   layoutText.color, layoutText.bgColor, layoutText.align, layoutText.effect, layoutText.speed);
  }
}

// =========================================================
// MQTT CALLBACK
// =========================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);

  // ---- Tín hiệu sống: web dang thao tac that ----
  if (topicStr == T_LIVE_PING) {
    lanCuoiNhanTinHieuSong = millis();
    return;
  }

  // ---- Đổi hướng màn hình ----
  if (topicStr == T_ORIENT) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_ORIENT cu/retained"); return; }
    String s = ""; for (unsigned int i = 0; i < length; i++) s += (char)payload[i];
    capNhatHuongManHinh(s.toInt());
    luuCauHinhChung();
    return;
  }

  // ---- Thời gian hiệu ứng chuyển cảnh ----
  if (topicStr == T_TRANS) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_TRANS cu/retained"); return; }
    String s = ""; for (unsigned int i = 0; i < length; i++) s += (char)payload[i];
    thoiGianChuyenCanh = s.toInt();
    luuCauHinhChung();
    return;
  }

  if (topicStr == T_BRIGHTNESS) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_BRIGHTNESS cu/retained"); return; }
    String s = ""; for (unsigned int i = 0; i < length; i++) s += (char)payload[i];
    doSangHienTai = (uint8_t)gioiHan(s.toInt(), 5, 255);
    dma_display->setBrightness8(doSangHienTai);
    luuCauHinhChung();
    return;
  }

  // ---- Đổi chế độ hiển thị ----
  if (topicStr == T_MODE) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_MODE cu/retained"); return; }
    String modeStr = ""; for (unsigned int i = 0; i < length; i++) modeStr += (char)payload[i];
    if (modeStr == "image") cheDoHienTai = CHEDO_ANH;
    else if (modeStr == "gif") { cheDoHienTai = CHEDO_GIF; xoaBufferGif(); }
    else if (modeStr == "clock") cheDoHienTai = CHEDO_DONGHO;
    else if (modeStr == "split") cheDoHienTai = CHEDO_SPLIT;
    else if (modeStr == "layout") cheDoHienTai = CHEDO_LAYOUT;
    Serial.println("Doi che do: " + modeStr);
    luuCauHinhChung();
    return;
  }

  // ---- Ảnh tĩnh ----
  if (topicStr == T_IMAGE) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_IMAGE cu/retained"); return; }
    int tongDiem = W_hienTai * H_hienTai;
    if (length >= (uint32_t)(tongDiem * 2)) {
      for (int i = 0; i < tongDiem; i++) {
        uint16_t hi = payload[i * 2], lo = payload[i * 2 + 1];
        bufferAnh[i] = (hi << 8) | lo;
      }
      cheDoHienTai = CHEDO_ANH;
      chuyenCanh([]() { veBufferLenMatrix(bufferAnh, W_hienTai, H_hienTai); });
      Serial.println("Da nhan anh tinh.");
      luuAnhTinhVaoFlash();
      luuCauHinhChung();
    }
    return;
  }

  // ---- Metadata GIF ----
  if (topicStr == T_GIF_META) {
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, payload, length)) {
      xoaBufferGif();
      int frames = doc["frames"] | 0;
      gifDelayMs = doc["delay"] | 100;
      soFrameGif = min(frames, MAX_GIF_FRAMES);
      for (int i = 0; i < soFrameGif; i++) {
        // dùng calloc để khung bị mất gói hiển thị màu đen thay vì rác trong RAM
        gifFrames[i] = (uint16_t*)calloc(W_hienTai * H_hienTai, sizeof(uint16_t));
      }
      gifVuaBatDau = true;
      Serial.printf("GIF meta: %d frame, delay=%dms\n", soFrameGif, gifDelayMs);
    }
    return;
  }

  // ---- Từng khung GIF (có gửi ACK phản hồi) ----
  if (topicStr == T_GIF_FRAME) {
    int tongDiem = W_hienTai * H_hienTai;
    if (length >= (uint32_t)(2 + tongDiem * 2)) {
      int idx = (payload[0] << 8) | payload[1];
      if (idx >= 0 && idx < soFrameGif && gifFrames[idx] != nullptr) {
        for (int i = 0; i < tongDiem; i++) {
          uint16_t hi = payload[2 + i * 2], lo = payload[2 + i * 2 + 1];
          gifFrames[idx][i] = (hi << 8) | lo;
        }
        gifFrameDaNhan[idx] = true;
        // gửi ACK để web biết đã nhận đúng khung này
        byte ack[2] = { (byte)((idx >> 8) & 0xFF), (byte)(idx & 0xFF) };
        mqttClient.publish(T_GIF_ACK.c_str(), ack, 2);
      }
    }
    return;
  }

  // ---- GIF gửi xong ----
  if (topicStr == T_GIF_DONE) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_GIF_DONE cu/retained"); return; }
    int daNhanDu = 0;
    for (int i = 0; i < soFrameGif; i++) if (gifFrameDaNhan[i]) daNhanDu++;
    gifFrameHienTai = 0;
    gifLastUpdate = millis();
    gifSanSang = (soFrameGif > 0);
    cheDoHienTai = CHEDO_GIF;
    Serial.printf("GIF san sang phat: %d/%d frame day du\n", daNhanDu, soFrameGif);
    if (gifSanSang) luuGifVaoFlash();
    luuCauHinhChung();
    return;
  }

  // ---- Cấu hình đồng hồ ----
  if (topicStr == T_CLOCK_CFG) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_CLOCK_CFG cu/retained"); return; }
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, payload, length)) {
      kieuDongHo = (const char*)(doc["style"] | "digital");
      mauDongHo = hexColorToRGB565(String((const char*)(doc["color"] | "#00FF00")));
      mauNenDongHo = hexColorToRGB565(String((const char*)(doc["bgColor"] | "#000000")));
      hienNgayDuong = doc["showDate"] | true;
      hienNgayAm = doc["showLunar"] | true;
      cheDoHienTai = CHEDO_DONGHO;
      Serial.println("Cap nhat cau hinh dong ho: " + kieuDongHo);
      luuCauHinhChung();
    }
    return;
  }

  if (topicStr == T_LAYOUT_CFG) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_LAYOUT_CFG cu/retained"); return; }
    StaticJsonDocument<768> doc;
    if (!deserializeJson(doc, payload, length)) {
      JsonObject clock = doc["clock"];
      JsonObject text = doc["text"];
      JsonObject image = doc["image"];

      layoutClock.enabled = clock["enabled"] | true;
      layoutClock.x = gioiHan(clock["x"] | 2, 0, W_hienTai - 1);
      layoutClock.y = gioiHan(clock["y"] | 2, 0, H_hienTai - 1);
      layoutClock.w = gioiHan(clock["w"] | 24, 4, W_hienTai - layoutClock.x);
      layoutClock.h = gioiHan(clock["h"] | 18, 4, H_hienTai - layoutClock.y);
      layoutClock.style = String((const char*)(clock["style"] | "boxed"));
      layoutClock.color = hexColorToRGB565(String((const char*)(clock["color"] | "#00FF88")));
      layoutClock.bgColor = hexColorToRGB565(String((const char*)(clock["bgColor"] | "#000000")));
      layoutClock.showDate = clock["showDate"] | true;
      layoutClock.showLunar = clock["showLunar"] | false;

      layoutText.enabled = text["enabled"] | false;
      layoutText.x = gioiHan(text["x"] | 0, 0, W_hienTai - 1);
      layoutText.y = gioiHan(text["y"] | 0, 0, H_hienTai - 1);
      layoutText.w = gioiHan(text["w"] | 12, 4, W_hienTai - layoutText.x);
      layoutText.h = gioiHan(text["h"] | 8, 4, H_hienTai - layoutText.y);
      layoutText.content = String((const char*)(text["content"] | ""));
      layoutText.color = hexColorToRGB565(String((const char*)(text["color"] | "#FFFFFF")));
      layoutText.bgColor = hexColorToRGB565(String((const char*)(text["bgColor"] | "#000000")));
      layoutText.align = String((const char*)(text["align"] | "center"));
      layoutText.effect = String((const char*)(text["effect"] | "static"));
      layoutText.speed = gioiHan(text["speed"] | 4, 1, 10);
      layoutTextOffsetX = layoutText.w;
      layoutTextOffsetY = layoutText.h;
      layoutTextLastStep = millis();
      layoutTextBlinkOn = true;
      layoutTextDirX = -1;
      layoutTextTypedChars = 1;

      layoutImage.enabled = image["enabled"] | false;
      String imgType = String((const char*)(image["type"] | "image"));
      layoutImage.isGif = (imgType == "gif");
      layoutImage.x = gioiHan(image["x"] | 0, 0, W_hienTai - 1);
      layoutImage.y = gioiHan(image["y"] | 0, 0, H_hienTai - 1);
      layoutImage.w = gioiHan(image["w"] | 12, 4, W_hienTai - layoutImage.x);
      layoutImage.h = gioiHan(image["h"] | 8, 4, H_hienTai - layoutImage.y);
      if (!layoutImage.enabled || layoutImage.isGif) xoaLayoutImage();
      if (!layoutImage.enabled || !layoutImage.isGif) xoaLayoutGif();

      cheDoHienTai = CHEDO_LAYOUT;
      chuyenCanh([]() { veBoCucTuDo(); });
      luuCauHinhChung();
    }
    return;
  }

  // ---- Cấu hình chế độ Kết hợp ----
  if (topicStr == T_SPLIT_CFG) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_SPLIT_CFG cu/retained"); return; }
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, payload, length)) {
      kieuDongHoSplit = (const char*)(doc["style"] | "digital");
      mauDongHoSplit = hexColorToRGB565(String((const char*)(doc["color"] | "#00FF00")));
      luuCauHinhChung();
    }
    return;
  }

  // ---- Ảnh cho nửa phải ở chế độ Kết hợp ----
  if (topicStr == T_SPLIT_IMG) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_SPLIT_IMG cu/retained"); return; }
    int rongAnh = W_hienTai - (W_hienTai / 2);
    int tongDiem = rongAnh * H_hienTai;
    if (length >= (uint32_t)(tongDiem * 2)) {
      if (splitAnhBuf != nullptr) free(splitAnhBuf);
      splitAnhBuf = (uint16_t*)malloc(tongDiem * sizeof(uint16_t));
      splitAnhRong = rongAnh;
      for (int i = 0; i < tongDiem; i++) {
        uint16_t hi = payload[i * 2], lo = payload[i * 2 + 1];
        splitAnhBuf[i] = (hi << 8) | lo;
      }
      cheDoHienTai = CHEDO_SPLIT;
      chuyenCanh([]() { veManHinhKetHop(); });
      luuAnhSplitVaoFlash();
      luuCauHinhChung();
    }
    return;
  }

  if (topicStr == T_LAYOUT_IMG) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_LAYOUT_IMG cu/retained"); return; }
    int tongDiem = layoutImage.w * layoutImage.h;
    if (length >= (uint32_t)(tongDiem * 2) && tongDiem > 0) {
      layoutImage.isGif = false;
      xoaLayoutGif();
      xoaLayoutImage();
      layoutImageBuf = (uint16_t*)malloc(tongDiem * sizeof(uint16_t));
      if (layoutImageBuf != nullptr) {
        layoutImageW = layoutImage.w;
        layoutImageH = layoutImage.h;
        for (int i = 0; i < tongDiem; i++) {
          uint16_t hi = payload[i * 2], lo = payload[i * 2 + 1];
          layoutImageBuf[i] = (hi << 8) | lo;
        }
        layoutImage.enabled = true;
        luuLayoutImageVaoFlash();
        luuCauHinhChung();
      }
    }
    return;
  }

  if (topicStr == T_LAYOUT_GIF_META) {
    xoaLayoutGif();
    StaticJsonDocument<256> doc;
    if (!deserializeJson(doc, payload, length)) {
      layoutImage.isGif = true;
      layoutGifSoFrame = min((int)(doc["frames"] | 0), MAX_LAYOUT_GIF_FRAMES);
      layoutGifDelayMs = doc["delay"] | 100;
      layoutGifW = doc["width"] | layoutImage.w;
      layoutGifH = doc["height"] | layoutImage.h;
      int diem = layoutGifW * layoutGifH;
      if (diem > 0) {
        for (int i = 0; i < layoutGifSoFrame; i++) {
          layoutGifFrames[i] = (uint16_t*)calloc(diem, sizeof(uint16_t));
          layoutGifDaNhan[i] = (layoutGifFrames[i] != nullptr);
        }
      }
      layoutGifFrameHienTai = 0;
      layoutGifLastUpdate = millis();
      layoutGifSanSang = false;
    }
    return;
  }

  if (topicStr == T_LAYOUT_GIF_FRAME) {
    int diem = layoutGifW * layoutGifH;
    if (length >= (uint32_t)(2 + diem * 2) && diem > 0) {
      int idx = (payload[0] << 8) | payload[1];
      if (idx >= 0 && idx < layoutGifSoFrame && layoutGifFrames[idx] != nullptr) {
        for (int i = 0; i < diem; i++) {
          uint16_t hi = payload[2 + i * 2], lo = payload[2 + i * 2 + 1];
          layoutGifFrames[idx][i] = (hi << 8) | lo;
        }
        layoutGifDaNhan[idx] = true;
      }
    }
    return;
  }

  if (topicStr == T_LAYOUT_GIF_DONE) {
    if (!laLenhConTuoi()) { Serial.println("Bo qua T_LAYOUT_GIF_DONE cu/retained"); return; }
    int daNhan = 0;
    for (int i = 0; i < layoutGifSoFrame; i++) if (layoutGifDaNhan[i]) daNhan++;
    layoutGifSanSang = (layoutGifSoFrame > 0 && daNhan > 0);
    layoutGifFrameHienTai = 0;
    layoutGifLastUpdate = millis();
    cheDoHienTai = CHEDO_LAYOUT;
    if (layoutGifSanSang) luuLayoutGifVaoFlash();
    luuCauHinhChung();
    return;
  }
}

// =========================================================
// KẾT NỐI WIFI
// =========================================================
void ketNoiWifi() {
  Serial.print("Dang ket noi WiFi: "); Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(400); Serial.print("."); }
  Serial.println("\nWiFi da ket noi, IP: " + WiFi.localIP().toString());
}

// =========================================================
// KẾT NỐI / SUBSCRIBE MQTT
// =========================================================
void ketNoiMqtt() {
  while (!mqttClient.connected()) {
    String clientId = "esp32ledmatrix-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    Serial.println("Dang ket noi MQTT...");
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("MQTT ket noi thanh cong.");
      mqttClient.subscribe(T_MODE.c_str());
      mqttClient.subscribe(T_IMAGE.c_str());
      mqttClient.subscribe(T_GIF_META.c_str());
      mqttClient.subscribe(T_GIF_FRAME.c_str());
      mqttClient.subscribe(T_GIF_DONE.c_str());
      mqttClient.subscribe(T_CLOCK_CFG.c_str());
      mqttClient.subscribe(T_ORIENT.c_str());
      mqttClient.subscribe(T_TRANS.c_str());
      mqttClient.subscribe(T_BRIGHTNESS.c_str());
      mqttClient.subscribe(T_SPLIT_CFG.c_str());
      mqttClient.subscribe(T_SPLIT_IMG.c_str());
      mqttClient.subscribe(T_LAYOUT_CFG.c_str());
      mqttClient.subscribe(T_LAYOUT_IMG.c_str());
      mqttClient.subscribe(T_LAYOUT_GIF_META.c_str());
      mqttClient.subscribe(T_LAYOUT_GIF_FRAME.c_str());
      mqttClient.subscribe(T_LAYOUT_GIF_DONE.c_str());
      mqttClient.subscribe(T_LIVE_PING.c_str());
    } else {
      Serial.print("That bai, rc="); Serial.print(mqttClient.state());
      Serial.println(" thu lai sau 2s");
      delay(2000);
    }
  }
}

// =========================================================
// SETUP
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  HUB75_I2S_CFG::i2s_pins _pins = {
    R1_PIN, G1_PIN, B1_PIN, R2_PIN, G2_PIN, B2_PIN,
    A_PIN, B_PIN, C_PIN, D_PIN, E_PIN,
    LAT_PIN, OE_PIN, CLK_PIN
  };
  HUB75_I2S_CFG mxconfig(PANEL_RES_X * 2, PANEL_RES_Y / 2, PANEL_CHAIN, _pins);

  dma_display = new MatrixPanel_I2S_DMA(mxconfig);
  if (!dma_display->begin()) Serial.println("!!! Cap phat bo nho DMA that bai !!!");
  dma_display->clearScreen();

  matrix = new VirtualMatrixPanel(*dma_display, NUM_ROWS, NUM_COLS, PANEL_RES_X, PANEL_RES_Y);
  matrix->setPhysicalPanelScanRate(FOUR_SCAN_32PX_HIGH);

  // Mount LittleFS và khôi phục cấu hình/dữ liệu đã lưu
  bool fsOk = LittleFS.begin(true);
  Serial.println(fsOk ? "LittleFS: mount OK" : "LittleFS: MOUNT THAT BAI (kiem tra Partition Scheme)");
  if (fsOk) Serial.printf("LittleFS: tong=%u da dung=%u\n", (unsigned)LittleFS.totalBytes(), (unsigned)LittleFS.usedBytes());

  taiCauHinhChung();
  Serial.printf("Khoi phuc tu NVS: mode=%d orient=%d bright=%d soFrameGif=%d\n",
                (int)cheDoHienTai, huongManHinh, doSangHienTai, soFrameGif);

  capNhatHuongManHinh(huongManHinh);
  dma_display->setBrightness8(doSangHienTai);
  matrix->fillScreen(matrix->color565(0, 0, 0));

  switch (cheDoHienTai) {
    case CHEDO_ANH: {
      bool ok = taiAnhTinhTuFlash();
      Serial.printf("Khoi phuc anh tinh: %s\n", ok ? "OK" : "THAT BAI");
      if (ok) veBufferLenMatrix(bufferAnh, W_hienTai, H_hienTai);
      break;
    }
    case CHEDO_GIF: {
      bool ok = taiGifTuFlash();
      Serial.printf("Khoi phuc GIF: %s (%d frame)\n", ok ? "OK" : "THAT BAI", soFrameGif);
      gifLastUpdate = millis();
      gifVuaBatDau = false;
      if (gifSanSang) veBufferLenMatrix(gifFrames[0], W_hienTai, H_hienTai);
      break;
    }
    case CHEDO_SPLIT: {
      bool ok = taiAnhSplitTuFlash();
      Serial.printf("Khoi phuc split: %s\n", ok ? "OK" : "THAT BAI");
      veManHinhKetHop();
      break;
    }
    case CHEDO_LAYOUT: {
      bool ok1 = taiLayoutImageTuFlash();
      bool ok2 = taiLayoutGifTuFlash();
      Serial.printf("Khoi phuc layout image=%s gif=%s\n", ok1 ? "OK" : "-", ok2 ? "OK" : "-");
      veBoCucTuDo();
      break;
    }
    case CHEDO_DONGHO:
    default:
      break; // đồng hồ sẽ tự vẽ trong loop() khi có giờ NTP
  }

  ketNoiWifi();
  configTime(7 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(8200);
  ketNoiMqtt();
}

// =========================================================
// LOOP
// =========================================================
void loop() {
  if (!mqttClient.connected()) ketNoiMqtt();
  mqttClient.loop();

  switch (cheDoHienTai) {
    case CHEDO_ANH:
      break; // đã vẽ ngay khi nhận

    case CHEDO_GIF:
      if (gifSanSang && soFrameGif > 0) {
        if (millis() - gifLastUpdate >= (uint32_t)gifDelayMs) {
          gifLastUpdate = millis();
          if (gifVuaBatDau) {
            gifVuaBatDau = false;
            chuyenCanh([]() { veBufferLenMatrix(gifFrames[gifFrameHienTai], W_hienTai, H_hienTai); });
          } else {
            veBufferLenMatrix(gifFrames[gifFrameHienTai], W_hienTai, H_hienTai);
          }
          gifFrameHienTai = (gifFrameHienTai + 1) % soFrameGif;
        }
      }
      break;

    case CHEDO_DONGHO:
      if (millis() - lastClockDraw >= 1000) { lastClockDraw = millis(); veDongHo(); }
      break;

    case CHEDO_SPLIT:
      if (millis() - lastClockDraw >= 1000) { lastClockDraw = millis(); veManHinhKetHop(); }
      break;

    case CHEDO_LAYOUT:
      if (layoutImage.enabled && layoutImage.isGif && layoutGifSanSang && layoutGifSoFrame > 0) {
        if (millis() - layoutGifLastUpdate >= (uint32_t)layoutGifDelayMs) {
          layoutGifLastUpdate = millis();
          layoutGifFrameHienTai = (layoutGifFrameHienTai + 1) % layoutGifSoFrame;
        }
      }
      if (millis() - lastClockDraw >= 60) { lastClockDraw = millis(); veBoCucTuDo(); }
      break;
  }
}