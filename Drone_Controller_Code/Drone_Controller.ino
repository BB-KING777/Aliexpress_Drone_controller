#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WiFiUdp.h> // UDP通信

// --- ディスプレイ設定 ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define I2C_SDA 25
#define I2C_SCL 26
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- ピン定義 (タクトスイッチ) ---
#define PIN_BTN_RIGHT 23 // SW_1 (未使用)
#define PIN_BTN_UP    22 // SW_2 (離陸)
#define PIN_BTN_LEFT  19 // SW_3 (Headless)
#define PIN_BTN_DOWN  27 // SW_4 (着陸)
#define PIN_BTN_OK    13 // SW_5 (キャリブレーション)

// --- ピン定義 (ジョイスティック) ---
#define PIN_LEFT_UD   35 // スロットル (Throttle)
#define PIN_LEFT_LR   34 // ヨー (Yaw)
#define PIN_RIGHT_UD  33 // ピッチ (Pitch)
#define PIN_RIGHT_LR  32 // ロール (Roll)

// --- WiFiスキャン関連 ---
int wifiNetworkCount = 0;
String wifiSSIDs[20]; 
int8_t wifiEncType[20];
#define MAX_NETWORKS_TO_SCAN 20
int selectedNetworkIndex = 0; 
int scrollOffset = 0; 
#define MAX_LINES_ON_SCREEN 6 

// --- 画面の状態 ---
enum ScreenState {
  STATE_SCANNING, STATE_SHOW_LIST, STATE_CONNECTING,
  STATE_POPUP_ENCRYPTED, STATE_CONNECTED, STATE_CONTROLLING
};
ScreenState currentState = STATE_SCANNING;

// --- ボタン入力制御 ---
unsigned long lastButtonPressTime = 0;
#define DEBOUNCE_DELAY 200 
unsigned long popupStartTime = 0;
#define POPUP_DURATION 3000 

// --- ドローン制御関連 ---
WiFiUDP udp;
IPAddress droneIP(192, 168, 169, 1); 
#define DRONE_UDP_PORT 8800
unsigned long lastPacketTime = 0;
#define CONTROL_INTERVAL 13 

// クラッシュバグ修正: 画面更新用のタイマー
unsigned long lastScreenUpdateTime = 0;
#define SCREEN_UPDATE_INTERVAL 100 

// --- パケットカウンター ---
uint16_t _ctr1 = 0x0000;
uint16_t _ctr2 = 0x0001;
uint16_t _ctr3 = 0x0002;

// --- 123バイトパケットの静的データ ---
const uint8_t _HEADER[]           = {0xef, 0x02, 0x7c, 0x00, 0x02, 0x02, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00};
const uint8_t _COUNTER1_SUFFIX[]  = {0x00, 0x00, 0x14, 0x00, 0x66, 0x14};
const uint8_t _CONTROL_SUFFIX[]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // 10 bytes
const uint8_t _CHECKSUM_SUFFIX[]  = {
  0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x32, 0x4b, 0x14, 0x2d, 0x00, 0x00
}; // 51 bytes
const uint8_t _COUNTER2_SUFFIX[]  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00,
  0xff, 0xff, 0xff, 0xff
}; // 18 bytes
const uint8_t _COUNTER3_SUFFIX[]  = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00
}; // 14 bytes

// --- ピーキー設定 (ご要望) ---

// --- ジョイスティックのEMAフィルタ ---
#define JOYSTICK_EMA_ALPHA 1.0 // 1.0 = ピーキー (フィルタ無効)
float avg_L_UD; float avg_L_LR;
float avg_R_UD; float avg_R_LR;

// --- コントロールの平滑化フィルタ ---
#define CONTROL_SMOOTH_ALPHA 1.0 // 1.0 = ピーキー (フィルタ無効)
float smooth_roll = 128.0; float smooth_pitch = 128.0;
float smooth_throttle = 128.0; float smooth_yaw = 128.0;

// --- ジョイスティック制御パラメータ ---
#define STICK_DEADZONE 100  
#define STICK_EXPO 0.0      // 0.0 = ピーキー (リニア)

// Y軸 (スロットル/ピッチ) のキャリブレーション
#define STICK_Y_CENTER 1963
#define STICK_Y_MIN 30
#define STICK_Y_MAX 4095

// X軸 (ヨー/ロール) のキャリブレーション
#define STICK_X_CENTER 1790
#define STICK_X_MIN 50
#define STICK_X_MAX 3770

// ドローンパケットの制御範囲
#define STICK_MIN 40    
#define STICK_NEUTRAL 128
#define STICK_MAX 220   

// --- ★★★ バグ修正: float対応の fmap() 関数を追加 ★★★ ---
float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// --- setup() ---
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed")); for(;;);
  }
  delay(500); 

  // スイッチピン設定
  pinMode(PIN_BTN_RIGHT, INPUT); pinMode(PIN_BTN_UP, INPUT);
  pinMode(PIN_BTN_LEFT, INPUT);  pinMode(PIN_BTN_DOWN, INPUT);
  pinMode(PIN_BTN_OK, INPUT);

  // ジョイスティックEMA初期化
  avg_L_UD = analogRead(PIN_LEFT_UD); avg_L_LR = analogRead(PIN_LEFT_LR);
  avg_R_UD = analogRead(PIN_RIGHT_UD); avg_R_LR = analogRead(PIN_RIGHT_LR);

  // WiFi初期化
  WiFi.STA.begin(); 
  Serial.println("WiFi STA Interface Enabled.");
  startWifiScan();
}

// --- loop() ---
// (クラッシュバグ修正版)
void loop() {
  switch (currentState) {
    case STATE_SHOW_LIST:
      handleButtonInput(); 
      break;
    case STATE_POPUP_ENCRYPTED:
      if (millis() - popupStartTime > POPUP_DURATION) {
        currentState = STATE_SHOW_LIST;
      }
      break;
    case STATE_CONNECTED:
      Serial.println("Starting UDP...");
      udp.begin(DRONE_UDP_PORT); 
      Serial.println("Transitioning to Control Mode");
      currentState = STATE_CONTROLLING;
      lastPacketTime = millis();
      lastScreenUpdateTime = millis(); 
      break;
      
    case STATE_CONTROLLING:
      // --- 制御パケットの送信 (高速: 13ms) ---
      if (millis() - lastPacketTime > CONTROL_INTERVAL) {
        lastPacketTime = millis();
        readJoystickInputs();
        sendControlPacket_123byte(); 
      }

      // --- 画面の更新 (低速: 100ms) ---
      if (millis() - lastScreenUpdateTime > SCREEN_UPDATE_INTERVAL) {
        lastScreenUpdateTime = millis();
        drawScreen(); 
      }
      break;
    
    case STATE_SCANNING:
    case STATE_CONNECTING:
      break; // 何もせず待機
  }

  // 制御モード以外では画面を描画
  if (currentState != STATE_CONTROLLING) {
     drawScreen();
     delay(100); 
  }
}

// --- ドローン制御パケット送信 (123バイト版) ---
void sendControlPacket_123byte() {
  
  // 1. スティック値を -1.0 ~ 1.0 に変換
  float throttle_f = mapJoystick(avg_L_UD, STICK_Y_CENTER, STICK_Y_MIN, STICK_Y_MAX); // Y軸
  float yaw_f      = mapJoystick(avg_L_LR, STICK_X_CENTER, STICK_X_MIN, STICK_X_MAX); // X軸
  float pitch_f    = mapJoystick(avg_R_UD, STICK_Y_CENTER, STICK_Y_MIN, STICK_Y_MAX); // Y軸
  float roll_f     = mapJoystick(avg_R_LR, STICK_X_CENTER, STICK_X_MIN, STICK_X_MAX); // X軸

  // 2. 制御値 (40-220) にマッピングし、平滑化
  // ★★★ バグ修正: map() を fmap() に置き換え ★★★
  smooth_roll     = (fmap(roll_f,     -1.0, 1.0, STICK_MIN, STICK_MAX) * CONTROL_SMOOTH_ALPHA) + (smooth_roll * (1.0 - CONTROL_SMOOTH_ALPHA));
  smooth_pitch    = (fmap(pitch_f,    -1.0, 1.0, STICK_MIN, STICK_MAX) * CONTROL_SMOOTH_ALPHA) + (smooth_pitch * (1.0 - CONTROL_SMOOTH_ALPHA));
  smooth_throttle = (fmap(throttle_f, -1.0, 1.0, STICK_MIN, STICK_MAX) * CONTROL_SMOOTH_ALPHA) + (smooth_throttle * (1.0 - CONTROL_SMOOTH_ALPHA)); // スロットル反転解除済み
  smooth_yaw      = (fmap(yaw_f,      -1.0, 1.0, STICK_MIN, STICK_MAX) * CONTROL_SMOOTH_ALPHA) + (smooth_yaw * (1.0 - CONTROL_SMOOTH_ALPHA));

  // 3. コマンドボタンの読み取り
  uint8_t command_byte = 0x00;
  if (digitalRead(PIN_BTN_UP) == LOW)      command_byte = 0x01; // 離陸
  else if (digitalRead(PIN_BTN_DOWN) == LOW) command_byte = 0x02; // 着陸/停止
  else if (digitalRead(PIN_BTN_OK) == LOW)   command_byte = 0x04; // キャリブレーション

  uint8_t headless_byte = (digitalRead(PIN_BTN_LEFT) == LOW) ? 0x03 : 0x02;
  
  // 4. パケットの構築
  // (クラッシュバグ修正: 124バイト確保)
  uint8_t packet[124]; 
  int pos = 0; 

  // ブロック1: ヘッダー (12 bytes)
  memcpy(packet + pos, _HEADER, sizeof(_HEADER));
  pos += sizeof(_HEADER);

  // ブロック2: カウンター1 (8 bytes total)
  packet[pos++] = _ctr1 & 0xFF;
  packet[pos++] = (_ctr1 >> 8) & 0xFF;
  memcpy(packet + pos, _COUNTER1_SUFFIX, sizeof(_COUNTER1_SUFFIX));
  pos += sizeof(_COUNTER1_SUFFIX);

  // ブロック3: 制御データ (16 bytes total)
  uint8_t controls[6] = {
    (uint8_t)smooth_roll,
    (uint8_t)smooth_pitch,
    (uint8_t)smooth_throttle,
    (uint8_t)smooth_yaw,
    command_byte,
    headless_byte
  };
  memcpy(packet + pos, controls, sizeof(controls));
  pos += sizeof(controls);
  memcpy(packet + pos, _CONTROL_SUFFIX, sizeof(_CONTROL_SUFFIX));
  pos += sizeof(_CONTROL_SUFFIX);

  // ブロック4: チェックサム (51 bytes total)
  uint8_t checksum = 0;
  for (int i = 0; i < 6; i++) {
    checksum ^= controls[i];
  }
  packet[pos++] = checksum;
  memcpy(packet + pos, _CHECKSUM_SUFFIX, sizeof(_CHECKSUM_SUFFIX));
  pos += sizeof(_CHECKSUM_SUFFIX);

  // ブロック5: カウンター2 (20 bytes total)
  packet[pos++] = _ctr2 & 0xFF;
  packet[pos++] = (_ctr2 >> 8) & 0xFF;
  memcpy(packet + pos, _COUNTER2_SUFFIX, sizeof(_COUNTER2_SUFFIX));
  pos += sizeof(_COUNTER2_SUFFIX);

  // ブロック6: カウンター3 (16 bytes total)
  packet[pos++] = _ctr3 & 0xFF;
  packet[pos++] = (_ctr3 >> 8) & 0xFF;
  memcpy(packet + pos, _COUNTER3_SUFFIX, sizeof(_COUNTER3_SUFFIX));
  pos += sizeof(_COUNTER3_SUFFIX);
  
  // 5. カウンターを進める
  _ctr1 = (_ctr1 + 1) & 0xFFFF;
  _ctr2 = (_ctr2 + 1) & 0xFFFF;
  _ctr3 = (_ctr3 + 1) & 0xFFFF;
  
  // 6. UDPパケット送信
  udp.beginPacket(droneIP, DRONE_UDP_PORT);
  // (クラッシュバグ修正: 124バイト送信)
  udp.write(packet, 124); 
  udp.endPacket();

  // 7. ディスプレイ描画 (削除済み)
}


// --- ジョイスティックの値読み取り ---
void readJoystickInputs() {
  // (ピーキー設定: ALPHA = 1.0)
  avg_L_UD = (analogRead(PIN_LEFT_UD) * JOYSTICK_EMA_ALPHA) + (avg_L_UD * (1.0 - JOYSTICK_EMA_ALPHA));
  avg_L_LR = (analogRead(PIN_LEFT_LR) * JOYSTICK_EMA_ALPHA) + (avg_L_LR * (1.0 - JOYSTICK_EMA_ALPHA));
  avg_R_UD = (analogRead(PIN_RIGHT_UD) * JOYSTICK_EMA_ALPHA) + (avg_R_UD * (1.0 - JOYSTICK_EMA_ALPHA));
  avg_R_LR = (analogRead(PIN_RIGHT_LR) * JOYSTICK_EMA_ALPHA) + (avg_R_LR * (1.0 - JOYSTICK_EMA_ALPHA));
}

// --- ジョイスティックの値を -1.0 ~ 1.0 に変換 ---
// (非対称対応 ＆ ピーキー設定版)
float mapJoystick(int adcValue, int center, int minVal, int maxVal) {
  // 1. デッドゾーンの適用
  if (abs(adcValue - center) < STICK_DEADZONE) {
    return 0.0;
  }

  // 2. -1.0 ~ 1.0 に正規化
  float normalized = 0.0;
  if (adcValue > center) {
    float range = (float)(maxVal - (center + STICK_DEADZONE));
    if (range <= 0) return 1.0; 
    normalized = (float)(adcValue - (center + STICK_DEADZONE)) / range;
  } else {
    float range = (float)((center - STICK_DEADZONE) - minVal);
    if (range <= 0) return -1.0;
    normalized = (float)(adcValue - (center - STICK_DEADZONE)) / range;
  }
  normalized = constrain(normalized, -1.0, 1.0);

  // 3. Expoカーブの適用 (ピーキー設定: EXPO = 0.0)
  float expoValue = (STICK_EXPO * pow(normalized, 3)) + ((1.0 - STICK_EXPO) * normalized);
  return expoValue;
}


// --- 画面描画（メイン） ---
void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  switch (currentState) {
    case STATE_SCANNING:      drawScanningScreen(); break;
    case STATE_SHOW_LIST:     drawWifiListScreen(); break;
    case STATE_CONNECTING:    drawConnectingScreen(wifiSSIDs[selectedNetworkIndex]); break;
    case STATE_CONNECTED:     drawConnectedScreen(wifiSSIDs[selectedNetworkIndex], WiFi.localIP().toString()); break;
    case STATE_POPUP_ENCRYPTED: drawEncryptedPopupScreen(); break;
    case STATE_CONTROLLING:   drawControlScreen(); break;
  }
  display.display();
}

// --- 制御中のディスプレイ表示 ---
void drawControlScreen() {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE); 
  display.setCursor(0, 0);
  display.print("Drone: ");
  display.println(wifiSSIDs[selectedNetworkIndex].substring(0, 12)); 
  display.setCursor(0, 10);
  display.printf("R:%03d P:%03d T:%03d Y:%03d",
                 (int)smooth_roll, (int)smooth_pitch,
                 (int)smooth_throttle, (int)smooth_yaw);
  display.setCursor(0, 28);
  display.println("--- Left ADC ---");
  display.printf("LR(Yw): %04d UD(Th): %04d", (int)avg_L_LR, (int)avg_L_UD);
  display.setCursor(0, 46);
  display.println("--- Right ADC ---");
  display.printf("LR(Rl): %04d UD(Pi): %04d", (int)avg_R_LR, (int)avg_R_UD);
}

// --- WiFiスキャン処理 (変更なし) ---
void startWifiScan() {
  currentState = STATE_SCANNING; drawScreen(); 
  Serial.println("WiFi Scan Start");
  wifiNetworkCount = WiFi.scanNetworks();
  Serial.printf("%d networks found.\n", wifiNetworkCount);
  if (wifiNetworkCount > 0) {
    for (int i = 0; i < wifiNetworkCount && i < MAX_NETWORKS_TO_SCAN; ++i) {
      wifiSSIDs[i] = WiFi.SSID(i);
      wifiEncType[i] = WiFi.encryptionType(i);
    }
    if (wifiNetworkCount > MAX_NETWORKS_TO_SCAN) {
      wifiNetworkCount = MAX_NETWORKS_TO_SCAN;
    }
  }
  WiFi.scanDelete(); Serial.println("WiFi Scan Deleted.");
  selectedNetworkIndex = 0; scrollOffset = 0;
  currentState = STATE_SHOW_LIST;
}

// --- ボタン入力処理 (変更なし) ---
void handleButtonInput() {
  if (millis() - lastButtonPressTime < DEBOUNCE_DELAY) return;
  if (digitalRead(PIN_BTN_UP) == LOW) {
    lastButtonPressTime = millis();
    if (selectedNetworkIndex > 0) {
      selectedNetworkIndex--;
      if (selectedNetworkIndex < scrollOffset) scrollOffset = selectedNetworkIndex;
    }
  } 
  else if (digitalRead(PIN_BTN_DOWN) == LOW) {
    lastButtonPressTime = millis();
    if (selectedNetworkIndex < wifiNetworkCount - 1) {
      selectedNetworkIndex++;
      if (selectedNetworkIndex >= scrollOffset + MAX_LINES_ON_SCREEN) {
        scrollOffset = selectedNetworkIndex - MAX_LINES_ON_SCREEN + 1;
      }
    }
  } 
  else if (digitalRead(PIN_BTN_OK) == LOW) {
    lastButtonPressTime = millis();
    if (wifiNetworkCount == 0) startWifiScan(); // No Wifiなら再スキャン
    else handleNetworkSelection();
  }
}

// --- 決定ボタンが押された時の処理 (変更なし) ---
void handleNetworkSelection() {
  if (wifiNetworkCount == 0) return;
  String selectedSSID = wifiSSIDs[selectedNetworkIndex];
  int selectedEnc = wifiEncType[selectedNetworkIndex];
  Serial.printf("Selected: %s (Enc: %d)\n", selectedSSID.c_str(), selectedEnc);
  if (selectedEnc == WIFI_AUTH_OPEN) connectToWifi(selectedSSID);
  else showEncryptedPopup();
}

// --- WiFi接続処理 (変更なし) ---
void connectToWifi(String ssid) {
  currentState = STATE_CONNECTING; drawScreen(); 
  WiFi.begin(ssid.c_str());
  unsigned long connectStartTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
    if (millis() - connectStartTime > 15000) {
      Serial.println("Connection Failed (Timeout)");
      WiFi.disconnect();
      currentState = STATE_SHOW_LIST; 
      return;
    }
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: "); Serial.println(WiFi.localIP());
  currentState = STATE_CONNECTED; drawScreen(); 
}

// --- ポップアップ表示 (変更なし) ---
void showEncryptedPopup() {
  currentState = STATE_POPUP_ENCRYPTED;
  popupStartTime = millis();
  drawScreen(); 
}

// --- 各画面の描画ヘルパー (変更なし) ---
void drawScanningScreen() {
  display.setTextSize(2);
  display.setCursor(10, (SCREEN_HEIGHT - 16) / 2); 
  display.println("Scanning...");
}
void drawWifiListScreen() {
  display.setTextSize(1); display.setCursor(0, 0);
  if (wifiNetworkCount == 0) {
    display.println("No WiFi found.");
    display.println();
    display.println("Press OK to rescan.");
    return;
  }
  int lineY = 0;
  for (int i = scrollOffset; i < (scrollOffset + MAX_LINES_ON_SCREEN) && i < wifiNetworkCount; i++) {
    display.setCursor(8, lineY); 
    String prefix = (wifiEncType[i] == WIFI_AUTH_OPEN) ? "(O) " : "(W) ";
    String line = prefix + wifiSSIDs[i];
    if (line.length() > 20) line = line.substring(0, 19) + ".";
    if (i == selectedNetworkIndex) {
      display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
      display.println(line);
      display.setTextColor(SSD1306_WHITE);
    } else {
      display.println(line);
    }
    lineY += 10; 
  }
  display.drawFastVLine(SCREEN_WIDTH - 2, 0, SCREEN_HEIGHT, SSD1306_WHITE);
  float barHeight = (float)SCREEN_HEIGHT / (float)wifiNetworkCount;
  float barY = (float)selectedNetworkIndex * barHeight;
  display.fillRect(SCREEN_WIDTH - 3, (int)barY, 3, (int)barHeight + 1, SSD1306_WHITE);
}
void drawConnectingScreen(String ssid) {
  display.setTextSize(1); display.setCursor(0, 10);
  display.println("Connecting to:");
  display.setTextSize(2); display.setCursor(0, 30);
  if (ssid.length() > 10) {
    display.setTextSize(1); display.setCursor(0, 35);
  }
  display.println(ssid);
}
void drawConnectedScreen(String ssid, String ip) {
  display.setTextSize(1); display.setCursor(0, 0);
  display.println("Connected!");
  display.println(ssid);
  display.println("---------------");
  display.println("IP Address:");
  display.println(ip);
}
void drawEncryptedPopupScreen() {
  display.fillRect(4, 4, SCREEN_WIDTH - 8, SCREEN_HEIGHT - 8, SSD1306_BLACK);
  display.drawRect(4, 4, SCREEN_WIDTH - 8, SCREEN_HEIGHT - 8, SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.println("Password Required");
  display.setCursor(10, 25);
  display.println("This network is");
  display.setCursor(10, 35);
  display.println("encrypted.");
  display.setCursor(10, 50);
  display.println("(Not supported.)");
}