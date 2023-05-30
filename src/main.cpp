#include <Arduino.h>
#include "m3508.h"
#include "mdds30.h"
#include "qei.hpp"
#include "esp_intr_alloc.h"
#include "udp.h"

// #define LED_PIN 23 
#define rx 4
#define tx 5
#define CAN_SPEED 1000E3

// AMT102
#define TOP_A 25 // Blue
#define TOP_B 33 // Yellow
#define BOTTOM_A 2 // Green
#define BOTTOM_B 15 // White

#define LIMIT_PIN 26
#define LASER_PIN 32

// My MacAddress: b8:d6:1a:bc:ee:68
// e0:5a:1b:75:13:24
uint8_t monitor_mac_addr[] = {0xe0, 0x5a, 0x1b, 0x75, 0x13, 0x24};

TaskHandle_t thp[2];

int16_t count_T;
int16_t count_B;

int8_t headerByte = 0x55; // -128 ~ 127, 0x12 = 18
int8_t addressByte = 0x00; // -128 ~ 127, 0x01 = 1
int8_t commandByte1 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte2 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte3 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t commandByte4 = 0x00; // -128 ~ 127, -1 or 0 or 1
int8_t checksum = 0x00; // -128 ~ 127

int16_t mangle[4] = {0, 0, 0, 0};
int16_t mrpm[4] = {0, 0, 0, 0};
int16_t mtorque[4] = {0, 0, 0, 0};
int16_t current_data[4] = {0, 0, 0, 0};
uint8_t send_data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

int rollr_rdsc[2] = {0, 0};
float updwn_rdsc = 0;
float updwn_angle = 0;
int distance = 0;
int trgt_angle = 0;
bool auto_updwn_flag = false;

int8_t updwn_cmd = 0;
int8_t rollr_cmd = 0;
int8_t rollr_spd_cmd = 0;
int8_t auto_updwn_cmd = 0;

int8_t rollr_val[2] = {0, 0};

int updwn_trgt = 500; // 昇降のターゲットRPM

int16_t rollr_trgt = 0; // ローラのターゲットRPM

// PID: 昇降
int16_t error = 0;
int16_t integral = 0;
int16_t derivative = 0;
int16_t prev_error = 0;

// PID: ローラ
int16_t rollr_error[2] = {0, 0};
int16_t rollr_integral[2] = {0, 0};
int16_t rollr_derivative[2] = {0, 0};
int16_t rollr_prev_error[2] = {0, 0};

float prev_time = 0;
float dt = 0;

float Kp = 15;
float Ki = 0.01;
float Kd = 0.01;

float rollr_Kp = 0.2;
float rollr_Ki = 0.005; // 0.005~
float rollr_Kd = 0.00;

void Core0a(void *args); // PS4 Controller
void Core0b(void *args);
void Core1b(void *args);

uint8_t data[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  pinMode(LASER_PIN, INPUT);
  // AMT102
  qei_setup_x1(PCNT_UNIT_2, TOP_A, TOP_B); // TOP: Blue, Yellow
  qei_setup_x1(PCNT_UNIT_3, BOTTOM_A, BOTTOM_B); // BOTTOM: Green, White
  // digitalWrite(LED_PIN, HIGH);
  // delay(3000);
  // digitalWrite(LED_PIN, LOW);
  Serial2.write(0x80);
  for (int i = 0; i < 3000; i++) {
    Serial2.write(0x80);
    delay(1);
  }
  while (!can_init(rx, tx, CAN_SPEED));

  while (!udp_init());
  Serial.println("> UDP: Started.");
  Serial.println("> UDP: My MacAddress: " + WiFi.macAddress());

  // UDP(ESPNOW): 接続したいESP32のMACアドレスを登録
  if (registerPeerInfo(monitor_mac_addr) == ESP_OK) {
    Serial.println("> UDP: Peer registered.");
  } else {
    Serial.println("> UDP: Peer not registered.");
  }

  xTaskCreatePinnedToCore(Core0a, "Core0a", 4096, NULL, 3, &thp[0], 0); // (タスク名, タスクのサイズ, パラメータ, 優先度, タスクハンドル, コア番号)
  xTaskCreatePinnedToCore(Core0b, "Core0b", 4096, NULL, 3, &thp[1], 0);
  xTaskCreatePinnedToCore(Core1b, "Core1b", 4096, NULL, 3, &thp[2], 1);
}

void loop() {
  m3508_read_data(0x201, mangle, mrpm, mtorque);
  dt = millis() - prev_time;
  prev_time = millis();

  // 自動昇降機構の制御
  if (auto_updwn_cmd == 1) {
    trgt_angle = 31.5;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 2) {
    trgt_angle = 41.0;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 3) {
    trgt_angle = 44.0;
    auto_updwn_flag = !auto_updwn_flag;
  } else if (auto_updwn_cmd == 4) {
    trgt_angle = 0;
    auto_updwn_flag = !auto_updwn_flag;
  }

  if (auto_updwn_flag == true) {
    float error_angle = trgt_angle - updwn_angle;
    // 差が1deg以下の時は制御しない
    if (error_angle > 0 && abs(error_angle) < 0.1) {
      updwn_cmd = 0;
      auto_updwn_flag = false;
    } else if (error_angle < 0 && abs(error_angle) < 2.5) {
      updwn_cmd = 0;
      auto_updwn_flag = false;
    } else {
      updwn_cmd = error_angle > 0 ? 1 : -1;
    }
  }

  if (updwn_cmd == -1 && updwn_angle < 1.7){
    updwn_trgt = 150;
  } else {
    updwn_trgt = 500;
  }

  if (digitalRead(LIMIT_PIN) == !HIGH && updwn_cmd == -1) {
    updwn_cmd = 0;
  }

  if (updwn_angle > 51 && updwn_cmd == 1){
    updwn_cmd = 0;
  }

  if (rollr_cmd == 1){
    rollr_trgt = 147 + rollr_spd_cmd * 5; // 32-33deg
  } else if (rollr_cmd == 2){
    rollr_trgt = 200 + rollr_spd_cmd * 5; // 41deg
  } else if (rollr_cmd == 3){
    rollr_trgt = 200 + rollr_spd_cmd * 5; // 44deg
  } else {
    rollr_trgt = 0;
  }

  rollr_trgt = (rollr_trgt > 300) ? 300 : rollr_trgt;

  error = (updwn_cmd * updwn_trgt) - mrpm[0];
  integral += error * dt;
  derivative = (error - prev_error) / dt;
  prev_error = error;

  current_data[0] = Kp * error + Ki * integral + Kd * derivative;
  current_data[0] = constrain(current_data[0], -2000, 2000);

  for (int i = 0; i < 2; i++) {
    rollr_error[i] = rollr_trgt - rollr_rdsc[i];
    rollr_integral[i] += rollr_error[i] * dt;
    rollr_derivative[i] = (rollr_error[i] - rollr_prev_error[i]) / dt;
    rollr_prev_error[i] = rollr_error[i];
    rollr_val[i] = rollr_Kp * rollr_error[i] + rollr_Ki * rollr_integral[i] + rollr_Kd * rollr_derivative[i];
  }

  rollr_val[0] = constrain(rollr_val[0], 0, 80);
  rollr_val[1] = constrain(rollr_val[1], 0, 80);

  m3508_make_data(current_data, send_data);
  m3508_send_data(send_data);
  mdds30_control_motor(0x00, rollr_val[0], rollr_val[1]);
  delay(1);
}

// エンコーダ値読み取り
void Core0a(void *args) {
  while (1) {
    // LASER_PINをアナログリード
    distance = analogRead(LASER_PIN);
    pcnt_get_counter_value(PCNT_UNIT_2, &count_T);
    pcnt_get_counter_value(PCNT_UNIT_3, &count_B);
    rollr_rdsc[0] = -((count_T / 5.0) / 512.0) * 1000 * (2 * PI); // 実際のrad/s(上のモーター) 0001
    rollr_rdsc[1] = -((count_B / 5.0) / 512.0) * 1000 * (2 * PI); // 実際のrad/s(下のモーター) 0001
    updwn_rdsc = mrpm[0] * 0.10472; // 実際のrad/s(昇降)
    if (digitalRead(LIMIT_PIN) == !HIGH) {
      updwn_angle = 0;
    } else {
      updwn_angle += ((updwn_rdsc * 0.0500000) * 57.2957795) * 0.0005224; // rad to deg 
    }
    pcnt_counter_clear(PCNT_UNIT_2); 
    pcnt_counter_clear(PCNT_UNIT_3);
    delay(5);
  }
}

void Core0b(void *args){
  while (1) {
    if (Serial2.available() > 0 && Serial2.read() == headerByte){
      addressByte = Serial2.read(); // アドレス 
      commandByte1 = Serial2.read(); // 昇降
      commandByte2 = Serial2.read(); // ローラ速度
      commandByte3 = Serial2.read(); // ローラ回転
      commandByte4 = Serial2.read(); // ローラ回転
      checksum = Serial2.read(); // 整合性チェック
      if (checksum == int8_t(headerByte + addressByte + commandByte1 + commandByte2 + commandByte3 + commandByte4)){
        updwn_cmd = commandByte1;
        rollr_cmd = commandByte2;
        rollr_spd_cmd = commandByte3;
        auto_updwn_cmd = commandByte4;
      } 
      Serial2.flush();
      // while (Serial2.available() > 0) {
      //   Serial2.read();
      // }
   }
   delay(10);
  }
}

void Core1b(void *args){
  while (1) {
    // Serial.print(String(updwn_cmd) + ", " + String(rollr_cmd));
    // Serial.print(" | ");
    // Serial.print(String(mrpm[0]) + ", " + String(mtorque[0]));
    // Serial.print(" | ");
    // Serial.print(String(current_data[0]));
    // Serial.print(" | ");
    Serial.print(String(rollr_rdsc[0]) + ", " + String(rollr_rdsc[1]));
    Serial.print(" | ");
    Serial.print(String(updwn_rdsc) + ", " + String(updwn_angle));
    // Serial.print(" | ");
    // Serial.print(String(updwn_trgt));
    Serial.print(" | ");
    // Serial.println(String(distance));
    Serial.print(dt);
    Serial.println();

    // rollr_rdsc[0] を data[0], data[1]
    data[0] = (uint8_t)(rollr_rdsc[0] >> 8) & 0xff; // 上位8bit
    data[1] = (uint8_t)rollr_rdsc[0] & 0xff; // 下位8bit
    // rollr_rdsc[1] を data[2], data[3]
    data[2] = (uint8_t)(rollr_rdsc[1] >> 8) & 0xff; // 上位8bit
    data[3] = (uint8_t)rollr_rdsc[1] & 0xff; // 下位8bit
    // updwn_angle 
    data[4] = (uint8_t)updwn_angle;
    data[5] = (uint8_t)(distance >> 8) & 0xff; // 上位8bit
    data[6] = (uint8_t)distance & 0xff; // 下位8bit
    // updwn_rdsc
    data[8] = (uint8_t)updwn_rdsc & 0xff; // 下位8bit
    data[12] = (uint8_t)dt;
    esp_now_send(monitor_mac_addr, data, sizeof(data)); // データを送信
    delay(50);
  }
}

/* 
ToDo;
・ローラをPIDさせる
*/