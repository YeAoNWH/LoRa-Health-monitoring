#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <TFT_eSPI.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <XSpaceBioV10.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <DHT.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <MQUnifiedsensor.h>

//--------------------屏幕布局定义--------------------
#define HEADER_HEIGHT    150
#define WAVEFORM_HEIGHT  130   
#define STATUS_BAR       20    
#define VALUE_WIDTH      80    

//--------------------硬件配置--------------------
#define TFT_BL 45
#define ECG_OUT_A 3
#define SDN_A 41
#define LOH_A 37
#define LOL_A 38
#define AD8232_A 0
#define FILTER_WINDOW 5
#define DHTPIN 2
#define DHTTYPE DHT22
#define BuzzerPin 12

//--------------------MQ-2传感器配置--------------------
#define MQ2_BOARD               "ESP-32"
#define MQ2_PIN                 1 
#define MQ2_TYPE                "MQ-2"
#define MQ2_VOLT_RES            3.3
#define MQ2_ADC_RES             12
#define RATIO_MQ2_CLEAN_AIR     9.83

// 创建MQ-2传感器对象
MQUnifiedsensor mq2Sensor(MQ2_BOARD, MQ2_VOLT_RES, MQ2_ADC_RES, MQ2_PIN, MQ2_TYPE);

//--------------------全局对象--------------------
TFT_eSPI tft;
XSpaceBioV10Board Board;
MAX30105 particleSensor;
DHT dht(DHTPIN, DHTTYPE);

// ECG参数
SemaphoreHandle_t tftMutex;
float ecgFilterBuffer[FILTER_WINDOW] = {0};
int ecgFilterIndex = 0;
float ecgBaseline = 0.0;
bool isSignalValid = true;

// MAX30102参数
uint32_t irBuffer[100];
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

// 温度和气体传感器参数
float envTemperature = 0.0;
float envHumidity = 0.0;
float mq2Voltage = 0.0;        // 用于存储MQ-2读数
SemaphoreHandle_t envMutex;
SemaphoreHandle_t mq2Mutex;

//--------------------MPU6050相关全局变量--------------------
MPU6050 mpu;
bool blinkState = false;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

// 存打包的ECG数值
volatile float lastECGValue = 0.0;

//--------------------函数声明--------------------
void drawStaticUI();
void ecgTask(void *pvParameters);
void hrTask(void *pvParameters);
void environmentTask(void *pvParameters);
void mpuTask(void *pvParameters);
void updateVitalSignsDisplay();
void calibrateBaseline();
float preprocessECG(float voltage);
int voltageToY(float voltage);
void drawWaveform(int x_pos, int *last_y, int y);
void loraTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  pinMode(BuzzerPin, OUTPUT);

  // 初始化TFT
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_WHITE);
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // 初始化AD8232
  Board.AD8232_init(AD8232_A, SDN_A, LOH_A, LOL_A, ECG_OUT_A);
  Board.AD8232_Wake(AD8232_XS1);
  pinMode(LOH_A, INPUT);
  pinMode(LOL_A, INPUT);

  // 初始化Wire1用于 MAX30105
  Wire1.begin(8, 9);  
  // 初始化 MAX30105
  if (!particleSensor.begin(Wire1, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 Not Found");
    while (1);
  }
  particleSensor.setup(60, 4, 2, 400, 411, 4096);

  // 初始化 DHT22
  dht.begin();

  // 初始化 ADC
  analogReadResolution(12);  // 设置12位 ADC

  // 创建互斥锁
  tftMutex = xSemaphoreCreateMutex();
  envMutex = xSemaphoreCreateMutex();
  mq2Mutex = xSemaphoreCreateMutex();

  // 绘制静态界面并标定ECG基线
  drawStaticUI();
  calibrateBaseline();

  // 初始化Wire用于MPU6050
  Wire.begin(36, 35);
  Wire.setClock(400000);

  // 初始化 MPU6050
  mpu.initialize();
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  // 初始化并校准 MQ-2 传感器
  mq2Sensor.setRegressionMethod(1);  // 设置回归方法
  mq2Sensor.setA(574.25); 
  mq2Sensor.setB(-2.222);
  mq2Sensor.init();
  
  Serial.print("Calibrating MQ2 please wait.");
  float calcR0 = 0;
  for (int i = 0; i < 10; i++) {
    mq2Sensor.update();
    calcR0 += mq2Sensor.calibrate(RATIO_MQ2_CLEAN_AIR);
    Serial.print(".");
    delay(500);  // 每次采样间隔500ms，保证稳定性
  }
  mq2Sensor.setR0(calcR0 / 10);
  Serial.println(" done!");
  if (isinf(calcR0)) {
    Serial.println("Warning: Connection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1);
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Connection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1);
  }

  // 创建各任务
  xTaskCreatePinnedToCore(ecgTask, "ECG", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(hrTask, "HR", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(environmentTask, "Env", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(mpuTask, "MPU", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(loraTask, "LoRa", 4096, NULL, 1, NULL, 0);
}

void loop() { 
  vTaskDelete(NULL); 
}

//--------------------ECG波形任务--------------------
void ecgTask(void *pvParameters) {
  int x_pos = 0;
  int last_y = HEADER_HEIGHT + (WAVEFORM_HEIGHT / 2);

  while (1) {
    float raw = Board.AD8232_GetVoltage(AD8232_XS1);
    float processed = preprocessECG(raw);
    lastECGValue = processed;  // 保存当前处理后的ECG值
    int y = voltageToY(processed);

    if (xSemaphoreTake(tftMutex, portMAX_DELAY)) {
      drawWaveform(x_pos, &last_y, y);
      xSemaphoreGive(tftMutex);
    }

    x_pos = (x_pos >= tft.width()) ? 0 : x_pos + 1;
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//--------------------打包所有采集任务--------------------
void loraTask(void *pvParameters) {
  while (1) {
    // 读取心率和血氧数据（hrTask）
    int currentHR = heartRate;
    int currentSpO2 = spo2;

    // 读取ECG最新值
    float currentECG = lastECGValue;

    // 从环境任务中读取温湿度（互斥锁）
    float currentEnvTemp, currentEnvHum;
    if (xSemaphoreTake(envMutex, pdMS_TO_TICKS(50))) {
      currentEnvTemp = envTemperature;
      currentEnvHum = envHumidity;
      xSemaphoreGive(envMutex);
    } else {
      currentEnvTemp = currentEnvHum = 0.0;
    }

    // 从MQ-2中读取气体数据
    mq2Sensor.update();
    float mq2Reading = mq2Sensor.readSensor();
    if (xSemaphoreTake(mq2Mutex, pdMS_TO_TICKS(50))) {
      mq2Voltage = mq2Reading;
      xSemaphoreGive(mq2Mutex);
    } else {
      mq2Voltage = 0.0;
    }

    // MPU6050任务中获取角度数据
    float yaw   = ypr[0];
    float pitch = ypr[1];
    float roll  = ypr[2];

    const char* mpustatus = "Normal";
    bool buzzerState = false;

    // MPU6050阈值
    if (abs(pitch) > 1.3) {
      mpustatus = "Tumble";
      buzzerState = true;
    }
    
    // 心率阈值
    if (currentHR > 150 || currentHR < 30) {
      buzzerState = true;
    }
    
    // 血氧阈值
    if (currentSpO2 < 95) {
      buzzerState = true;
    }

    // ECG信号阈值
    if (abs(currentECG) < 1){
      buzzerState = true;
    }
    
    // 环境温度阈值
    if (currentEnvTemp > 40) {
      buzzerState = true;
    }

    // MQ2气体阈值
    if (mq2Voltage > 12){
      buzzerState = true;
    }

    digitalWrite(BuzzerPin, buzzerState ? HIGH : LOW);

    // 将所有数据打包成CSV
    Serial.printf("NO:%d,HR:%d,SpO2:%d,ECG:%.2f,Temp:%.1f,Hum:%.1f,Gas:%.2f,Status:%s\r\n",
                  1,currentHR, currentSpO2, currentECG,
                  currentEnvTemp, currentEnvHum, mq2Voltage,
                  mpustatus);

    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

//--------------------心率和血氧任务--------------------
void hrTask(void *pvParameters) {
  uint32_t partialIR[25], partialRed[25];
  
  // 初始采集
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample();
  }

  while (1) {
    // 采集新样本
    for (byte i = 0; i < 25; i++) {
      while (!particleSensor.available()) particleSensor.check();
      partialRed[i] = particleSensor.getRed();
      partialIR[i] = particleSensor.getIR();
      particleSensor.nextSample();
    }

    // 更新缓冲区
    if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(50))) {
      memmove(redBuffer, redBuffer + 25, 75 * sizeof(uint32_t));
      memmove(irBuffer, irBuffer + 25, 75 * sizeof(uint32_t));
      memcpy(redBuffer + 75, partialRed, 25 * sizeof(uint32_t));
      memcpy(irBuffer + 75, partialIR, 25 * sizeof(uint32_t));
      xSemaphoreGive(tftMutex);
    }

    // 计算参数
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, 
                                             &spo2, &validSPO2, &heartRate, &validHeartRate);

    // 更新显示
    static uint32_t lastUpdate = 0;
    if (millis() - lastUpdate > 200) {
      if (xSemaphoreTake(tftMutex, pdMS_TO_TICKS(50))) {
        updateVitalSignsDisplay();
        xSemaphoreGive(tftMutex);
        lastUpdate = millis();
      }
    }
    vTaskDelay(1);
  }
}

//--------------------环境监测任务--------------------
void environmentTask(void *pvParameters) {
  while (1) {
    // 读取DHT22数据
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    // 更新数据
    if (xSemaphoreTake(envMutex, portMAX_DELAY)) {
      envHumidity = h;
      envTemperature = t;
      xSemaphoreGive(envMutex);
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

//--------------------MPU6050任务--------------------
void mpuTask(void *pvParameters) {
  while (1) {
    if (!dmpReady) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    
    // 轮询方式获取FIFO数据
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      blinkState = !blinkState;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//--------------------静态界面绘制--------------------
void drawStaticUI() {
  xSemaphoreTake(tftMutex, portMAX_DELAY);
  
  tft.fillRect(0, 0, tft.width(), HEADER_HEIGHT, TFT_WHITE);
  
  // 心率血氧信息
  tft.setTextSize(2);
  tft.setTextColor(TFT_BLACK);
  tft.setCursor(10, 10);
  tft.print("HR:");
  tft.setCursor(10, 40);
  tft.print("SpO2:");

  // 温湿度信息
  tft.setCursor(10, 70);
  tft.print("Env:");
  tft.setCursor(10, 100);
  tft.print("Hum:");
  
  // 气体传感器信息
  tft.setCursor(10, 130);
  tft.print("Gas:");
  
  // 波形显示区
  tft.drawRect(0, HEADER_HEIGHT, tft.width(), WAVEFORM_HEIGHT, TFT_WHITE);
  
  xSemaphoreGive(tftMutex);
}

//--------------------更新生命体征显示--------------------
void updateVitalSignsDisplay() {
  static int lastHR = -1, lastSpO2 = -1;
  static float lastEnvTemp = -1, lastHum = -1, lastGas = -1;
  
  // 心率显示
  if (validHeartRate && (heartRate != lastHR)) {
    tft.fillRect(tft.width() - VALUE_WIDTH - 10, 10, VALUE_WIDTH, 30, TFT_WHITE);
    tft.setTextSize(2);
    tft.setTextColor((heartRate > 150 || heartRate < 30) ? TFT_RED : TFT_GREEN);
    tft.setCursor(tft.width() - VALUE_WIDTH - 10, 10);
    tft.printf("%03d", heartRate);
    lastHR = heartRate;
  }

  // 血氧显示
  if (validSPO2 && (spo2 != lastSpO2)) {
    tft.fillRect(tft.width() - VALUE_WIDTH - 10, 40, VALUE_WIDTH, 30, TFT_WHITE);
    tft.setTextColor(spo2 < 95 ? TFT_RED : TFT_BLUE);
    tft.setCursor(tft.width() - VALUE_WIDTH - 10, 40);
    tft.printf("%02d%%", spo2);
    lastSpO2 = spo2;
  }

  // 环境温度与湿度显示
  float currentEnvTemp, currentHum;
  if (xSemaphoreTake(envMutex, pdMS_TO_TICKS(50))) {
    currentEnvTemp = envTemperature;
    currentHum = envHumidity;
    xSemaphoreGive(envMutex);
    
    if (currentEnvTemp != lastEnvTemp) {
      tft.fillRect(tft.width() - VALUE_WIDTH - 10, 70, VALUE_WIDTH, 20, TFT_WHITE);
      tft.setTextSize(2);
      tft.setTextColor(TFT_ORANGE);
      tft.setCursor(tft.width() - VALUE_WIDTH - 10, 70);
      tft.printf("%.1fC", currentEnvTemp);
      lastEnvTemp = currentEnvTemp;
    }

    if (currentHum != lastHum) {
      tft.fillRect(tft.width() - VALUE_WIDTH - 10, 100, VALUE_WIDTH, 20, TFT_WHITE);
      tft.setTextColor(TFT_CYAN);
      tft.setCursor(tft.width() - VALUE_WIDTH - 10, 100);
      tft.printf("%.1f%%", currentHum);
      lastHum = currentHum;
    }
  }

  float currentGas;
  if (xSemaphoreTake(mq2Mutex, pdMS_TO_TICKS(50))) {
    currentGas = mq2Voltage;
    xSemaphoreGive(mq2Mutex);
    
    if (currentGas != lastGas) {
      tft.fillRect(tft.width() - VALUE_WIDTH - 10, 130, VALUE_WIDTH, 20, TFT_WHITE);
      tft.setTextSize(2);
      tft.setTextColor(currentGas > 1.0 ? TFT_RED : TFT_DARKGREY);
      tft.setCursor(tft.width() - VALUE_WIDTH - 10, 130);
      tft.printf("%.2f", currentGas);
      lastGas = currentGas;
    }
  }
}

//--------------------ECG处理相关函数--------------------
void calibrateBaseline() {
  float sum = 0;
  for (int i = 0; i < 100; i++) {
    sum += Board.AD8232_GetVoltage(AD8232_XS1);
    delay(10);
  }
  ecgBaseline = sum / 100.0;
}

float preprocessECG(float voltage) {
  if (voltage < 0.15 || voltage > 3.3) return ecgBaseline;
  
  ecgFilterBuffer[ecgFilterIndex++] = voltage;
  ecgFilterIndex %= FILTER_WINDOW;
  
  float sum = 0;
  for (int i = 0; i < FILTER_WINDOW; i++)
    sum += ecgFilterBuffer[i];
  
  return (sum / FILTER_WINDOW) - ecgBaseline;
}

int voltageToY(float voltage) {
  const int DISPLAY_TOP = HEADER_HEIGHT + 20;
  const int DISPLAY_BOTTOM = HEADER_HEIGHT + WAVEFORM_HEIGHT - 20;
  return map(constrain(voltage * 1000, -500, 1500), -500, 1500, DISPLAY_BOTTOM, DISPLAY_TOP);
}

void drawWaveform(int x_pos, int *last_y, int y) {
  if (x_pos == 0) {
    tft.fillRect(1, HEADER_HEIGHT + 1, tft.width() - 2, WAVEFORM_HEIGHT - 2, TFT_WHITE);
  }

  if (x_pos > 0) {
    tft.drawLine(x_pos - 1, *last_y, x_pos, y, TFT_RED);
    if (x_pos % 5 == 0) {
      tft.drawFastVLine(x_pos + 2, HEADER_HEIGHT + 1, WAVEFORM_HEIGHT - 2, TFT_WHITE);
    }
  }
  *last_y = y;
}
