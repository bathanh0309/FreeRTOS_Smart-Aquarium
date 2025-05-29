#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>




// Cấu hình chân
const uint8_t BUTTON_MODE = 19;      // Nút chuyển chế độ
const uint8_t BUTTON_FILTER = 5;     // Nút điều khiển lọc
const uint8_t BUTTON_LIGHT = 4;      // Nút điều khiển đèn
const uint8_t BUTTON_FEED = 18;      // Nút kích hoạt motor cho ăn
const uint8_t MOTOR_FILTER = 1;      // Motor lọc
const uint8_t MOTOR_LIGHT = 3;       // Đèn sưởi
const uint8_t MOTOR_FEED = 10;       // Motor cho ăn
const uint8_t TEMP_SENSOR_PIN = 2;   // Cảm biến nhiệt độ DS18B20
const uint8_t CLARITY_SENSOR_PIN = 0; // Cảm biến độ trong




// Cấu hình OLED
const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 64;
const int8_t OLED_RESET = -1;
const uint8_t SDA_PIN = 6;
const uint8_t SCL_PIN = 7;




// Cấu hình PWM
const uint8_t PWM_OFF = 0;
const uint8_t PWM_WEAK = 100;
const uint8_t PWM_AN = 25; // xung cho an
const uint8_t PWM_STRONG = 255;




// Cấu hình thời gian cho ăn
const char* FEEDING_TIME = "11:00";  // Thời gian cho ăn
const uint32_t FEEDING_DURATION = 3000; // Thời gian chạy motor (ms)
uint8_t lastFeedDay = 255;           // Ngày cuối cùng motor cho ăn




// Cấu hình debounce
const uint32_t DEBOUNCE_DELAY = 200; // ms




// Ngưỡng cảm biến
const uint16_t CLARITY_CLEAN = 3000;  // Nước rất trong
const uint16_t CLARITY_DIRTY = 2000;  // Nước kém trong
const float TEMP_COOL = 30.0;
const float TEMP_WARM = 32.0;




// Khởi tạo đối tượng
TwoWire I2C = TwoWire(0);
Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C, OLED_RESET);
RTC_DS3231 rtc;
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature tempSensor(&oneWire);




// Định nghĩa trạng thái
enum DeviceMode { AUTO, MANUAL };
enum DeviceState { OFF, WEAK, STRONG };
enum ButtonCommand { MODE_TOGGLE, FILTER_TOGGLE, LIGHT_TOGGLE, FEED_ACTIVATE };




// Cấu trúc dữ liệu
struct SensorData {
  float temperature;
  uint16_t clarity;
};




struct AquariumState {
  DeviceMode mode = AUTO;
  DeviceState filter = OFF;
  DeviceState light = OFF;
  float temperature = 0.0;
  uint16_t clarity = 0;
  bool feedActive = false;
  DateTime time;
};




// Khởi tạo queues
QueueHandle_t sensorQueue;
QueueHandle_t buttonQueue;
QueueHandle_t stateQueue;




// Khởi tạo trạng thái
AquariumState aquarium;




// Biến debounce cho interrupt
volatile uint32_t lastButtonTimes[4] = {0, 0, 0, 0}; // Thời gian ngắt cuối cùng cho mỗi nút




// Áp dụng trạng thái PWM
void applyPWM(uint8_t pin, DeviceState state) {
  uint8_t pwmVal = (state == OFF) ? PWM_OFF : (state == WEAK) ? PWM_WEAK : PWM_STRONG;
  analogWrite(pin, pwmVal);
}




// Hàm xử lý ngắt chung
void IRAM_ATTR handleButtonInterrupt(void* arg) {
  uint8_t buttonIndex = (uint8_t)(uintptr_t)arg;
  uint32_t currentTime = millis();




  // Kiểm tra debounce
  if (currentTime - lastButtonTimes[buttonIndex] > DEBOUNCE_DELAY) {
    ButtonCommand cmd;
    switch (buttonIndex) {
      case 0: cmd = MODE_TOGGLE; break;
      case 1: cmd = FILTER_TOGGLE; break;
      case 2: cmd = LIGHT_TOGGLE; break;
      case 3: cmd = FEED_ACTIVATE; break;
      default: return;
    }
    // Gửi lệnh vào queue từ ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(buttonQueue, &cmd, &xHigherPriorityTaskWoken);
    lastButtonTimes[buttonIndex] = currentTime;




    // Đánh thức task nếu cần
    if (xHigherPriorityTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}




// Task đọc cảm biến
void SensorTask(void *pvParameters) {
  while (1) {
    SensorData data;
    data.clarity = analogRead(CLARITY_SENSOR_PIN);
    tempSensor.requestTemperatures();
    data.temperature = tempSensor.getTempCByIndex(0);
   
    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500)); // Đọc mỗi 500ms
  }
}




// Task điều khiển
void ControlTask(void *pvParameters) {
  SensorData sensorData;
  ButtonCommand buttonCmd;
  TickType_t feedStartTime = 0;
  bool feedInProgress = false;




  while (1) {
    // Cập nhật thời gian
    aquarium.time = rtc.now();




    // Nhận dữ liệu cảm biến
    if (xQueueReceive(sensorQueue, &sensorData, 0)) {
      aquarium.temperature = sensorData.temperature;
      aquarium.clarity = sensorData.clarity;




      // Điều khiển đèn sưởi trong chế độ Auto
      if (aquarium.mode == AUTO) {
        if (sensorData.temperature < TEMP_COOL) {
          aquarium.light = STRONG; // Nhiệt độ < 34°C
        } else if (sensorData.temperature >= TEMP_COOL && sensorData.temperature < TEMP_WARM) {
          aquarium.light = WEAK; // 34°C <= Nhiệt độ < 35°C
        } else {
          aquarium.light = OFF; // Nhiệt độ >= 35°C
        }
        applyPWM(MOTOR_LIGHT, aquarium.light);
      }
    }




    // Xử lý nút nhấn từ queue (từ interrupt)
    if (xQueueReceive(buttonQueue, &buttonCmd, 0)) {
      if (buttonCmd == MODE_TOGGLE) {
        aquarium.mode = (aquarium.mode == AUTO) ? MANUAL : AUTO;
      } else if (aquarium.mode == MANUAL) {
        if (buttonCmd == FILTER_TOGGLE) {
          aquarium.filter = static_cast<DeviceState>((aquarium.filter + 1) % 3);
          applyPWM(MOTOR_FILTER, aquarium.filter);
        } else if (buttonCmd == LIGHT_TOGGLE) {
          aquarium.light = static_cast<DeviceState>((aquarium.light + 1) % 3);
          applyPWM(MOTOR_LIGHT, aquarium.light);
        } else if (buttonCmd == FEED_ACTIVATE && !feedInProgress) {
          aquarium.feedActive = true;
          feedInProgress = true;
          feedStartTime = xTaskGetTickCount();
          //applyPWM(MOTOR_FEED, WEAK);
          analogWrite(MOTOR_FEED, PWM_AN);
        }
      }
    }




    // Điều khiển tự động
    if (aquarium.mode == AUTO) {
      if (aquarium.clarity > CLARITY_CLEAN) {
        aquarium.filter = OFF;
      } else if (aquarium.clarity > CLARITY_DIRTY) {
        aquarium.filter = WEAK;
      } else {
        aquarium.filter = STRONG;
      }
      applyPWM(MOTOR_FILTER, aquarium.filter);




      // Kiểm tra thời gian cho ăn
      uint8_t feedHour, feedMinute;
      sscanf(FEEDING_TIME, "%hhu:%hhu", &feedHour, &feedMinute);
      if (aquarium.time.hour() == feedHour && aquarium.time.minute() == feedMinute &&
          aquarium.time.day() != lastFeedDay && !feedInProgress) {
        aquarium.feedActive = true;
        feedInProgress = true;
        feedStartTime = xTaskGetTickCount();
        applyPWM(MOTOR_FEED, WEAK);
        lastFeedDay = aquarium.time.day();
      }
    }




    // Kiểm tra thời gian cho ăn
    if (feedInProgress && (xTaskGetTickCount() - feedStartTime) >= pdMS_TO_TICKS(FEEDING_DURATION)) {
      aquarium.feedActive = false;
      feedInProgress = false;
      applyPWM(MOTOR_FEED, OFF);
    }




    // Gửi trạng thái đến queue hiển thị/in
    xQueueOverwrite(stateQueue, &aquarium);
    vTaskDelay(pdMS_TO_TICKS(100)); // Chạy mỗi 100ms
  }
}




// Task hiển thị OLED
void DisplayTask(void *pvParameters) {
  AquariumState state;
  while (1) {
    if (xQueuePeek(stateQueue, &state, portMAX_DELAY)) {
      display.clearDisplay();
      display.setTextColor(SH110X_WHITE);
      display.setTextSize(1);




      display.setCursor(0, 0);
      display.print("Mode: ");
      display.println(state.mode == AUTO ? "AUTO" : "MANUAL");




      display.setCursor(0, 10);
      display.printf("Time: %02d:%02d:%02d\n", state.time.hour(), state.time.minute(), state.time.second());
      display.setCursor(0, 20);
      display.printf("Date: %02d/%02d/%04d\n", state.time.day(), state.time.month(), state.time.year());




      display.setCursor(0, 30);
      display.print("Filter:");
      display.print(state.filter == OFF ? "OFF" : state.filter == WEAK ? "WEAK" : "STRONG");
      display.setCursor(80, 30);
      display.print("Cl:");
      display.print(state.clarity);




      display.setCursor(0, 40);
      display.print("Light:");
      display.print(state.light == OFF ? "OFF" : state.light == WEAK ? "WEAK" : "STRONG");
      display.setCursor(80, 40);
      display.print("T:");
      state.temperature < 0 ? display.print("Err") : display.print(state.temperature, 1);
      display.print("C");




      display.setCursor(0, 50);
      display.print("Feed:");
      display.print(FEEDING_TIME);
      display.setCursor(65, 50);
      display.print("Water:");
      if (state.clarity > CLARITY_CLEAN) {
        display.print("Clr");
      } else if (state.clarity > CLARITY_DIRTY) {
        display.print("Cldy");
      } else {
        display.print("Mrky");
      }




      display.display();
    }
    vTaskDelay(pdMS_TO_TICKS(200)); // Cập nhật mỗi 200ms
  }
}




// Task in Serial
void SerialPrintTask(void *pvParameters) {
  AquariumState state;
  while (1) {
    if (xQueuePeek(stateQueue, &state, portMAX_DELAY)) {
      Serial.printf("Mode: %s  Time: %02d:%02d:%02d  Date: %02d/%02d/%04d\n",
                    state.mode == AUTO ? "AUTO" : "MANUAL",
                    state.time.hour(), state.time.minute(), state.time.second(),
                    state.time.day(), state.time.month(), state.time.year());
      Serial.print("Temp: ");
      state.temperature < 0 ? Serial.print("Err") : Serial.print(state.temperature, 1);
      Serial.print("C  Clrt: ");
      Serial.print(state.clarity);
      Serial.print(" (");
      if (state.clarity > CLARITY_CLEAN) {
        Serial.print("Clr");
      } else if (state.clarity > CLARITY_DIRTY) {
        Serial.print("Cldy");
      } else {
        Serial.print("Mrky");
      }
      Serial.print(")  Filter: ");
      Serial.print(state.filter == OFF ? "OFF" : state.filter == WEAK ? "WEAK" : "STRONG");
      Serial.print("  Light: ");
      Serial.print(state.light == OFF ? "OFF" : state.light == WEAK ? "WEAK" : "STRONG");
      Serial.print("  Feed: ");
      Serial.println(state.feedActive ? "ON" : FEEDING_TIME);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // In mỗi 1s
  }
}




void setup() {
  Serial.begin(115200);
  Serial.println("Smart Aquarium System (FreeRTOS with Interrupts)");




  // Cấu hình chân
  uint8_t inputPins[] = {BUTTON_MODE, BUTTON_FILTER, BUTTON_LIGHT, BUTTON_FEED};
  uint8_t outputPins[] = {MOTOR_FILTER, MOTOR_LIGHT, MOTOR_FEED};
  for (auto pin : inputPins) pinMode(pin, INPUT_PULLUP);
  for (auto pin : outputPins) {
    pinMode(pin, OUTPUT);
    analogWrite(pin, PWM_OFF);
  }




  // Khởi tạo I2C
  I2C.begin(SDA_PIN, SCL_PIN);




  // Khởi tạo OLED
  if (!display.begin()) {
    Serial.println("Khởi tạo OLED thất bại!");
    while (1);
  }




  // Khởi tạo RTC
  if (!rtc.begin(&I2C)) {
    Serial.println("Khởi tạo RTC thất bại!");
    while (1);
  }




  // Khởi tạo cảm biến nhiệt độ
  tempSensor.begin();
  Serial.print("Số cảm biến DS18B20: ");
  Serial.println(tempSensor.getDeviceCount());




  // khởi tạo ngày tháng năm giờ phút giây
  //rtc.adjust(DateTime(2025, 5, 29, 11, 26, 0));




  // Cấu hình OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.display();




  // Tạo queues
  sensorQueue = xQueueCreate(10, sizeof(SensorData));
  buttonQueue = xQueueCreate(10, sizeof(ButtonCommand));
  stateQueue = xQueueCreate(1, sizeof(AquariumState));




  // Cấu hình interrupt cho các nút
  attachInterruptArg(BUTTON_MODE, handleButtonInterrupt, (void*)0, FALLING);
  attachInterruptArg(BUTTON_FILTER, handleButtonInterrupt, (void*)1, FALLING);
  attachInterruptArg(BUTTON_LIGHT, handleButtonInterrupt, (void*)2, FALLING);
  attachInterruptArg(BUTTON_FEED, handleButtonInterrupt, (void*)3, FALLING);




  // Tạo tasks
  xTaskCreate(SensorTask, "SensorTask", 2048, NULL, 2, NULL);
  xTaskCreate(ControlTask, "ControlTask", 4096, NULL, 3, NULL);
  xTaskCreate(DisplayTask, "DisplayTask", 4096, NULL, 1, NULL);
  xTaskCreate(SerialPrintTask, "SerialPrintTask", 4096, NULL, 1, NULL);
}




void loop() {
  // Không cần loop, FreeRTOS quản lý tasks
}

