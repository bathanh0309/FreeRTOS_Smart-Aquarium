#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <EEPROM.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>


//---------------------------------------------------
const char *service_name = "ESP_C3_Device";
const char *pop = "12345678";  // Proof of Possession
//---------------------------------------------------
#define EEPROM_SIZE 4
//---------------------------------------------------
// Device names
char device1[] = "Den";
char device2[] = "Cho an";
char device3[] = "Loc";
//---------------------------------------------------
// GPIO settings
static uint8_t RELAY_1 = 3;
static uint8_t RELAY_2 = 10;
static uint8_t RELAY_3 = 1;
static uint8_t WIFI_LED = 2;    // Built-in LED
static uint8_t gpio_reset = 0;  // Reset button
//---------------------------------------------------
// Relay states
bool STATE_RELAY_1 = LOW;
bool STATE_RELAY_2 = LOW;
bool STATE_RELAY_3 = LOW;
//---------------------------------------------------
static Switch my_switch1(device1, &RELAY_1);
static Switch my_switch2(device2, &RELAY_2);
static Switch my_switch3(device3, &RELAY_3);
//---------------------------------------------------
// Queue for relay control
QueueHandle_t relayQueue;


// Structure to hold relay control data
struct RelayCommand {
  int relay_no;
  int relay_pin;
  bool status;
};


//---------------------------------------------------
void sysProvEvent(arduino_event_t *sys_event) {
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
      Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
      printQR(service_name, pop, "softap");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("\nConnected to Wi-Fi!");
      digitalWrite(WIFI_LED, HIGH);
      break;
  }
}


void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx) {
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();
  RelayCommand cmd;


  if (strcmp(device_name, device1) == 0 && strcmp(param_name, "Power") == 0) {
    STATE_RELAY_1 = val.val.b;
    cmd = {1, RELAY_1, STATE_RELAY_1};
    xQueueSend(relayQueue, &cmd, portMAX_DELAY);
  } else if (strcmp(device_name, device2) == 0 && strcmp(param_name, "Power") == 0) {
    STATE_RELAY_2 = val.val.b;
    cmd = {2, RELAY_2, STATE_RELAY_2};
    xQueueSend(relayQueue, &cmd, portMAX_DELAY);
  } else if (strcmp(device_name, device3) == 0 && strcmp(param_name, "Power") == 0) {
    STATE_RELAY_3 = val.val.b;
    cmd = {3, RELAY_3, STATE_RELAY_3};
    xQueueSend(relayQueue, &cmd, portMAX_DELAY);
  }
}


void control_relay(int relay_no, int relay_pin, boolean status) {
  digitalWrite(relay_pin, status);
  EEPROM.write(relay_no - 1, status);
  EEPROM.commit();
  Serial.printf("Relay %d is %s\n", relay_no, status ? "ON" : "OFF");
}


// Task to handle relay control from queue
void RelayControlTask(void *pvParameters) {
  RelayCommand cmd;
  while (1) {
    if (xQueueReceive(relayQueue, &cmd, portMAX_DELAY)) {
      control_relay(cmd.relay_no, cmd.relay_pin, cmd.status);
    }
  }
}


// Task to monitor reset button
void ResetButtonTask(void *pvParameters) {
  while (1) {
    if (digitalRead(gpio_reset) == LOW) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      int startTime = millis();
      while (digitalRead(gpio_reset) == LOW) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
      }
      int endTime = millis();


      if ((endTime - startTime) > 4000) {
        Serial.println("Factory Reset");
        RMakerFactoryReset(2);
      } else if ((endTime - startTime) > 3000) {
        Serial.println("WiFi Reset");
        RMakerWiFiReset(2);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// Task to monitor WiFi status
void WiFiStatusTask(void *pvParameters) {
  while (1) {
    if (WiFi.status() != WL_CONNECTED) {
      digitalWrite(WIFI_LED, LOW);
    } else {
      digitalWrite(WIFI_LED, HIGH);
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}


// New task to monitor relay states
void MonitorRelayTask(void *pvParameters) {
  while (1) {
    Serial.printf("Relay States: Den=%s, Cho an=%s, Loc=%s\n",
                  STATE_RELAY_1 ? "ON" : "OFF",
                  STATE_RELAY_2 ? "ON" : "OFF",
                  STATE_RELAY_3 ? "ON" : "OFF");
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Print every 5 seconds
  }
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);


  // Initialize pins
  pinMode(RELAY_1, OUTPUT);
  pinMode(RELAY_2, OUTPUT);
  pinMode(RELAY_3, OUTPUT);
  pinMode(gpio_reset, INPUT_PULLUP);
  pinMode(WIFI_LED, OUTPUT);


  digitalWrite(RELAY_1, !STATE_RELAY_1);
  digitalWrite(RELAY_2, !STATE_RELAY_2);
  digitalWrite(RELAY_3, !STATE_RELAY_3);
  digitalWrite(WIFI_LED, LOW);


  // Create queue for relay commands
  relayQueue = xQueueCreate(10, sizeof(RelayCommand));
  if (relayQueue == NULL) {
    Serial.println("Failed to create queue");
    return;
  }


  // Initialize RainMaker
  Node my_node = RMaker.initNode("Tech Trends Shameer");
  my_switch1.addCb(write_callback);
  my_node.addDevice(my_switch1);
  my_node.addDevice(my_switch2);
  my_node.addDevice(my_switch3);
  my_switch2.addCb(write_callback);
  my_switch3.addCb(write_callback);


  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();


  uint32_t chipId = 0;
  for (int i = 0; i < 17; i += 8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }


  Serial.printf("\nChip ID: %d Service Name: %s\n", chipId, service_name);
  Serial.println("Starting ESP RainMaker...");


  RMaker.start();
  WiFi.onEvent(sysProvEvent);


  WiFiProv.beginProvision(
    NETWORK_PROV_SCHEME_SOFTAP,
    NETWORK_PROV_SCHEME_HANDLER_NONE,
    NETWORK_PROV_SECURITY_1,
    pop,
    service_name
  );


  // Restore relay states from EEPROM
  STATE_RELAY_1 = EEPROM.read(0);
  digitalWrite(RELAY_1, STATE_RELAY_1);
  my_switch1.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_1);
  Serial.printf("Relay1 is %s\n", STATE_RELAY_1 ? "ON" : "OFF");


  STATE_RELAY_2 = EEPROM.read(1);
  digitalWrite(RELAY_2, STATE_RELAY_2);
  my_switch2.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_2);
  Serial.printf("Relay2 is %s\n", STATE_RELAY_2 ? "ON" : "OFF");


  STATE_RELAY_3 = EEPROM.read(2);
  digitalWrite(RELAY_3, STATE_RELAY_3);
  my_switch3.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, STATE_RELAY_3);
  Serial.printf("Relay3 is %s\n", STATE_RELAY_3 ? "ON" : "OFF");


  // Create FreeRTOS tasks
  xTaskCreate(RelayControlTask, "RelayControlTask", 2048, NULL, 5, NULL);
  xTaskCreate(ResetButtonTask, "ResetButtonTask", 2048, NULL, 4, NULL);
  xTaskCreate(WiFiStatusTask, "WiFiStatusTask", 2048, NULL, 3, NULL);
  xTaskCreate(MonitorRelayTask, "MonitorRelayTask", 2048, NULL, 2, NULL);
}


void loop() {
  // Empty: FreeRTOS handles tasks
}

