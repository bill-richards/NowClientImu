#include <Gsdc_SSD1306.h>
#include "DataTaskRunner.h"

const char BOARD_ID[16] = "IMU Sensor";

DEFAULT_NVM()
DataTaskRunner * _taskRunner;
Gsdc_SSD1306 _display(0x3C, SDA, SCL);

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char buffer[11];
  _display.clear();

  _display.leftHeading("Position");

  sprintf(buffer, "Pitch: %f", _taskRunner->getPitch());
  _display.left(MIDDLE, buffer);

  sprintf(buffer, "Roll : %f", _taskRunner->getRoll());
  _display.left(BOTTOM, buffer);
}

String _serverMacAddress;

void setup() {
  Serial.begin(250000);

  Wire.begin();
  _display.begin();

  READ_CREDENTIALS_FROM_EEPROM_AND_CONNECT_TO_WIFI_AS_A_STATION()
  _serverMacAddress = SERVER_MAC_ADDRESS_STRING()                         // TODO: Create the mac address from the string
  CALL_AND_WAIT_500_MILLIS(_display.leftHeading("INITIALIZE"))
  CALL_AND_WAIT_500_MILLIS(_display.left(MIDDLE, "ESP Now"))
  INITIALIZE_ESP_NOW_AND_REGISTER_SEND_HANDLER(OnDataSent)
  REGISTER_ESP_NOW_PEER(local_broadcastAddress)
  CALL_AND_WAIT_500_MILLIS(_display.important("Setup Complete"))
  _display.scrub();
  
  _display.scroll(MIDDLE, BOARD_ID);
  _taskRunner = new DataTaskRunner(BOARD_ID, _serverMacAddress, 2);
  _taskRunner->begin();
}

void loop() 
{
  delay(50);
} 