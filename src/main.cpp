#include <Arduino.h>
// #include <micro_ros_platformio.h>
#include "micro_rosso.h"

#include <Wire.h>

#define I2C_SCL 22
#define I2C_SDA 21

#include "micro_rosso_bno08x.h"
ImuBNO08x imu;
// #include "imu_mpu6050.h"
// ImuMPU6050 imu;

#include "mobility_tracked.h"
MobilityTracked mobility;

#include "micro_rosso_bme680.h"
EnvBME680 env_sensor;

#include "ticker.h"
Ticker ticker;

#include "sync_time.h"
SyncTime sync_time;

#include "ros_status.h"
RosStatus ros_status;

void setup()
{
  D_println("Booting...");

  Wire.begin(I2C_SDA, I2C_SCL);

  D_print("Setting up transport... ");
#if defined(MICRO_ROS_TRANSPORT_ARDUINO_WIFI)
  // if Wifi.config not called, will use DHCP
  // WiFi.config(MICRO_ROS_TRANSPORT_WIFI_STATIC_IP,MICRO_ROS_TRANSPORT_WIFI_STATIC_GATEWAY, MICRO_ROS_TRANSPORT_WIFI_STATIC_SUBNET);
  set_microros_wifi_transports(ssid, pass, agent_ip, agent_port);
#elif defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
#endif
  D_println("Done.");

  if (!micro_rosso::setup("oruga_rclc"))
  {
    D_println("FAIL micro_rosso.setup()");
  }

  if (!imu.setup(Wire))
  {
    D_println("FAIL imu.setup()");
  };

  if (!env_sensor.setup(Wire))
  {
    D_println("FAIL env_sensor.setup()");
  };

  if (!mobility.setup())
  {
    D_println("FAIL mobility.setup()");
  };

  if (!ticker.setup())
  {
    D_println("FAIL ticker.setup()");
  };

  if (!sync_time.setup())
  {
    D_println("FAIL sync_time.setup()");
  };

  if (!ros_status.setup())
  {
    D_println("FAIL ros_status.setup()");
  };

  D_println("Boot completed.");
}

void loop()
{
  micro_rosso::loop();
}