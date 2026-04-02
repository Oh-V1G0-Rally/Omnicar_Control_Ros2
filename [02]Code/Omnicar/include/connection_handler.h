#pragma once

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFi.h>

// Forward declaration of the Robot class
class Robot;

class ConnHandler
{
 protected:

  WiFiClient m_esp_client_;
  PubSubClient m_client_;

  Robot* m_robot_;

 public:

  ConnHandler();

  void init(Robot* robot);
  void reconnectMQTTBroker();
  void mqttCallback(char* topic, byte* payload, unsigned int length);
  void sendMQTTmsg(const char* topic_name, const char* message);

  void brokerLoop();

  void pubDt(unsigned long dt);
  void pubPose(float x, float y, float th);
  void pubData(unsigned long dt, int32_t pwm_r, int32_t pwm_l, float w_r,
               float w_l, float x, float y, float th, float v_ref, float w_ref);
  void pubPico(int error, float dist, float angle);
  void pubLdr(int read);
  void pubState();
};
