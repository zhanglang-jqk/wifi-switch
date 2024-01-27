/***********************************************************************
 * @file 	:	main.cpp
 * @author	:	CH
 * @brief	:
 * @Copyright (C)  2024-01-30  .cdWFVCEL. all right reserved
 ***********************************************************************/

/* include ------------------------------------------------------------------------------------------------- */
// #include "main.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <NTPClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
// #include <ModbusSlave.h>
#include <iostream>
/* macro definition ------------------------------------------------------------------------------------------------- */
#define tp() Serial.printf("%s:%d\r\n", __FILE__, __LINE__)

#define SOFT_VERSION "1.1"

#define LED_RELAY_STAT 9
#define LED_SIGNAL 10
#define ON_LED_RELAY 0
#define OFF_LED_RELAY 1
#define ON_LED_SIGNAL 0
#define OFF_LED_SIGNAL 1

#define RELAY_GPIO 4
#define KEY_GPIO 14

#define SW_DELAY_ONOFF_MS (5000 * 60) // 5分钟
#define MB_DATASIZE 1000              // modbus 1帧数据最大的字节数决定,扩展为1000字节(modbus默认仅支持255字节)
/* type definition ------------------------------------------------------------------------------------------------- */
class ModbusSlave;
// class Paramer;
typedef struct
{
  // uint8_t fsm = 0;
  uint32_t ms_st = 0;
  bool flag = false;
} DelayOnOffFsm_t;

class Paramer
{
public:
  String soft_version; // 程序中,程序对外的接口
  String client_id;
  String drive_no;
  String ssid;
  String password;
  String mqtt_server;
  uint16_t mqtt_port;
  String mqtt_username;
  String mqtt_password;
  String group_ctrl_topic;
  String group_stat_topic;
  String pub_ctrl_topic;
  String pub_stat_topic;
  String ctrl_topic;
  String stat_topic;
  String modify_config_topic;
  String read_config_topic;
  Paramer();
  void ResetParamToDefault();
  void Load();
  void Save();
  void Print();
  String GetParamsJsonStr();
  // private:
  StaticJsonDocument<MB_DATASIZE> jdoc; // 方便保存到EEPROM中
};

class ModbusSlave
{
public:
  HardwareSerial &_serial;
  uint8_t recv_buf[MB_DATASIZE + 10]; // 1帧数据最大的字节数决定
  ModbusSlave(HardwareSerial &serial);
  // @brief 计算 modbus crc16
  uint16_t modbus_crc16(uint8_t *buf, uint16_t len);
  void poll();
};

/* variable declaration ------------------------------------------------------------------------------------------------- */
/* function declaration ------------------------------------------------------------------------------------------------- */
void ConnWifi();
uint8_t ConnMQTT_Server();
void PublicKWStatMsg(const char *topic);
void SubscribeTopic();
void KeyInterrupt();
void MqttRecv_cb(char *topic, byte *payload, uint16_t length);
uint8_t ReadHoldReg_cb(uint8_t fc, uint16_t address, uint16_t length);
uint8_t WriteHoldReg_cb(uint8_t fc, uint16_t address, uint16_t length);
void LED_pool();
void RelayOn();
void RelayOff();
/* variable definition ------------------------------------------------------------------------------------------------- */
WiFiUDP NTP_UDP;
NTPClient time_client(NTP_UDP, "pool.ntp.org");
String sw_on_cmd = "sw_on_cmd", sw_off_cmd = "sw_off_cmd", delay_sw_on_cmd = "delay_sw_on_cmd", delay_sw_off_cmd = "delay_sw_off_cmd", get_sw_status_cmd = "get_sw_status_cmd";
Ticker led_ticker;
uint8_t g_tmp_buf[1024] = {0};
ModbusSlave md_slave(Serial);
static const char mqtt_ca_crt[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD
QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB
CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97
nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt
43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P
T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4
gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO
BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR
TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw
DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr
hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg
06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF
PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls
YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk
CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=
-----END CERTIFICATE-----
)EOF";

BearSSL::WiFiClientSecure wifi_client;
/* set SSL/TLS certificate */
BearSSL::X509List mqttcert(mqtt_ca_crt);
PubSubClient mqtt_client(wifi_client);
bool is_conn_server = false;
StaticJsonDocument<MB_DATASIZE> jdoc;
Paramer paramer;
DelayOnOffFsm_t delay_on_fsm, delay_off_fsm;
/* function implementation ------------------------------------------------------------------------------------------------- */

void setup()
{
  Serial.begin(115200);
  Serial.println("Power on..............");
  // pinMode(RELAY_GPIO, OUTPUT);
  // pinMode(LED_RELAY_STAT, OUTPUT);
  // pinMode(LED_SIGNAL, OUTPUT);
  // pinMode(KEY_GPIO, INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(KEY_GPIO), KeyInterrupt, FALLING);
  // digitalWrite(RELAY_GPIO, HIGH);
  // digitalWrite(LED_RELAY_STAT, ON_LED_RELAY), digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);

  // led_ticker.attach_ms(500, LED_pool);

  // Serial.println(WiFi.macAddress());
  paramer.ResetParamToDefault();
  paramer.Load();
  paramer.Print();

  WiFi.mode(WIFI_STA); // 设置ESP8266工作模式为无线终端模式
  ConnWifi();          // 连接WiFi

  mqtt_client.setClient(wifi_client);
  // Serial.printf("mqtt_server:%s \r\n",paramer.mqtt_server.c_str());
  // Serial.printf("mqtt_port:%d \r\n",paramer.mqtt_server.c_str());
  mqtt_client.setServer(paramer.mqtt_server.c_str(), paramer.mqtt_port);
  mqtt_client.setCallback(MqttRecv_cb);
  is_conn_server = ConnMQTT_Server(); // 连接MQTT服务器

  time_client.begin();
  time_client.update();
}

void loop()
{
  static uint32_t last_seconed = 0;

  // if (delay_on_fsm.flag)
  // { // relay delay on
  //   if (millis() - delay_on_fsm.ms_st >= SW_DELAY_ONOFF_MS)
  //   {
  //     // digitalWrite(RELAY_GPIO, LOW);
  //     RelayOn();
  //     delay_on_fsm.flag = false;
  //   }
  // }
  // if (delay_off_fsm.flag)
  // { // relay delay off
  //   if (millis() - delay_off_fsm.ms_st >= SW_DELAY_ONOFF_MS)
  //   {
  //     // digitalWrite(RELAY_GPIO, HIGH);
  //     RelayOff();
  //     delay_off_fsm.flag = false;
  //   }
  // }

  if (mqtt_client.connected())
  {
    if (millis() / 1000 - last_seconed >= 60) // interval 60s send public message
    {
      Serial.println("public mqtt message");
      // PublicKWStatMsg(paramer.pub_stat_topic.c_str());   // 查询开关状态;
      PublicKWStatMsg(paramer.group_stat_topic.c_str()); // 查询开关状态;
      PublicKWStatMsg(paramer.stat_topic.c_str());       // 查询开关状态;
      last_seconed = millis() / 1000;
    }
    mqtt_client.loop(); // keep mqtt alive
  }
  else
  {                    // 如果开发板未能成功连接服务器
    ConnMQTT_Server(); // 则尝试连接服务器
  }

  md_slave.poll();
}

/**
 * @brief LED 控制
 *        relay_stat_led: 继电器状态指示灯,继电器吸合时亮,断开时灭
 *        signal_led: 信号灯,联网中闪烁提示,联网成功后常亮,联网失败后灭
 */
void LED_pool() // interval 500ms callback
{
  if (is_conn_server == false) // then connecting to server and signal led blink
  {
    // signal led blink
    digitalRead(LED_SIGNAL) == ON_LED_SIGNAL ? digitalWrite(LED_SIGNAL, OFF_LED_SIGNAL) : digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);
  }
  else // connected to server and signal led on
  {
    digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);
  }

  // according to relay status to control relay stat led
  if (digitalRead(RELAY_GPIO) == LOW)
  {
    digitalWrite(LED_RELAY_STAT, ON_LED_RELAY);
  }
  else
  {
    digitalWrite(LED_RELAY_STAT, OFF_LED_RELAY);
  }
}
/**
 * @brief
 *
 * @return uint8_t 1:成功连接服务器 0:未能成功连接服务器
 */
uint8_t ConnMQTT_Server()
{
  // 根据ESP8266的MAC地址生成客户端ID（避免与其它ESP8266的客户端ID重名）
  // 连接MQTT服务器
  // Serial.println(paramer.client_id.c_str());
  // Serial.println(paramer.mqtt_username.c_str());
  // Serial.println(paramer.mqtt_password.c_str());
  if (mqtt_client.connect(paramer.client_id.c_str(), paramer.mqtt_username.c_str(), paramer.mqtt_password.c_str()))
  {
    Serial.println("MQTT Server Connected Successful.");
    // 订阅主题
    SubscribeTopic();
    return 1;
  }
  else
  {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqtt_client.state());
    return 0;
  }
}

// 发布继电器状态返回信息
void PublicKWStatMsg(const char *topic)
{
  String payload;
  jdoc["drive_no"] = paramer.drive_no;
  jdoc["timestamp"] = time_client.update(),
  time_client.getEpochTime();
  jdoc["sw_stat"] = digitalRead(RELAY_GPIO);
  serializeJson(jdoc, payload);
  bool isok = mqtt_client.publish(topic, payload.c_str());
  isok ? Serial.println("stat_topic publish ok") : Serial.println("stat_topic publish fail");
}

// 收到信息后的回调函数
void MqttRecv_cb(char *topic, byte *payload, uint16_t length)
{
  DeserializationError err = deserializeJson(jdoc, topic);
  if (err)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(err.f_str());
    return;
  }
  String cmd = String(jdoc["cmd"]);

  if (String(topic) == paramer.ctrl_topic)
  {
    Serial.println("String(topic) == ctrl_topic");
    if (cmd == sw_on_cmd) // 打开继电器命令
    {
      Serial.println("改为低电平，继电器通路");
      digitalWrite(RELAY_GPIO, LOW); // 低电平，继电器则开
    }
    if (cmd == sw_off_cmd) // 关闭继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      digitalWrite(RELAY_GPIO, HIGH); // 高电平，继电器则关
    }
    if (cmd == delay_sw_off_cmd) // 延时关闭继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      digitalWrite(RELAY_GPIO, LOW); // 低电平，继电器断开
      delay_off_fsm.flag = true;
      delay_off_fsm.ms_st = millis();
    }
    if (cmd == delay_sw_on_cmd) // 延时打开继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      //      digitalWrite(RELAY_GPIO, HIGH); // 低电平，继电器断开
      delay_on_fsm.flag = true;
      delay_on_fsm.ms_st = millis();
    }
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("收到单个查询指令，发送开关状态");
      PublicKWStatMsg(paramer.ctrl_topic.c_str()); // 查询开关状态
    }
  }
  if (String(topic) == paramer.pub_ctrl_topic)
  {
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("收到总查询指令，发送开关状态");
      PublicKWStatMsg(paramer.pub_stat_topic.c_str()); // 查询开关状态
    }
    if (cmd == sw_on_cmd)
    { // 开启继电器
      Serial.println("所有改为低电平，继电器通路");
      digitalWrite(RELAY_GPIO, LOW);
    }
    if (cmd == sw_off_cmd)
    { // 关闭继电器
      Serial.println("所有改为高电平，继电器断路");
      digitalWrite(RELAY_GPIO, HIGH);
    }
  }
  if (String(topic) == paramer.group_ctrl_topic)
  {
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("收到总查询指令，发送开关状态");
      PublicKWStatMsg(paramer.group_stat_topic.c_str()); // 查询开关状态
    }
    if (cmd == sw_on_cmd)
    { // 开启继电器
      Serial.println("所有改为低电平，继电器通路");
      digitalWrite(RELAY_GPIO, LOW);
    }
    if (cmd == sw_off_cmd)
    { // 关闭继电器
      Serial.println("所有改为高电平，继电器断路");
      digitalWrite(RELAY_GPIO, HIGH);
    }
  }
  if (String(topic) == paramer.read_config_topic)
  {
    String payload;
    jdoc["soft_version"] = paramer.soft_version;
    jdoc["ssid"] = paramer.ssid;
    jdoc["password"] = paramer.password;
    jdoc["mqtt_server"] = paramer.mqtt_server;
    jdoc["mqtt_port"] = paramer.mqtt_port;
    jdoc["group_ctrl_topic"] = paramer.group_ctrl_topic;
    serializeJson(jdoc, payload);
    bool isok = mqtt_client.publish(paramer.read_config_topic.c_str(), payload.c_str());
    isok ? Serial.println("read_config_topic publish ok") : Serial.println("read_config_topic publish fail");
  }
  if (String(topic) == paramer.modify_config_topic)
  {
    if (jdoc["ssid"].as<String>() != "")
      paramer.ssid = jdoc["ssid"].as<String>();
    if (jdoc["password"].as<String>() != "")
      paramer.password = jdoc["password"].as<String>();
    if (jdoc["mqtt_server"].as<String>() != "")
      paramer.mqtt_server = jdoc["mqtt_server"].as<String>();
    if (jdoc["mqtt_port"].as<String>() != "")
      paramer.mqtt_port = jdoc["mqtt_port"].as<uint16_t>();
    if (jdoc["group_ctrl_topic"].as<String>() != "")
      paramer.group_ctrl_topic = jdoc["group_ctrl_topic"].as<String>();

    paramer.Save();
  }
}
// 订阅指定主题
void SubscribeTopic()
{
  String topic = paramer.pub_ctrl_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
  topic = paramer.ctrl_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
  topic = paramer.stat_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
  topic = paramer.group_ctrl_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
  topic = paramer.modify_config_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
  topic = paramer.read_config_topic;
  mqtt_client.subscribe(topic.c_str()) ? Serial.println("subscribe " + topic + "is ok") : Serial.println("subsribe " + topic + "is fail");
}

// ESP8266连接wifi
void ConnWifi()
{
  // 设置重连超时时间和尝试次数
  WiFi.setAutoReconnect(true);        // 启用自动重新连接
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // 设置睡眠模式（在连接失败时保持唤醒）
  WiFi.persistent(true);              // 持久化配置（使重启后仍能记住WiFi配置）

  WiFi.begin(paramer.ssid, paramer.password);

  // 等待WiFi连接,成功连接后输出成功信息
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected Successful to WiFi");
  Serial.println(WiFi.SSID()); // WiFi名称
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP()); // IP
  // STEP3: Set CA Certification for TLS
  Serial.print("MAC address:\t");
  Serial.println(WiFi.macAddress());
  wifi_client.setTrustAnchors(&mqttcert);
  // if no check the CA Certification
  wifi_client.setInsecure();
}

void KeyInterrupt()
{
  uint32_t start_st = millis();
  while (!digitalRead(KEY_GPIO)) // wait for key release
  {
    if (millis() - start_st > 3000) // todo 阻塞时间过长,是否有可能导致看门狗重启,待确定
    {
      Serial.println("按键按下超过3秒，恢复出厂设置");
      paramer.ResetParamToDefault();
      paramer.Save();
      ESP.restart();
    }
    else
    {
      // reverse kw status
      digitalWrite(RELAY_GPIO, !digitalRead(RELAY_GPIO));
      Serial.println("按键按下，反转开关状态");
    }
  }
}

void RelayOn()
{
  digitalWrite(RELAY_GPIO, LOW);
}

void RelayOff()
{
  digitalWrite(RELAY_GPIO, HIGH);
}

// Paramer class implement -------------------------------------------------------------------------------------------------
Paramer::Paramer()
{
  // 构造函数的实现
}
void Paramer::ResetParamToDefault()
{
  jdoc["soft_version"] = SOFT_VERSION;
  String mac_addr = WiFi.macAddress();
  mac_addr.replace(":", "");
  jdoc["drive_no"] = mac_addr.c_str();
  jdoc["client_id"] = mac_addr.c_str();
  jdoc["ssid"] = "88888888";
  jdoc["password"] = "88888888";
  jdoc["mqtt_server"] = "hcdda1ed.ala.cn-hangzhou.emqxsl.cn";
  jdoc["mqtt_port"] = 8883;
  jdoc["mqtt_username"] = "dl2binary";
  jdoc["mqtt_password"] = "wocaonima88jqk";
  jdoc["group_ctrl_topic"] = "group_control";
  jdoc["group_stat_topic"] = "group_status";
  jdoc["pub_ctrl_topic"] = "public_control";
  jdoc["pub_stat_topic"] = "public_status";
  jdoc["ctrl_topic"] = jdoc["drive_no"].as<String>() + "/" + "contrl";
  jdoc["stat_topic"] = jdoc["drive_no"].as<String>() + "/" + "status";
  jdoc["modify_config_topic"] = "config_modify";
  jdoc["read_config_topic"] = "read_config";

  soft_version = SOFT_VERSION;
  drive_no = jdoc["drive_no"].as<String>();
  client_id = jdoc["client_id"].as<String>();
  ssid = jdoc["ssid"].as<String>();
  password = jdoc["password"].as<String>();
  mqtt_server = jdoc["mqtt_server"].as<String>();
  mqtt_port = jdoc["mqtt_port"].as<uint16_t>();
  mqtt_username = jdoc["mqtt_username"].as<String>();
  mqtt_password = jdoc["mqtt_password"].as<String>();
  group_ctrl_topic = jdoc["group_ctrl_topic"].as<String>();
  group_stat_topic = jdoc["group_stat_topic"].as<String>();
  pub_ctrl_topic = jdoc["pub_ctrl_topic"].as<String>();
  pub_stat_topic = jdoc["pub_stat_topic"].as<String>();
  ctrl_topic = jdoc["ctrl_topic"].as<String>();
  stat_topic = jdoc["stat_topic"].as<String>();
  modify_config_topic = jdoc["modify_config_topic"].as<String>();
  read_config_topic = jdoc["read_config_topic"].as<String>();

  Save();
}

void Paramer::Load()
{
  EEPROM.begin(MB_DATASIZE);
  // uint16_t datalen = 0;
  // datalen = EEPROM.get(0, datalen);
  uint16_t datalen = EEPROM.read(0) << 8 | EEPROM.read(1);
  // Serial.printf("datalen:%d\r\n", datalen);
  if (datalen == 0xffff) // first time run
  {
    Serial.println("no data in eeprom");
    ResetParamToDefault();
    // Save();
  }
  else if (datalen > MB_DATASIZE)
  {
    Serial.println("eeprom data error,reset to default");
    ResetParamToDefault();
    // Save();
  }
  else
  {
    uint8_t data[datalen];
    memset(data, 0, sizeof(data));
    for (int i = 0; i < datalen; i++)
      data[i] = EEPROM.read(i + 2);

    DeserializationError err = deserializeJson(jdoc, data);
    if (err)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(err.f_str());
      ResetParamToDefault();
      // Save();
    }
    else
    {
      // tp();
      drive_no = jdoc["drive_no"].as<String>();
      ssid = jdoc["ssid"].as<String>();
      password = jdoc["password"].as<String>();
      mqtt_server = jdoc["mqtt_server"].as<String>();
      mqtt_port = jdoc["mqtt_port"].as<uint16_t>();
      mqtt_username = jdoc["mqtt_username"].as<String>();
      mqtt_password = jdoc["mqtt_password"].as<String>();
      group_ctrl_topic = jdoc["group_ctrl_topic"].as<String>();
      pub_ctrl_topic = jdoc["pub_ctrl_topic"].as<String>();
      ctrl_topic = jdoc["ctrl_topic"].as<String>();
      stat_topic = jdoc["stat_topic"].as<String>();
      modify_config_topic = jdoc["modify_config_topic"].as<String>();
      read_config_topic = jdoc["read_config_topic"].as<String>();
    }
  }
  EEPROM.end();
}

void Paramer::Save()
{
  EEPROM.begin(MB_DATASIZE);

  jdoc["soft_version"] = soft_version;
  jdoc["drive_no"] = drive_no;
  jdoc["client_id"] = client_id;
  jdoc["ssid"] = ssid;
  jdoc["password"] = password;
  jdoc["mqtt_server"] = mqtt_server;
  jdoc["mqtt_port"] = mqtt_port;
  jdoc["mqtt_username"] = mqtt_username;
  jdoc["mqtt_password"] = mqtt_password;
  jdoc["group_ctrl_topic"] = group_ctrl_topic;
  jdoc["pub_ctrl_topic"] = pub_ctrl_topic;
  jdoc["ctrl_topic"] = ctrl_topic;
  jdoc["stat_topic"] = stat_topic;
  jdoc["modify_config_topic"] = modify_config_topic;
  jdoc["read_config_topic"] = read_config_topic;

  // 将 doc 转换成字符串
  String json_str;
  serializeJson(jdoc, json_str);
  uint16_t datalen = json_str.length() + 1;
  // Serial.printf("datalen:%d\r\n", datalen);
  EEPROM.write(0, (datalen >> 8) & 0x00ff), EEPROM.write(1, datalen & 0x00ff); // the first two bytes is the length of data
  for (int i = 0; i < datalen; i++)
  {
    EEPROM.write(i + 2, json_str[i]);
  }
  EEPROM.commit();
  EEPROM.end();
}

void Paramer::Print()
{
  String jstr;
  serializeJson(jdoc, jstr);
  Serial.println(jstr);
}

String Paramer::GetParamsJsonStr()
{
  String jstr;
  serializeJson(jdoc, jstr);
  return jstr;
}

// modbus class implement -------------------------------------------------------------------------------------------------

ModbusSlave::ModbusSlave(HardwareSerial &serial) : _serial(serial)
{
  // 构造函数的实现
  _serial.setTimeout(200);
}

// @brief 计算 modbus crc16 (LSB)
uint16_t ModbusSlave::modbus_crc16(uint8_t *buf, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {            // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else         // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

void ModbusSlave::poll()
{
  if (_serial.available())
  {
    uint16_t cnt = _serial.readBytes(recv_buf, sizeof(recv_buf)); // get a data frame
    // for (int i = 0; i < cnt; i++)
    //   Serial.printf("%02x ", recv_buf[i]);

    uint8_t addr = recv_buf[0];
    uint8_t fcode = recv_buf[1];

    if (addr == 0x01 && fcode == 0x03) // read mutimul register
    {
      { // 接收数据解析
        // 接收数据格式
        // 设备地址（1字节）：表示请求的设备的地址。
        // 功能码（1字节）：表示执行的功能，对于0x03功能码的请求，这个值应该是0x03。
        // 起始地址（2字节）：表示要读取的第一个寄存器的地址，按照Big-Endian（高字节在前）的顺序存储。
        // 寄存器数量（2字节）：表示要读取的寄存器的数量，按照Big-Endian的顺序存储。
        // CRC校验（2字节）：表示前面所有字节的CRC校验值，按照Little-Endian（低字节在前）的顺序存储。
        uint16_t start_ret = recv_buf[2] << 8 | recv_buf[3];
        uint8_t reg_cnt = recv_buf[4] << 8 | recv_buf[5];
        uint16_t recv_crc = recv_buf[6] << 8 | recv_buf[7];

        uint16_t calc_crc = modbus_crc16(recv_buf, 6);

        if (recv_crc != calc_crc)
        {
          Serial.printf("recv_crc:%04x, calc_crc:%04x\r\n", recv_crc, calc_crc);
          goto error;
        }
      }

      { // 响应数据
        // paramer.Load();
        String jstr = paramer.GetParamsJsonStr();

        // 响应报文格式
        // 设备地址（1字节）：表示响应的设备的地址
        // 功能码（1字节）：表示执行的功能，对于03功能码的应答，这个值应该是03。
        // 字节计数（2字节）：表示接下来的数据字节的数量。扩展为2字节,标准modbus协议为1字节
        // 数据（n字节）：表示读取的寄存器的值，每个寄存器的值占2字节，按照Big-Endian（高字节在前）的顺序存储。
        // CRC校验（2字节）：表示前面所有字节的CRC校验值，按照Little-Endian（低字节在前）的顺序存储。

        uint8_t addr = 0x01;
        uint8_t fcode = 0x03;
        uint16_t byte_size = MB_DATASIZE;

        uint16_t idx = 0;
        memset(g_tmp_buf, 0, sizeof(g_tmp_buf));
        g_tmp_buf[idx++] = addr, g_tmp_buf[idx++] = fcode, g_tmp_buf[idx++] = (byte_size >> 8) & 0x00ff, g_tmp_buf[idx++] = byte_size;
        memcpy(&g_tmp_buf[idx], jstr.c_str(), jstr.length()), idx += jstr.length();
        uint16_t crc = modbus_crc16(g_tmp_buf, idx);
        g_tmp_buf[idx++] = crc, g_tmp_buf[idx++] = crc >> 8;

        _serial.write(g_tmp_buf, MB_DATASIZE);
      }
    }
    else if (addr == 0x01 && fcode == 0x10) // write mutimul register
    {
      // tp();
      { // 接收数据
        // 接收数据格式
        // 设备地址（1字节）：表示请求的设备的地址。
        // 功能码（1字节）：表示执行的功能，对于0x10功能码的请求，这个值应该是0x10。
        // 起始地址（2字节）：表示要写入的第一个寄存器的地址，按照Big-Endian（高字节在前）的顺序存储。
        // 寄存器数量（2字节）：表示要写入的寄存器的数量，按照Big-Endian的顺序存储。
        // 字节计数（2字节）：表示接下来的数据字节的数量。扩展为2字节,标准modbus协议为1字节
        // 数据（n字节）：表示要写入的寄存器的值，每个寄存器的值占2字节，按照Big-Endian的顺序存储。
        // CRC校验（2字节）：表示前面所有字节的CRC校验值，按照Little-Endian（低字节在前）的顺序存储。

        // 接收数据解析
        uint16_t start_ret = recv_buf[2] << 8 | recv_buf[3];
        uint8_t reg_cnt = recv_buf[4] << 8 | recv_buf[5];
        uint16_t byte_cnt = recv_buf[6] << 8 | recv_buf[7];

        uint16_t recv_crc = recv_buf[8 + byte_cnt] << 8 | recv_buf[8 + byte_cnt + 1];

        uint16_t calc_crc = modbus_crc16(recv_buf, 1 + 1 + 2 + 2 + 2 + byte_cnt);

        if (recv_crc != calc_crc)
        {
          Serial.printf("recv_crc:%04x, calc_crc:%04x\r\n", recv_crc, calc_crc);
          goto error;
        }
      }
      { // 写数据

        uint8_t *payload = recv_buf + 8;
        uint8_t len = recv_buf[6];

        DeserializationError err = deserializeJson(jdoc, payload);
        if (err)
        {
          Serial.print(F("deserializeJson() failed: "));
          Serial.println(err.f_str());
          goto error;
        }

        if (jdoc["ssid"].as<String>() != "")
          paramer.ssid = jdoc["ssid"].as<String>();
        if (jdoc["password"].as<String>() != "")
          paramer.password = jdoc["password"].as<String>();
        if (jdoc["mqtt_server"].as<String>() != "")
          paramer.mqtt_server = jdoc["mqtt_server"].as<String>();
        if (jdoc["mqtt_port"].as<String>() != "")
          paramer.mqtt_port = jdoc["mqtt_port"].as<uint16_t>();
        if (jdoc["group_ctrl_topic"].as<String>() != "")
          paramer.group_ctrl_topic = jdoc["group_ctrl_topic"].as<String>();

        paramer.Save();
        // paramer.Print();
      }
      { // 响应报文
        // 设备地址（1字节）：表示响应的设备的地址。
        // 功能码（1字节）：表示执行的功能，对于0x10功能码的响应，这个值应该是0x10。
        // 起始地址（2字节）：表示写入的第一个寄存器的地址，按照Big-Endian（高字节在前）的顺序存储。
        // 寄存器数量（2字节）：表示写入的寄存器的数量，按照Big-Endian的顺序存储。
        // CRC校验（2字节）：表示前面所有字节的CRC校验值，按照Little-Endian（低字节在前）的顺序存储

        memset(g_tmp_buf, 0, sizeof(g_tmp_buf));
        // g_tmp_buf[0] = 0x01, g_tmp_buf[1] = 0x10, g_tmp_buf[2] = 0x0, g_tmp_buf[3] = 0x0;
        uint8_t addr = 0x01, fcode = 0x10, start_ret = 0x0, reg_cnt = 127;
        g_tmp_buf[0] = addr, g_tmp_buf[1] = fcode, g_tmp_buf[2] = (start_ret >> 8), g_tmp_buf[3] = start_ret, g_tmp_buf[4] = (reg_cnt >> 8), g_tmp_buf[5] = reg_cnt;
        uint16_t crc = modbus_crc16(g_tmp_buf, 6);
        g_tmp_buf[6] = crc, g_tmp_buf[7] = crc >> 8;
        // for (int i = 0; i < 8; i++)
        //   Serial.printf("%02x ", g_tmp_buf[i]);
        _serial.write(g_tmp_buf, 8);
      }
    }
    else // error, 解析报文失败,响应失败报文
    {
    error:
      tp();
      // 设备地址（1字节）：表示响应的设备的地址。
      // 功能码（1字节）：表示执行的功能，这个值应该是原请求的功能码 + 0x80。例如，如果请求的功能码是0x03，那么异常响应的功能码应该是0x83。
      // 异常码（1字节）：表示异常的类型。常见的异常码有：
      // 0x01：非法功能码，表示请求的功能码设备不支持。
      // 0x02：非法数据地址，表示请求的数据地址不存在。
      // 0x03：非法数据值，表示请求的数据值不合法。
      // 0x04：设备故障，表示设备在处理请求时发生了故障。
      // CRC校验（2字节）：表示前面所有字节的CRC校验值，按照Little-Endian（低字节在前）的顺序存储。
      uint8_t addr = 0x01;                          // 设备地址
      uint8_t _fcode = fcode | 0x80;                // 功能码
      uint8_t excode = 0x04;                        // 异常码，非法数据地址
      uint8_t buf[5] = {addr, fcode, excode, 0, 0}; // 初始化缓冲区
      uint16_t crc = modbus_crc16(buf, 3);          // 计算CRC校验值
      buf[3] = crc;                                 // 存储CRC校验值的低字节
      buf[4] = crc >> 8;                            // 存储CRC校验值的高字节
    }
  }

  return;
}

// main.cpp
