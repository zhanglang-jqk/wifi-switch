/**********************************************************************
  项目名称/Project          : 电路控制开关
  程序名称/Program name     : Control The Switch
  团队/Team                : YGKJ
  作者/Author              : YGKJ
***********************************************************************/
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <NTPClient.h>

#define LED_PWR 9
#define LED_SIGNAL 10
#define ON_LED_PWR 0
#define OFF_LED_PWR 1
#define ON_LED_SIGNAL 0
#define OFF_LED_SIGNAL 1

String soft_version = "1.0";

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
const char *mqttServer = "hcdda1ed.ala.cn-hangzhou.emqxsl.cn";
const int SwitchGPIO = 16; // 继电器对应GPIO号
const int brokerPort = 8883;
// 需要修改的信息
//  MQTT服务端连接用户名密码
const char *mqttUserName = "dl2binary";
const char *mqttPassword = "wocaonima88jqk";
// const char *ssid = "303";
// const char *password = "VT4009030242";
const char *ssid = "CMCC-CL4X";
const char *password = "ZQYJZ926294";

String deviceNo; // 设备编号,与mac地址相同(不带冒号)
String clientId; // 客户端ID,与mac地址相同(不带冒号)

String publicControlTopic = "public_control"; // 公共控制主题,多路控制,在需要同时控制多个电闸时使用
// String controlTopic = deviceNo + "/" + "contrl"; // 当前继电器控制主题
// String statusTopic = deviceNo + "/" + "status";  // 当前继电器状态接收主题
String controlTopic; // 当前继电器控制主题,由于需要修改,所以不在此处初始化
String statusTopic;  // 当前继电器状态接收主题,由于需要修改,所以不在此处初始化
// 需要修改的信息
String cmd;
String sw_on_cmd = "sw_on_cmd";                 // 打开电闸
String sw_off_cmd = "sw_off_cmd";               // 关闭电闸
String delay_sw_on_cmd = "delay_sw_on_cmd";     // 延时打开电闸
String delay_sw_off_cmd = "delay_sw_off_cmd";   // 延时关闭电闸
String get_sw_status_cmd = "get_sw_status_cmd"; // 查询电闸状态

Ticker ticker;
Ticker led_ticker;
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

BearSSL::WiFiClientSecure wifiClient;
/* set SSL/TLS certificate */
BearSSL::X509List mqttcert(mqtt_ca_crt);
// WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

int count; // Ticker计数用变量
// 倒计时
uint32_t startTime = 0;             // 定义一个变量来存储开始时间 计算锁门
uint32_t delayTime = 5 * 60 * 1000; // 5分钟的延迟时间（以毫秒为单位）
bool startCountdown = false;        // 布尔变量用于跟踪是否需要启动倒计时
bool startLightCountdown = false;   // 设置为true，表示需要启动倒计时
uint32_t startLightTime = 0;        // 定义一个变量来存储开始时间 计算关灯

bool en_sigled_toggle = true; // 控制信号灯闪烁
// uint32_t sigled_tog_led_cnt = 0;

void connectWifi();
void receiveCallback(char *topic, byte *payload, unsigned int length);
void tickerCount();
uint8_t connectMQTTServer();
void pubMQTTmsg();
void subscribeTopic();

Ticker timer;
void test();

void setup()
{
  Serial.begin(9600);
  pinMode(SwitchGPIO, OUTPUT);
  // pinMode(LED_PWR, OUTPUT);
  pinMode(LED_SIGNAL, OUTPUT);

  // 设置ESP8266工作模式为无线终端模式
  WiFi.mode(WIFI_STA);

  digitalWrite(SwitchGPIO, HIGH);
  digitalWrite(LED_PWR, ON_LED_PWR), digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);
  led_ticker.attach(0.5, // 联网过程中,LED闪烁提示
                    []()
                    {
                      if (en_sigled_toggle)
                      {
                        digitalWrite(LED_SIGNAL, !digitalRead(LED_SIGNAL));
                      }
                    });
  // 连接WiFi
  connectWifi();

  deviceNo = WiFi.macAddress();
  deviceNo.replace(":", "");
  clientId = deviceNo;
  controlTopic = deviceNo + "/" + "contrl"; // 当前继电器控制主题
  statusTopic = deviceNo + "/" + "status";  // 当前继电器状态接收主题

  // STEP4: Set MQTT Parameters
  mqttClient.setClient(wifiClient);
  // 设置MQTT服务器和端口号
  mqttClient.setServer(mqttServer, brokerPort);
  mqttClient.setCallback(receiveCallback);

  // 连接MQTT服务器
  uint8_t isconn = connectMQTTServer();

  if (isconn) // if connect to netword is ok
  {
    digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);
  }
  else // if connect to netword is fail
  {
    digitalWrite(LED_SIGNAL, OFF_LED_SIGNAL);
  }

  led_ticker.detach();

  // Ticker定时对象
  ticker.attach(1, tickerCount);

  timeClient.begin();
  timeClient.update();

  Serial.print("mqtt_server"), Serial.println(mqttServer);
  Serial.print("mqtt_port"), Serial.println(brokerPort);
  Serial.print("client_id:"), Serial.println(clientId);
  Serial.print("device_no:"), Serial.println(deviceNo);
}

void loop()
{
  // 如果需要启动倒计时且已经过了5分钟
  if (startCountdown && millis() - startTime >= delayTime)
  {
    digitalWrite(SwitchGPIO, HIGH); // 5分钟后触发的操作
    startCountdown = false;         // 重置标志
    startTime = 0;                  // 倒计时重置
  }
  if (startLightCountdown && millis() - startLightTime >= delayTime)
  {
    digitalWrite(SwitchGPIO, LOW); // 5分钟后触发的操作
    startLightCountdown = false;   // 重置标志
    startLightTime = 0;            // 倒计时重置
  }
  if (mqttClient.connected())
  { // 如果开发板成功连接服务器
    // Serial.println(millis());
    // 每隔60秒钟发布一次信息
    if (count >= 3600)
    {
      Serial.println("public mqtt message");
      pubMQTTmsg(); // 查询开关状态
      count = 0;
    }
    // 保持心跳
    mqttClient.loop();
  }
  else
  {                      // 如果开发板未能成功连接服务器
    connectMQTTServer(); // 则尝试连接服务器
  }
}

void tickerCount()
{
  count++;
}

/**
 * @brief
 *
 * @return uint8_t 1:成功连接服务器 0:未能成功连接服务器

 */
uint8_t connectMQTTServer()
{
  // 根据ESP8266的MAC地址生成客户端ID（避免与其它ESP8266的客户端ID重名）
  // 连接MQTT服务器
  if (mqttClient.connect(clientId.c_str(), mqttUserName, mqttPassword))
  {
    Serial.println("MQTT Server Connected Successful.");
    // 订阅主题
    subscribeTopic();
    return 1;
  }
  else
  {
    Serial.print("MQTT Server Connect Failed. Client State:");
    Serial.println(mqttClient.state());
    return 0;
  }
}

// 发布继电器状态返回信息
void pubMQTTmsg()
{
  String topicString = statusTopic;
  char publishTopic[topicString.length() + 1];
  strcpy(publishTopic, topicString.c_str());
  // String messageString = "{\"clientId\": " +"\""+ clientId+"\"";
  String clientHead = "{\"clientId\": ";
  String TimeHead = ",\"timestamp\": ";
  String slash = "\"";
  timeClient.update();
  uint32_t currentTime = timeClient.getEpochTime();
  String messageString = clientHead + slash + clientId + slash + TimeHead + currentTime;
  Serial.println("messageString: " + messageString);
  // 建立发布信息。
  messageString += ",\"Switch1Status\":" + String(digitalRead(SwitchGPIO)) + "}";
  // String messageString = "{\"clientId: " + String(clientId) + ",Switch1Status: " + String(GPIOStatus1) + ",Switch2Status: " + String(GPIOStatus2) + "}";

  char publishMsg[messageString.length() + 1];
  strcpy(publishMsg, messageString.c_str());

  // 实现ESP8266向主题发布信息
  if (mqttClient.publish(publishTopic, publishMsg))
  {
    Serial.println("Publish Topic:");
    Serial.println(publishTopic);
    Serial.println("Publish message:");
    Serial.println(publishMsg);
  }
  else
  {
    Serial.println("Message Publish Failed.");
  }
}
// 收到信息后的回调函数
void receiveCallback(char *topic, byte *payload, unsigned int length)
{
  cmd = "";
  Serial.print("Message Received [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    cmd += (char)payload[i];
  }
  Serial.println("end ");
  Serial.print(cmd);
  Serial.println(" end");
  if (String(topic) == controlTopic)
  {
    Serial.println("String(topic) == controlTopic");
    if (cmd == sw_on_cmd) // 打开继电器命令
    {
      Serial.println("改为低电平，继电器通路");
      digitalWrite(SwitchGPIO, LOW); // 低电平，继电器则开
    }
    if (cmd == sw_off_cmd) // 关闭继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      digitalWrite(SwitchGPIO, HIGH); // 高电平，继电器则关
    }
    if (cmd == delay_sw_off_cmd) // 延时关闭继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      digitalWrite(SwitchGPIO, LOW); // 低电平，继电器断开
      startCountdown = true;         // 设置为true，表示需要启动倒计时
      startTime = millis();          // 记录开始时间
    }
    if (cmd == delay_sw_on_cmd) // 延时打开继电器命令
    {
      Serial.println("改为高电平，继电器断路");
      //      digitalWrite(SwitchGPIO, HIGH); // 低电平，继电器断开
      startLightCountdown = true; // 设置为true，表示需要启动倒计时
      startLightTime = millis();  // 记录开始时间
    }
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("收到单个查询指令，发送开关状态");
      pubMQTTmsg(); // 查询开关状态
    }
  }
  if (String(topic) == publicControlTopic)
  {
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("收到总查询指令，发送开关状态");
      pubMQTTmsg(); // 查询开关状态
    }
    if (cmd == sw_on_cmd)
    { // 开启继电器
      Serial.println("所有改为低电平，继电器通路");
      digitalWrite(SwitchGPIO, LOW);
    }
    if (cmd == sw_off_cmd)
    { // 关闭继电器

      Serial.println("所有改为高电平，继电器断路");
      digitalWrite(SwitchGPIO, HIGH);
    }
  }
}
// 订阅指定主题
void subscribeTopic()
{
  // 建立订阅独立主题
  // c change s status 改变状态
  String topicString = controlTopic;
  char subTopic[topicString.length() + 1];
  strcpy(subTopic, topicString.c_str());
  // 通过串口监视器输出是否成功订阅独立主题
  if (mqttClient.subscribe(subTopic))
  {
    Serial.print("Successful Subscribe Individual Topic:"), Serial.println(subTopic);
  }
  else
  {
    Serial.println("Individual Subscribe Fail...");
  }

  // 建立订阅总控制主题
  if (mqttClient.subscribe(publicControlTopic.c_str()))
  {
    Serial.print("Successful Subscribe Individual Topic:"), Serial.println(publicControlTopic);
  }
  else
  {
    Serial.println("publicControlTopic Fail...");
  }

  if (mqttClient.subscribe(statusTopic.c_str()))
  {
    Serial.print("Successful Subscribe Individual Topic:"), Serial.println(statusTopic);
  }
  else
  {
    Serial.println("statusTopic Fail...");
  }
}

// ESP8266连接wifi
void connectWifi()
{
  // 设置重连超时时间和尝试次数
  WiFi.setAutoReconnect(true);        // 启用自动重新连接
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // 设置睡眠模式（在连接失败时保持唤醒）
  WiFi.persistent(true);              // 持久化配置（使重启后仍能记住WiFi配置）

  WiFi.begin(ssid, password);
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
  wifiClient.setTrustAnchors(&mqttcert);
  // if no check the CA Certification
  wifiClient.setInsecure();
}