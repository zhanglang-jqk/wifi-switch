/**********************************************************************
  ��Ŀ����/Project          : ��·���ƿ���
  ��������/Program name     : Control The Switch
  �Ŷ�/Team                : YGKJ
  ����/Author              : YGKJ
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
const int SwitchGPIO = 16; // �̵�����ӦGPIO��
const int brokerPort = 8883;
// ��Ҫ�޸ĵ���Ϣ
//  MQTT����������û�������
const char *mqttUserName = "dl2binary";
const char *mqttPassword = "wocaonima88jqk";
// const char *ssid = "303";
// const char *password = "VT4009030242";
const char *ssid = "CMCC-CL4X";
const char *password = "ZQYJZ926294";

String deviceNo; // �豸���,��mac��ַ��ͬ(����ð��)
String clientId; // �ͻ���ID,��mac��ַ��ͬ(����ð��)

String publicControlTopic = "public_control"; // ������������,��·����,����Ҫͬʱ���ƶ����բʱʹ��
// String controlTopic = deviceNo + "/" + "contrl"; // ��ǰ�̵�����������
// String statusTopic = deviceNo + "/" + "status";  // ��ǰ�̵���״̬��������
String controlTopic; // ��ǰ�̵�����������,������Ҫ�޸�,���Բ��ڴ˴���ʼ��
String statusTopic;  // ��ǰ�̵���״̬��������,������Ҫ�޸�,���Բ��ڴ˴���ʼ��
// ��Ҫ�޸ĵ���Ϣ
String cmd;
String sw_on_cmd = "sw_on_cmd";                 // �򿪵�բ
String sw_off_cmd = "sw_off_cmd";               // �رյ�բ
String delay_sw_on_cmd = "delay_sw_on_cmd";     // ��ʱ�򿪵�բ
String delay_sw_off_cmd = "delay_sw_off_cmd";   // ��ʱ�رյ�բ
String get_sw_status_cmd = "get_sw_status_cmd"; // ��ѯ��բ״̬

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

int count; // Ticker�����ñ���
// ����ʱ
uint32_t startTime = 0;             // ����һ���������洢��ʼʱ�� ��������
uint32_t delayTime = 5 * 60 * 1000; // 5���ӵ��ӳ�ʱ�䣨�Ժ���Ϊ��λ��
bool startCountdown = false;        // �����������ڸ����Ƿ���Ҫ��������ʱ
bool startLightCountdown = false;   // ����Ϊtrue����ʾ��Ҫ��������ʱ
uint32_t startLightTime = 0;        // ����һ���������洢��ʼʱ�� ����ص�

bool en_sigled_toggle = true; // �����źŵ���˸
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

  // ����ESP8266����ģʽΪ�����ն�ģʽ
  WiFi.mode(WIFI_STA);

  digitalWrite(SwitchGPIO, HIGH);
  digitalWrite(LED_PWR, ON_LED_PWR), digitalWrite(LED_SIGNAL, ON_LED_SIGNAL);
  led_ticker.attach(0.5, // ����������,LED��˸��ʾ
                    []()
                    {
                      if (en_sigled_toggle)
                      {
                        digitalWrite(LED_SIGNAL, !digitalRead(LED_SIGNAL));
                      }
                    });
  // ����WiFi
  connectWifi();

  deviceNo = WiFi.macAddress();
  deviceNo.replace(":", "");
  clientId = deviceNo;
  controlTopic = deviceNo + "/" + "contrl"; // ��ǰ�̵�����������
  statusTopic = deviceNo + "/" + "status";  // ��ǰ�̵���״̬��������

  // STEP4: Set MQTT Parameters
  mqttClient.setClient(wifiClient);
  // ����MQTT�������Ͷ˿ں�
  mqttClient.setServer(mqttServer, brokerPort);
  mqttClient.setCallback(receiveCallback);

  // ����MQTT������
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

  // Ticker��ʱ����
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
  // �����Ҫ��������ʱ���Ѿ�����5����
  if (startCountdown && millis() - startTime >= delayTime)
  {
    digitalWrite(SwitchGPIO, HIGH); // 5���Ӻ󴥷��Ĳ���
    startCountdown = false;         // ���ñ�־
    startTime = 0;                  // ����ʱ����
  }
  if (startLightCountdown && millis() - startLightTime >= delayTime)
  {
    digitalWrite(SwitchGPIO, LOW); // 5���Ӻ󴥷��Ĳ���
    startLightCountdown = false;   // ���ñ�־
    startLightTime = 0;            // ����ʱ����
  }
  if (mqttClient.connected())
  { // ���������ɹ����ӷ�����
    // Serial.println(millis());
    // ÿ��60���ӷ���һ����Ϣ
    if (count >= 3600)
    {
      Serial.println("public mqtt message");
      pubMQTTmsg(); // ��ѯ����״̬
      count = 0;
    }
    // ��������
    mqttClient.loop();
  }
  else
  {                      // ���������δ�ܳɹ����ӷ�����
    connectMQTTServer(); // �������ӷ�����
  }
}

void tickerCount()
{
  count++;
}

/**
 * @brief
 *
 * @return uint8_t 1:�ɹ����ӷ����� 0:δ�ܳɹ����ӷ�����

 */
uint8_t connectMQTTServer()
{
  // ����ESP8266��MAC��ַ���ɿͻ���ID������������ESP8266�Ŀͻ���ID������
  // ����MQTT������
  if (mqttClient.connect(clientId.c_str(), mqttUserName, mqttPassword))
  {
    Serial.println("MQTT Server Connected Successful.");
    // ��������
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

// �����̵���״̬������Ϣ
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
  // ����������Ϣ��
  messageString += ",\"Switch1Status\":" + String(digitalRead(SwitchGPIO)) + "}";
  // String messageString = "{\"clientId: " + String(clientId) + ",Switch1Status: " + String(GPIOStatus1) + ",Switch2Status: " + String(GPIOStatus2) + "}";

  char publishMsg[messageString.length() + 1];
  strcpy(publishMsg, messageString.c_str());

  // ʵ��ESP8266�����ⷢ����Ϣ
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
// �յ���Ϣ��Ļص�����
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
    if (cmd == sw_on_cmd) // �򿪼̵�������
    {
      Serial.println("��Ϊ�͵�ƽ���̵���ͨ·");
      digitalWrite(SwitchGPIO, LOW); // �͵�ƽ���̵�����
    }
    if (cmd == sw_off_cmd) // �رռ̵�������
    {
      Serial.println("��Ϊ�ߵ�ƽ���̵�����·");
      digitalWrite(SwitchGPIO, HIGH); // �ߵ�ƽ���̵������
    }
    if (cmd == delay_sw_off_cmd) // ��ʱ�رռ̵�������
    {
      Serial.println("��Ϊ�ߵ�ƽ���̵�����·");
      digitalWrite(SwitchGPIO, LOW); // �͵�ƽ���̵����Ͽ�
      startCountdown = true;         // ����Ϊtrue����ʾ��Ҫ��������ʱ
      startTime = millis();          // ��¼��ʼʱ��
    }
    if (cmd == delay_sw_on_cmd) // ��ʱ�򿪼̵�������
    {
      Serial.println("��Ϊ�ߵ�ƽ���̵�����·");
      //      digitalWrite(SwitchGPIO, HIGH); // �͵�ƽ���̵����Ͽ�
      startLightCountdown = true; // ����Ϊtrue����ʾ��Ҫ��������ʱ
      startLightTime = millis();  // ��¼��ʼʱ��
    }
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("�յ�������ѯָ����Ϳ���״̬");
      pubMQTTmsg(); // ��ѯ����״̬
    }
  }
  if (String(topic) == publicControlTopic)
  {
    if (cmd == get_sw_status_cmd)
    {
      Serial.println("�յ��ܲ�ѯָ����Ϳ���״̬");
      pubMQTTmsg(); // ��ѯ����״̬
    }
    if (cmd == sw_on_cmd)
    { // �����̵���
      Serial.println("���и�Ϊ�͵�ƽ���̵���ͨ·");
      digitalWrite(SwitchGPIO, LOW);
    }
    if (cmd == sw_off_cmd)
    { // �رռ̵���

      Serial.println("���и�Ϊ�ߵ�ƽ���̵�����·");
      digitalWrite(SwitchGPIO, HIGH);
    }
  }
}
// ����ָ������
void subscribeTopic()
{
  // �������Ķ�������
  // c change s status �ı�״̬
  String topicString = controlTopic;
  char subTopic[topicString.length() + 1];
  strcpy(subTopic, topicString.c_str());
  // ͨ�����ڼ���������Ƿ�ɹ����Ķ�������
  if (mqttClient.subscribe(subTopic))
  {
    Serial.print("Successful Subscribe Individual Topic:"), Serial.println(subTopic);
  }
  else
  {
    Serial.println("Individual Subscribe Fail...");
  }

  // ���������ܿ�������
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

// ESP8266����wifi
void connectWifi()
{
  // ����������ʱʱ��ͳ��Դ���
  WiFi.setAutoReconnect(true);        // �����Զ���������
  WiFi.setSleepMode(WIFI_NONE_SLEEP); // ����˯��ģʽ��������ʧ��ʱ���ֻ��ѣ�
  WiFi.persistent(true);              // �־û����ã�ʹ���������ܼ�סWiFi���ã�

  WiFi.begin(ssid, password);
  // �ȴ�WiFi����,�ɹ����Ӻ�����ɹ���Ϣ
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
  }
  Serial.println("");
  Serial.print("Connected Successful to WiFi");
  Serial.println(WiFi.SSID()); // WiFi����
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP()); // IP
  // STEP3: Set CA Certification for TLS
  Serial.print("MAC address:\t");
  Serial.println(WiFi.macAddress());
  wifiClient.setTrustAnchors(&mqttcert);
  // if no check the CA Certification
  wifiClient.setInsecure();
}