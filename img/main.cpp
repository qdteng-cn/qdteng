// add ESPUI webpage
#include <Arduino.h>
#include <DNSServer.h>
#include <WiFi.h>
#include <ESPUI.h>

#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <mbedtls/md.h>
#include <tuya_cacert.h>

#define OUTPUT USBSerial

const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);
DNSServer dnsServer;

const char *ssid = "Honor-zhao";
const char *pass = "**********";
const char *hostname = "espui";

float tempC01, tempC02;

int statusLabelId;
int tempGra01;
int tempGra02;
int tempLab01;
int tempLab02;


// put your Lewei API key here,find it in lewei50.com->my account->account setting
#define LW_USERKEY "3cbf19a2d60441e7aaa0793cd98c2556"
#define LW_SERVER "www.lewei50.com"
// put your gateway number here,01 as default
#define LW_GATEWAY "01"

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 10
#define TEMPERATURE_PRECISION 9

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature tempSensors(&oneWire);
// arrays to hold device addresses
DeviceAddress addSensor0, addSensor1;
const char deviceSecret[32] = "kAhWFAawoJOrHyJ7";
const char deviceId[32] = "267b3744a33b88f4d6qa4o";
// MQTT Broker settings
const char *tuyaBroker = "m1.tuyacn.com";
const int tuyaPort = 8883;

// WiFi and MQTT client for Tuya IoT Link
WiFiClientSecure espSecureClient;
PubSubClient tuyaClient(espSecureClient);

// WiFi and MQTT client for local NAS and HomeAssistant
WiFiClient mqttClient;
PubSubClient nasClient(mqttClient);

#define HMAC_PASS_LEN 32
#define NTP_SERVER "ntp1.aliyun.com"

String glb_SSID = "Honor-zhao";
String glb_PSWD = "zhouQian2";

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // zero pad the address if necessary
        if (deviceAddress[i] < 16)
            OUTPUT.print("0");
        OUTPUT.print(deviceAddress[i], HEX);
    }
}

// function to print the temperature for a device
float printTemperature(DeviceAddress deviceAddress)
{
    float tempC = tempSensors.getTempC(deviceAddress);
    OUTPUT.print("Temp: ");
    OUTPUT.print(tempC);
    return tempC;
}

// function to print a device's resolution
void printResolution(DeviceAddress deviceAddress)
{
    OUTPUT.print("Res: ");
    OUTPUT.print(tempSensors.getResolution(deviceAddress));
    OUTPUT.println();
}

// main function to print information about a device
float printData(DeviceAddress deviceAddress)
{
    OUTPUT.print("Device: ");
    printAddress(deviceAddress);
    OUTPUT.print(" ");
    float tempC = printTemperature(deviceAddress);
    OUTPUT.println();
    return tempC;
}

String msgAppend(const char *name, float temp)
{
    String jsonData = "{\'Name\':\'" + String(name) + "\',\'Value\':\'" + String(temp) + "\'}";
    return jsonData;
}

// this method makes a HTTP connection to the server:
int leweiDataUpload(float Tin, float Tout)
{
    int iResult = 0;
    WiFiClient lwClient;
    // if there's a successful connection:
    if (!lwClient.connect(LW_SERVER, 80))
    {
        lwClient.stop();
        return iResult;
    }

    String jsonMsg = "[";
    jsonMsg += msgAppend("Tin", Tin);
    jsonMsg += ",";
    jsonMsg += msgAppend("Tout", Tout);
    jsonMsg += "]";

    // send the HTTP POST request:
    String cmd;
    cmd = "POST /api/V1/Gateway/UpdateSensors/";
    cmd += LW_GATEWAY;
    cmd += " HTTP/1.1\r\n";
    cmd += "Host: www.lewei50.com\r\n";
    cmd += "userkey: ";
    cmd += LW_USERKEY;
    cmd += "\r\n";
    cmd += "Content-Length: ";
    cmd += String(jsonMsg.length());
    cmd += "\r\n";
    cmd += "Connection: close\r\n";
    cmd += "\r\n";
    cmd += jsonMsg;

    lwClient.print(cmd);
    OUTPUT.println(cmd);

    String respMsg = lwClient.readStringUntil('}');
    OUTPUT.println(respMsg);

    int idx = respMsg.indexOf("Message");
    if (idx > 0)
    {
        if (respMsg.indexOf("Successful", idx) > 0)
        {
            OUTPUT.println("Upload data is successful!");
            iResult = 0;
        }
        else
        {
            OUTPUT.println("Upload data is failed!");
            iResult = 1;
        }
    }
    else
    {
        OUTPUT.println("Didn't find Message from response!");
        iResult = 0;
    }

    lwClient.stop();
    return iResult;
}

// 获取当前时间
bool updateLocalTime()
{
    tm timeinfo;
    for (int i = 0; i < 10; i++)
    {
        if (!getLocalTime(&timeinfo))
        {
            OUTPUT.println("failed to obtain time.");
            delay(2000);
        }
        else
        {
            OUTPUT.println(&timeinfo, "%F %T %A");
            return true;
        }
    }
    return false;
}

// 获取当前时间戳
bool updateEpochTime(time_t &curTime)
{
    for (int i = 0; i < 10; i++)
    {
        time(&curTime);
        if (curTime > 1712927540)
        {
            OUTPUT.print("time: ");
            OUTPUT.println(curTime);
            return true;
        }

        OUTPUT.println("failed to obtain epochtime.");
        delay(2000);
    }

    return false;
}

bool connectWifi(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(glb_SSID.c_str(), glb_PSWD.c_str()); // 连接上一次连接成功的wifi
    OUTPUT.print(F("connecting to wifi: "));
    OUTPUT.println(glb_SSID);

    int count = 0;
    while (WiFi.status() != WL_CONNECTED)
    {
        count++;
        delay(500);
        OUTPUT.print(".");
        if (count > 20)
        {
            // 如果10秒内没有连上，就开启Web配网 可适当调整这个时间
            OUTPUT.println(F("\nconnect failed!"));
            return false; // 失败退出，防止无限初始化
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        // 如果连接上，就输出IP信息 防止未连接上break后会误输出
        OUTPUT.print(F("\nconnected: "));
        OUTPUT.println(WiFi.localIP()); // 打印esp8266的IP地址
    }

    dnsServer.start(DNS_PORT, "*", WiFi.localIP());
    return true;
}

bool hmacSha256(const char *key, const char *payload, unsigned char *digest)
{
    if ((NULL == digest) || (NULL == key) || (NULL == payload))
    {
        OUTPUT.println("invalide pointer.");
        return false;
    }

    mbedtls_md_context_t ctx;
    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;

    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 1);
    mbedtls_md_hmac_starts(&ctx, (const unsigned char *)key, strlen(key));
    mbedtls_md_hmac_update(&ctx, (const unsigned char *)payload, strlen(payload));
    mbedtls_md_hmac_finish(&ctx, digest);
    mbedtls_md_free(&ctx);

    return true;
}

bool tuyaMQTTPassCal(const char *deviceId, const char *deviceSecret, char *clientId, char *user, char *pass)
{
    if (NULL == deviceId || NULL == deviceSecret || NULL == clientId || NULL == user || NULL == pass)
    {
        OUTPUT.println("invalide pointer.");
        return false;
    }

    time_t curTime;
    if (!updateEpochTime(curTime))
        return false;

    /* client ID */
    snprintf(clientId, 32, "tuyalink_%s", deviceId);
    snprintf(user, 128, "%s|signMethod=hmacSha256,timestamp=%d,secureMode=1,accessType=1", deviceId, curTime);

    /* password */
    char passward_stuff[255] = "";
    uint8_t digest[HMAC_PASS_LEN];
    size_t slen = snprintf(passward_stuff, 255, "deviceId=%s,timestamp=%d,secureMode=1,accessType=1", deviceId, curTime);
    hmacSha256(deviceSecret, passward_stuff, digest);
    for (int i = 0; i < HMAC_PASS_LEN; i++)
    {
        sprintf(pass + 2 * i, "%02x", digest[i]);
    }

    return true;
}

void tuyaCallback(char *topic, byte *payload, unsigned int length)
{
    OUTPUT.print("Message arrived in topic: ");
    OUTPUT.println(topic);
    OUTPUT.print("Message:");
    for (int i = 0; i < length; i++)
    {
        OUTPUT.print((char)payload[i]);
    }
    OUTPUT.println();
    OUTPUT.println("-----------------------");
}

bool tuyaMQTTConnect()
{
    char clientId[32] = "";
    char szUser[128] = "";
    char szPass[66] = "";
    if (!tuyaMQTTPassCal(deviceId, deviceSecret, clientId, szUser, szPass))
        return false;

    bool bRet = tuyaClient.connected();
    while (!bRet)
    {
        OUTPUT.printf("connecting to MQTT as %s\nuser: %s, pass: %s ...\n", clientId, szUser, szPass);
        if (tuyaClient.connect(clientId, szUser, szPass))
        {
            OUTPUT.println("connected to tuya MQTT broker");
            tuyaClient.subscribe("tylink/267b3744a33b88f4d6qa4o/thing/property/set");
            return true;
        }
        else
        {
            OUTPUT.print("failed to connect to MQTT broker, rc=");
            OUTPUT.print(tuyaClient.state());
            OUTPUT.println(" retrying in 5 seconds.");
            delay(5000);
        }
    }

    return bRet;
}

bool nashaMQTTConnect()
{
    bool bRet = nasClient.connected();
    while (!bRet)
    {
        OUTPUT.println("connecting to NAS Local MQTT");
        nasClient.setServer("192.168.3.100",1883);
        if (nasClient.connect("esp32_xiao_abdf"))
        {
            OUTPUT.println("connected to NAS Local MQTT broker");
            return true;
        }
        else
        {
            OUTPUT.print("failed to connect to NAS Local MQTT broker, rc=");
            OUTPUT.print(nasClient.state());
            OUTPUT.println(" retrying in 5 seconds.");
            delay(5000);
        }
    }

    return bRet;
}

void setup(void)
{
    OUTPUT.begin(115200);
    randomSeed(analogRead(0));

    connectWifi();

    ESPUI.setVerbosity(Verbosity::VerboseJSON);
    statusLabelId = ESPUI.label("Status:", ControlColor::Turquoise, "Init...");

    tempLab01 = ESPUI.label("Temp In:", ControlColor::Emerald, "0");
    tempLab02 = ESPUI.label("Temp Ex:", ControlColor::Emerald, "0");

    tempGra01 = ESPUI.graph("Temp In", ControlColor::Wetasphalt);
    tempGra02 = ESPUI.graph("Temp Ex", ControlColor::Wetasphalt);

    ESPUI.begin("ESPUI Control");

    // Start up the library
    tempSensors.begin();

    // locate devices on the bus
    OUTPUT.println("locating devices...");
    OUTPUT.print(tempSensors.getDeviceCount(), DEC);
    OUTPUT.println(" devices founded.");

    // report parasite power requirements
    OUTPUT.print("parasite power is: ");
    if (tempSensors.isParasitePowerMode())
        OUTPUT.println("ON");
    else
        OUTPUT.println("OFF");

    // Search for devices on the bus and assign based on an index. Ideally,
    // you would do this to initially discover addresses on the bus and then
    // use those addresses and manually assign them (see above) once you know
    // the devices on your bus (and assuming they don't change).
    //
    // method 1: by index
    if (!tempSensors.getAddress(addSensor0, 0))
        OUTPUT.println("unable to find address for Device 0");
    if (!tempSensors.getAddress(addSensor1, 1))
        OUTPUT.println("unable to find address for Device 1");

    // show the addresses we found on the bus
    OUTPUT.print("Device0: ");
    printAddress(addSensor0);
    OUTPUT.println();

    OUTPUT.print("Device1: ");
    printAddress(addSensor1);
    OUTPUT.println();

    // set the resolution to 9 bit per device
    tempSensors.setResolution(addSensor0, TEMPERATURE_PRECISION);
    tempSensors.setResolution(addSensor1, TEMPERATURE_PRECISION);

    OUTPUT.print("Device0 Res: ");
    OUTPUT.print(tempSensors.getResolution(addSensor0), DEC);
    OUTPUT.println();

    OUTPUT.print("Device1 Res: ");
    OUTPUT.print(tempSensors.getResolution(addSensor1), DEC);
    OUTPUT.println();

    tempC01 = 99.99;
    tempC02 = 99.99;

    const long gmtOffset_sec = 8 * 3600;
    const int daylightOffset_sec = 0;
    configTime(gmtOffset_sec, daylightOffset_sec, NTP_SERVER);

    ESPUI.print(statusLabelId, F("Updating local time..."));    
    delay(2000);
    if (!updateLocalTime())
    {
        OUTPUT.println("ESP.restart()...");
        ESP.restart();
    }

    espSecureClient.setCACert(tuya_cacert_pem);
    tuyaClient.setServer(tuyaBroker, tuyaPort);
    tuyaClient.setKeepAlive(60);
    tuyaClient.setCallback(tuyaCallback);

    tuyaMQTTConnect();

    OUTPUT.println("setup done!");    
    ESPUI.print(statusLabelId, F("Setup done!"));    

}

bool tuyaMQTTDataPublish(const char *deviceId, float temp_indoor, float temp_outdoor)
{
    char topic[64];
    snprintf(topic, 64, "tylink/%s/thing/property/report", deviceId);

    char msgId[16];
    snprintf(msgId, 16, "%08x", random(0x7FFFFFFF));
    OUTPUT.println(msgId);
    time_t curTime;
    if (!updateEpochTime(curTime))
    {
        OUTPUT.println("updateEpochTime failed");
        return false;
    }

    // 封装json
    DynamicJsonDocument doc(512);
    DynamicJsonDocument jsdata(256);
    DynamicJsonDocument tempIndoor(32);
    DynamicJsonDocument tempOutdoor(32);

    tempIndoor["value"] = (int)temp_indoor;
    tempIndoor["time"] = curTime;
    tempOutdoor["value"] = (int)temp_outdoor;
    tempOutdoor["time"] = curTime;

    jsdata["temp_current"] = tempIndoor;
    jsdata["temp_outdoor"] = tempOutdoor;

    doc["msgId"] = msgId;
    doc["time"] = curTime;
    doc["data"] = jsdata;

    String str;
    serializeJson(doc, str);
    OUTPUT.println(str);

    // Sending to MQTT
    char *p = (char *)str.c_str();
    if (tuyaClient.publish(topic, p))
    {
        OUTPUT.println("date updated.");
        return true;
    }
    else
        OUTPUT.println("date update failed.");

    return false;
}

bool nasMQTTDataPublish(const char *deviceId, float temp_indoor, float temp_outdoor)
{
    char payload[16];
    snprintf(payload,16,"%.2f",temp_indoor);

    // Sending to MQTT
    if (nasClient.publish("home/livingroom/temperature", payload))
    {
        OUTPUT.println("date updated.");
    }
    else
        OUTPUT.println("date update failed.");


    // Sending to MQTT
    snprintf(payload,16,"%.2f",temp_outdoor);
    if (nasClient.publish("home/outside/temperature", payload))
    {
        OUTPUT.println("date updated.");
        return true;
    }
    else
        OUTPUT.println("date update failed.");


    return false;
}

uint32_t giCount;
void loop(void)
{
    dnsServer.processNextRequest();

    delay(100);

    if ((giCount % 50) == 0)
    {
        ESPUI.print(statusLabelId, F("Requesting temp..."));    

        OUTPUT.print("Requesting temp...");
        tempSensors.requestTemperatures();
        OUTPUT.println("DONE");

        tempC01 = printData(addSensor0); // outdoor
        tempC02 = printData(addSensor1); // indoor

        ESPUI.print(tempLab01, String(tempC01));
        ESPUI.print(tempLab02, String(tempC02));
        ESPUI.addGraphPoint(tempGra01, (int)tempC01);
        ESPUI.addGraphPoint(tempGra02, (int)tempC02);
    }

    // upload sensor data to lewei and tuya IoT every 30 seconds
    if (giCount >= 600)
    {
        ESPUI.print(statusLabelId, F("Uploading..."));    

        // 1. lewei 
        leweiDataUpload(tempC02, tempC01);

        // 2. tuya IoT
        if (!tuyaClient.connected())
        {
            tuyaMQTTConnect();
        }

        tuyaMQTTDataPublish(deviceId, tempC02, tempC01);
        tuyaClient.loop();
        // tuya IoT end ... 

        // 3. NAS MQTT and HomeAssistant
        if (!nasClient.connected())
        {
            nashaMQTTConnect();
        }

        nasMQTTDataPublish("abdf", tempC02, tempC01);
        nasClient.loop();        
        // NAS MQTT end ... 
        giCount = 0;
    }
    else
        giCount++;

}
