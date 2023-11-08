/*
 * oled_infor
 * OLED显示连网信息
 */
#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
MAX30105 particleSensor;//Max10302地址
void oled_display(const char * str,int a ,int b);

const char* ssid     = "507507";//连接WIFI名(SSID)
const char* password = "507507507";//WIFI密码

uint8_t adress = 0x38; //传感器设备地址 默认7位2进制数
float T; float T_oled;
float RH; 
int data[6]; 

//定义最多多少个client可以连接本server
#define MAX_SRV_CLIENTS 4
//创建server
WiFiServer server(8088);//端口号，随意修改，范围0-65535
//管理clients
WiFiClient serverClients[MAX_SRV_CLIENTS];

//心率变量申明
const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
//心率变量申明
Adafruit_SSD1306 oled(128, 64, &Wire,-1);
int wlan_flag = 0;//do

//舒适度参数
int confortable_num =  0 ;
void setup()
{
  Wire.begin();
  //启动server
  server.begin();
  Serial.begin(115200);
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.setTextColor(WHITE);//开像素点发光
  oled.clearDisplay();//清屏
  wlan_flag=wlan_connect();
  Max10302_init();//初始化心率传感器
   pinMode(D4, INPUT);  // 将按钮引脚设置为输入模式
   attachInterrupt(digitalPinToInterrupt(D4), func1, CHANGE);//设置中断号、响应函数、触发方式
}

void loop() {
  NewClient();
  oled.clearDisplay();//清屏
  T_oled=AHT10_measure();//aht10传感器参数获取
  Max10302_measure();//获取心率传感参数
  oled_display(beatsPerMinute,T_oled);//oled展示函数
  oled.display(); // 开显示
  //信息发送给端口
  for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (serverClients[i] && serverClients[i].connected()) {
      serverClients[i].print(T_oled);
      serverClients[i].print(" ");
      serverClients[i].print(beatsPerMinute);
      serverClients[i].print(" ");
      serverClients[i].print(beatAvg);
      serverClients[i].println();
    }
  }
}


int wlan_connect(){
  //此段函数的目的是为了与WiF完成通信
  oled.setTextSize(1); //设置字体大小  
  oled.setCursor(15, 5);//设置显示位置
  oled.println("WiFi Information");
  oled.setCursor(2, 20);//设置显示位置

  WiFi.begin(ssid,password);//启动网络连接
  int wlan_break=0;//这里设置中断变量，返回网络链接
  while (WiFi.status() != WL_CONNECTED)//检测网络是否连接成功,25s之后若还没连接，则设为连接失败
  {
    delay(500);
    wlan_break++;
    oled.print(".");//设置显示位置
    oled.display(); // 开显示
    if(wlan_break>=25){
      break;
    }
  }
  if(wlan_break>=25){
    return 0; //连接失败
  }
  else{
    return 1;//连接成功
  }
}

void oled_display(float a ,float b){
/*此函数目的是用来驱动oled显示*/
//a表示心率，b表示温度//
  oled.setTextSize(1); //设置字体大小  
  oled.setCursor(2, 0);//设置显示位置
  oled.println("IP address:");
  oled.println(WiFi.localIP());

  oled.setCursor(2, 20);//设置显示位置
  oled.print("HEART RATE:");
  oled.print(a);

  oled.setCursor(2, 30);//设置显示位置
  oled.print("TEMP:");
  oled.print(b);

  oled.setCursor(2, 40);//设置显示位置
  oled.print("TSV:");
  oled.print(confortable_num);
  oled.display(); // 开显示
}

float AHT10_measure()
{
  //传感器参数测量函数，此函数目的主要是测量传感器获取的温度、湿度参数。
    Wire.beginTransmission(0x38);

    Wire.requestFrom(0x38, 6);

    while (Wire.available())
    {
        for (int i = 0; i < 6; i++)
        {
            data[i] = Wire.read();
            // Serial.println(data[i]);
        }

        if (data[0] & 0x08 == 0)
        {
            // Serial.println("进行初始化");
            // Serial.println(Wire.read());
            Wire.write(0xE1);
            Wire.write(0x08);
            Wire.write(0x00);
        }
        else
        {
            //Serial.println("不需要初始化");
            // Serial.println(Wire.read());
            Wire.write(0xAC);
            Wire.write(0x33);
            Wire.write(0x00);
            delayMicroseconds(75);


            if (data[0] & 0x80 == 0)
            {
                //Serial.println("需要等待完成");
                // Serial.println(Wire.read());
            }
            else
            {
                //Serial.println("不需要等待");
                // Serial.println(Wire.read());
            }
        }
    }
    Wire.endTransmission();
    RH = ((data[1] << 12) | (data[2] << 4)) | (data[3] >> 4);
    T = ((data[3] & 0x0f) << 16) | (data[4] << 8) | data[5];
    RH = (RH / pow(2, 20)) * 100;
    T = (T / pow(2, 20)) * 200 - 50;
    return T;
}

void Max10302_init(){
  //心率传感器初始化函数
   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    //Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void Max10302_measure(){
  long irValue = particleSensor.getIR();
  long irValue_kalmanFilter=kalmanFilter(irValue);
  if (checkForBeat(irValue_kalmanFilter) == true) {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }
  Serial.print(" ");
  Serial.print(irValue_kalmanFilter);
  Serial.print(" ");
  Serial.print(irValue);
  Serial.print(" ");
  //Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
   Serial.print(" ");
  //Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  Serial.println();
}


//检测新的client
void NewClient() {
  //检测是否有新的client请求进来
  if (server.hasClient()) {
    for (int i = 0; i < MAX_SRV_CLIENTS; i++) {
      //释放旧无效或者断开的client
      if (!serverClients[i] || !serverClients[i].connected()) {
        if (serverClients[i]) {
          serverClients[i].stop();
        }
        //分配最新的client
        serverClients[i] = server.available();
        //Serial.print("New client: ");
        //Serial.print(i);
        break;
      }
    }
  }
}

// 卡尔曼滤波
long kalmanFilter(long Z)
{
    static float K = 0.5;            // 卡尔曼增益
    static float P = 1;            // 估计误差协方差
    static const float Q = 0.0025; // 过程噪声协方差，值增大，动态响应变快，收敛稳定性变差，  Q控制误差，R控制响应速度
    static const float R = 0.2;    // 测量噪声协方差，传感器产生的噪声，值增大，动态响应变慢，收敛稳定性变好
    static float prevData = 0;
 
    P = P + Q;
    K = P / (P + R);
    Z = prevData + K * (Z - prevData);
    P = (1.0 - K) * P;
    prevData = Z;
    return Z;
}
//中断响应函数
ICACHE_RAM_ATTR void func1()
{
  //按键中断检测函数
   if (digitalRead(D4)==0) {    // 如果按钮按下
   if(confortable_num>=3){confortable_num = -4;}
    confortable_num++;                   // 计数器加1
    Serial.print("Button pressed. Count: ");
    Serial.println(confortable_num);
    delay(500);                // 等待200毫秒，避免连续多次计数
  }
}
