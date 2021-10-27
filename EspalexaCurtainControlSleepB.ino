#include <Espalexa.h>
#include <time.h>
#include <TimeLib.h>
#include <WiFiManager.h>
#include <esp_sntp.h>
#include <Ticker.h>

#define START1  5   //朝の開始時刻 ( 5 は 午前5時を示す）
#define END1    7   //朝の終了時刻
#define START2  17  //夕の開始時刻
#define END2    18  //夕の終了時刻

#define JST     3600*9
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */

//GPIO numbers
#define LED       16
#define PWM_OUT   15
#define CLS_SW    12 // Left switch (Normally closed contact)
#define OPN_SW     5 // Right switch (Normally closed contact)

const int deg000 = 500; // Servomotor control signal pulse width minimum [microSec]
const int deg180 = 2400; // Servomotor control signal pulse width maximum [microSec]
const int OPN_SPEED = 33; //right rotational speed set (+100 is max speed to right)
const int CLS_SPEED = -22;  //left rotational speed set (-100 is max speed to left)
const int Z_SPEED = 0;  // zero speed set to stop

// variables
int microSec;
int speedset_OPN, speedset_CLS, speedset_Z, motor_running_count; //***Added 2021.10.27
boolean light_on = 0;
boolean  flag_close_cmd = 0;
boolean  flag_open_cmd = 0;
boolean  flag_interrupt = false; //****Modified 2021.10.9

RTC_DATA_ATTR int bootCount = 0;  //このRTCメモリの変数はスリープ状態からのリブート時にはクリアされない。スリープした回数を数える。
RTC_DATA_ATTR boolean INIT = true;



// prototypes
void flushLED();
void pwm();
void control_stop();
void control_open();
void control_close();
void IRAM_ATTR closeSwitchOn();
void IRAM_ATTR openSwitchOn();

Espalexa espalexa;
Ticker serv1;
Ticker light;
Ticker checklight;

//------------------------------------------------------------------------------------------------
// setup()
//------------------------------------------------------------------------------------------------
void setup(){
  boolean flag_loop_init = true;
  boolean flag_initialTimeSynchronized;
  boolean flag_mainTask_executed = false;
    
  Serial.begin(115200);
  delay(1000); //Take some time to open up the Serial Monitor
  
  pinMode(LED,OUTPUT);

  // GPIO割り込み発生時の処理 //****Modified 2021.10.9
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if(wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("******* EXT0 INTERRUPT ******");
    flag_interrupt = true;
    bootCount=20;
  }
  

  //**イニシャル起動時にまず時刻同期
  if(INIT) {
    timesynch();
    flag_initialTimeSynchronized = INIT;
    INIT = false;
    goto timecheck;//初回はスリープサイクルをスキップさせ、動作設定時刻かどうかをチェックさせる。
  }


  //**スリープサイクル(30秒スリープを20回繰り返すと10分になる)******
  if(bootCount>=20) {//20回スリープしたので時刻を確認する。
    struct tm localTime;
    getLocalTime(&localTime);
    setTime(mktime(&localTime)+JST);
    Serial.printf("%02d:%02d:%02d\n", hour(), minute(), second());
    
timecheck:
    while((START1<=hour())^(hour()<END1)^(START1<END1) || (START2<=hour())^(hour()<END2)^(START2<END2) || flag_interrupt) {
      //--------メインタスクの実行--------------------------------------   
      if(flag_loop_init) {//メインタスクの初期設定
                
        if(!flag_initialTimeSynchronized){//イニシャル起動時はすでに同期しているので時刻同期をスキップする。
          timesynch();
        }
  
        Serial.println("**** Wake Up. Start main task !!!! *****");
        
        //メインタスクが実行中であることがわかるように5秒毎にLEDを光らせる周期タスクを開始
        checklight.attach_ms(5000, checkLED);//

        //サーボモータとリミットスイッチのI/O初期化処理
        pinMode(PWM_OUT,OUTPUT);
        pinMode(CLS_SW,INPUT_PULLUP);
        attachInterrupt(CLS_SW, closeSwitchOn, RISING); //GPIO割り込みで停止処置を呼ぶ設定
        pinMode(OPN_SW,INPUT_PULLUP);
        attachInterrupt(OPN_SW, openSwitchOn, RISING); //GPIO割り込みで停止処置を呼ぶ設定

        //サーボモータの回転速度設定から指令パルス幅(μsec)への変換
        speedset_OPN = map(OPN_SPEED,-100,100,deg000,deg180);
        speedset_CLS = map(CLS_SPEED,-100,100,deg000,deg180);
        speedset_Z = map(Z_SPEED,-100,100,deg000,deg180); 
      
        //espAlexaの利用開始処理
        espalexa.addDevice("Curtain", firstLightChanged); //simplest definition, default state off
        espalexa.begin();
              
        flag_loop_init = false;//初期設定が完了したのでフラグをオフ。
      }//メインタスクの初期設定ここまで
      
      //--------メインタスクのループ実行--------------------------------------
      for(int i=0;i<10;i++){//10回で10分間繰り返すループ
        Serial.printf("espalexa is running... %02d:%02d:%02d\n", hour(), minute(), second());
        for(int j=0;j<120;j++){//60秒間繰り返すループ
          espalexa.loop();
              delay(500);
        }
      }
      flag_mainTask_executed = true;
      flag_interrupt = false; //****Modified 2021.10.9
      
    }//End of While loop

    //メインタスクの終了処理
    if(flag_mainTask_executed) {
      checklight.detach();
      WiFi.disconnect(true);//スリープ移行前にWiFi接続を切る。
    }
       
    
    bootCount = 0;

  }//End of if:スリープサイクル
  bootCount++;


  //------------------------------------------------------------------------------------------------
  //スリープ移行処理
  //------------------------------------------------------------------------------------------------
  Serial.println("!!!!!! Sleep count: " + String(bootCount));
  
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_0,0); //****Modified 2021.10.9
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
  Serial.println("Going to sleep now");
  Serial.flush(); 
  esp_deep_sleep_start();// *************ここでスリープ開始*************************************
  Serial.println("This will never be printed");
}


void loop(){
  //This is not going to be called
}




void checkLED(){  // LEDをフラッシュさせる。(delayを使わずにtiker実行により点滅タイミングを作る)
    digitalWrite(LED, HIGH);
    delay(10);
    digitalWrite(LED, LOW);
}

void flushLED(){  // LEDを点滅させる。(delayを使わずにtiker実行により点滅タイミングを作る)
  //***Modified on 2021.10.27
  light_on = !light_on;
  digitalWrite(LED, light_on);

  //***Added on 2021.10.27
  if(motor_running_count++ > 58) { // 500msec x 58 = 29sec.をカウントしたら停止する（リミットスイッチが検出できなかった時のバックアップ停止）
    motor_running_count = 0;
    Serial.print("Time count up and stopped.");
    control_stop();
  }
}


//------------------------------------------------------------------------------------------------
//WiFiに接続してntpサーバと時刻同期
//------------------------------------------------------------------------------------------------
void timesynch() {
  WiFiManager wm;   // added for WiFi Manager
  if (!wm.autoConnect("ESP32")) {     // added for WiFi Manager
    Serial.println("Cannot connect to WiFi. Please check data and reset the ESP.");
    delay(2500);
    ESP.restart();
    delay(5000);
  } 
  else {
    Serial.println("Connected to WiFi.");
  }
  
  struct tm localTime;
  configTzTime("JST-9", "ntp.nict.jp", "time.google.com", "ntp.jst.mfeed.ad.jp");
  Serial.println("[NTP Svr] Connecting.");
  while(sntp_get_sync_status()==SNTP_SYNC_STATUS_RESET){
    delay(1000);
  }
  getLocalTime(&localTime);
  //Serial.println(&localTime, "%H:%M:%S");
  setTime(mktime(&localTime)+JST);
  Serial.printf("%02d:%02d:%02d\n", hour(), minute(), second());
}

void IRAM_ATTR closeSwitchOn() { //閉側スイッチ動作による割り込み発生時に呼ばれる停止処置
  if(flag_close_cmd) {
    control_stop();
    flag_close_cmd = 0;
    Serial.println("Interrupted and stopped closing");
  }
}

void IRAM_ATTR openSwitchOn() { //開側スイッチ動作による割り込み発生時に呼ばれる停止処置
  if(flag_open_cmd) {
    control_stop();
    flag_open_cmd = 0;
    Serial.println("Interrupted and stopped Opening");
  }
}

void pwm(){  // サーボモーターへの制御パルス出力
  digitalWrite(PWM_OUT, HIGH);
  delayMicroseconds(microSec);
  digitalWrite(PWM_OUT, LOW);
}

//------------------------------------------------------------------------------------------------
//alexaから飛んでくる指示の処理。送られてきたbrightness値を解析し、結果に応じてカーテンの操作を行う関数を呼ぶ
//------------------------------------------------------------------------------------------------
void firstLightChanged(uint8_t brightness) {
    Serial.print("Device 1 changed to ");
    
    //do what you need to do here

    if (brightness == 255) {   // Turned ON to CLOSE
      Serial.print("ON(CLOSE): ");
      Serial.println(brightness);
      control_close();
    }
    else if (brightness == 0) {   // Turned OFF to OPEN
      Serial.print("OFF(OPEN): ");
      Serial.println(brightness);
      control_open();
    }
    else {
      Serial.print("STOP: ");
      Serial.println(brightness);
      control_stop();
    }
}

//------------------------------------------------------------------------------------------------
//開・閉・停止操作を実行する関数
//------------------------------------------------------------------------------------------------
void control_open(){
   flag_open_cmd = 1;
  microSec = speedset_OPN; // OPEN側へ動かす
  serv1.attach_ms(20, pwm); 
  light.attach_ms(500, flushLED); // 500msec周期でflushLED()を実行
}

void control_close() {
   flag_close_cmd = 1;
  microSec = speedset_CLS; // CLOSE側へ動かす
  serv1.attach_ms(20, pwm); 
  light.attach_ms(500, flushLED); // 500msec周期でflushLED()を実行
}

void control_stop(){  // 停止させる
  microSec = speedset_Z;
  serv1.detach();
  light.detach();
  digitalWrite(LED,LOW);
}
