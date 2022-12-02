//=====================================================LIBRARY=======================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h> 
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"


//======================================================DONE=========================================================================


//==================================================INITIALIZATION=======================================================================
// =======Initialize Wifi =========
const char* ssid = "Sitibudi"; 
const char* password = "luncuran123";

//const char* ssid = "Bro-Bor"; 
//const char* password = "9434276267";

// ========Initialize Telegram BOT==============
String BOTtoken = "5664311941:AAFTyRdL17bsLJ9946V0OVJa4jz1hchR7eE";  
String CHAT_ID = "925595481";
//#define BOT_TOKEN "5664311941:AAFTyRdL17bsLJ9946V0OVJa4jz1hchR7eE"
//Your own ID is: 925595481
bool sendPhoto = false;

//Checks for new messages every 1 second.
int botRequestDelay = 1000;
unsigned long lastTimeBotRan;

WiFiClientSecure clientTCP;
UniversalTelegramBot bot(BOTtoken, clientTCP);

// =======Initialize Servo============
Servo myservo;
int servoPin = 12;

// =======Initialize PIR Sensor============
int PIRstate = LOW; // we start, assuming no motion detected
int val = 0;

// the time we give the sensor to calibrate (approx. 10-60 secs according to datatsheet)
const int calibrationTime = 30; // 30 secs


// =======Initialize Camera============
#define FLASH_LED_PIN 4
bool flashState = LOW;



//======================================================DONE=========================================================================

//====================================================FUNCTION=======================================================================

// ========Camera Setup===========
void configInitCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;  //0-63 lower number means higher quality
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;  //0-63 lower number means higher quality
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  // Drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);  // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
  s->set_vflip(s, 1);  // 0 = disable , 1 = enable vertical flip
}





// ========TELEGRAM HANDLE MESSAGES===========
void handleNewMessages(int numNewMessages) {
  Serial.print("Handle New Messages: ");
  Serial.println(numNewMessages);

  for (int i = 0; i < numNewMessages; i++) {
    String chat_id = String(bot.messages[i].chat_id);
    if (chat_id != CHAT_ID) {
      bot.sendMessage(chat_id, "Unauthorized user", "");
      continue;
    }

    // Print the received message
    String text = bot.messages[i].text;
    Serial.println(text);

    String from_name = bot.messages[i].from_name;
    if (text == "/start") {
      String welcome = "Welcome , " + from_name + "\n";
      welcome += "Use the following commands to interact with the ESP32-CAM \n";
      welcome += "/photo : takes a new photo\n";
      welcome += "/flash : toggles flash LED \n";
      bot.sendMessage(CHAT_ID, welcome, "");
    }
    if (text == "/flash") {
      flashState = !flashState;
      digitalWrite(FLASH_LED_PIN, flashState);
      Serial.println("Change flash LED state");
    }
    if (text == "/photo") {
      sendPhoto = true;
      Serial.println("New photo request");
    }
  }
}

// ============== SEND PHOTO FUNCTION===============
String sendPhotoTelegram() {
  const char* myDomain = "api.telegram.org";
  String getAll = "";
  String getBody = "";

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    delay(1000);
    ESP.restart();
    return "Camera capture failed";
  }

  Serial.println("Connect to " + String(myDomain));


  if (clientTCP.connect(myDomain, 443)) {
    Serial.println("Connection successful");

    String head = "--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + CHAT_ID + "\r\n--c010blind3ngineer\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--c010blind3ngineer--\r\n";

    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;

    clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
    clientTCP.println("Host: " + String(myDomain));
    clientTCP.println("Content-Length: " + String(totalLen));
    clientTCP.println("Content-Type: multipart/form-data; boundary=c010blind3ngineer");
    clientTCP.println();
    clientTCP.print(head);

    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    for (size_t n = 0; n < fbLen; n = n + 1024) {
      if (n + 1024 < fbLen) {
        clientTCP.write(fbBuf, 1024);
        fbBuf += 1024;
      }
      else if (fbLen % 1024 > 0) {
        size_t remainder = fbLen % 1024;
        clientTCP.write(fbBuf, remainder);
      }
    }

    clientTCP.print(tail);

    esp_camera_fb_return(fb);

    int waitTime = 10000;   // timeout 10 seconds
    long startTimer = millis();
    boolean state = false;

    while ((startTimer + waitTime) > millis()) {
      Serial.print(".");
      delay(100);
      while (clientTCP.available()) {
        char c = clientTCP.read();
        if (state == true) getBody += String(c);
        if (c == '\n') {
          if (getAll.length() == 0) state = true;
          getAll = "";
        }
        else if (c != '\r')
          getAll += String(c);
        startTimer = millis();
      }
      if (getBody.length() > 0) break;
    }
    clientTCP.stop();
    Serial.println(getBody);
  }
  else {
    getBody = "Connected to api.telegram.org failed.";
    Serial.println("Connected to api.telegram.org failed.");
  }
  return getBody;
}

//======================================================DONE=========================================================================


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  // put your setup code here, to run once:
   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
   // Config and init the camera
  configInitCamera();
  
  // Set LED Flash as output
  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, flashState);
  // Set SERVO ATTACH AND DEFAULT POSITION
  myservo.attach(servoPin);
  myservo.write(0);

  // Set PIR sensor as input and LED as output
  pinMode(PIRsensor, INPUT);

  // Give some time for the PIR sensor to warm up
  Serial.println("Waiting for the sensor to warm up on first boot");
  delay(calibrationTime * 10); // Time converted back to miliseconds


  // Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Add root certificate for api.telegram.org
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("ESP32-CAM IP Address: ");
  Serial.println(WiFi.localIP());
  
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // put your main code here, to run repeatedly:
  val = digitalRead(PIRsensor);

  if (val == HIGH) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    if (PIRstate == LOW) {
      // we have just turned on because movement is detected
      Serial.println("Motion detected!");
      delay(500);
      Serial.println("Sending photo to Telegram");
      sendPhotoTelegram();
      PIRstate = HIGH;
    }
  }
  else if (sendPhoto) {
    Serial.println("Preparing photo");
    // SERVO WILL TURN TO 90 DEGREES 
    myservo.write(90);
    digitalWrite(FLASH_LED_PIN, HIGH);
    Serial.println("Flash state set to HIGH");
    delay(500);
    //CALL FUNCTION TO SEND PHOTO TO TELEGRAM
    sendPhotoTelegram();
    sendPhoto = false;
    digitalWrite(FLASH_LED_PIN, LOW);
    // SERVO WILL TURN TO DEFAULT POSITION
    myservo.write(0);
    Serial.println("Flash state set to LOW");
  }
  else if (millis() > lastTimeBotRan + botRequestDelay)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    while (numNewMessages) {
      Serial.println("got response");
      handleNewMessages(numNewMessages);
      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }
    lastTimeBotRan = millis();
  }
  else {
    digitalWrite(FLASH_LED_PIN, LOW);
    if (PIRstate == HIGH) {
      Serial.println("Motion ended!");
      PIRstate = LOW;
    }
  }

}
