/*********
  ESP32-CAM - a cheap ESP-powered portable camera with programmable behaviour. Copyright (c) Playful Technology 2021

  - "TRIGGER" mode captures photo capture based on GPIO input - (which could be a physical button, a sensor such as PIR signal, or trigger from another Arduino)
  - "TIMELAPSE" mode automatically takes pictures every x seconds (configurable)
  - When not taking photo, deep sleep mode activated, which conserves battery life. Can run off single 3.7V Li-Po for a day. 
  - Optional front-facing LED can be turned on to provide a flash  
  - Photos stored to local SD card
  - Photos incrementally numbered
  - Retrieve timestamp from NTP internet time server (requires Wi-Fi)
  - eMail photo as attachment (requires Wi-Fi)
  
  For accompanying video tutorial, see:
  https://youtu.be/FmlxC0goKew

  The following sources provided helpful references (though none of them provided the functionality I wanted!):
  https://RandomNerdTutorials.com/esp32-cam-pir-motion-detector-photo-capture/
  https://github.com/easytarget/esp32-cam-webserver
  https://github.com/bnbe-club/video-recording-with-esp32-cam-diy-11/blob/master/diy-e11/diy-e11.ino
  https://sites.google.com/site/pcusbprojects/5-custom-projects/sc-esp32-cam-based-secure-flash-led-camera
 
  IMPORTANT!!!
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode

   Hardware:
	ESP32-CAM: https://www.banggood.com/custlink/33KEli3R0U
	AM312 PIR sensor: https://www.ebay.co.uk/itm/202625567146
	ESP32-CAM programmer board: https://www.aliexpress.com/item/1005001872947921.html
	FTDI programmer: https://www.banggood.com/custlink/3KKYoIGR6f

*********/

// INCLUDES
#include "Arduino.h" // General functionality
#include "esp_camera.h" // Camera
#include <SD.h> // SD Card 
#include "FS.h" // File System
#include "soc/soc.h" // System settings (e.g. brownout)
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <EEPROM.h> // EEPROM flash memory
#include <WiFi.h> // WiFi
#include "time.h" // Time functions
// "ESP Mail Client" by Mobizt, tested with v1.6.4
#include "ESP_Mail_Client.h" // e-Mail

// DEFINES
//#define USE_INCREMENTAL_FILE_NUMBERING //Uses EEPROM to store latest file stored
#define USE_TIMESTAMP // Uses Wi-Fi to retrieve current time value
#define SEND_EMAIL // Uses Wi-Fi to email photo attachment
//#define TRIGGER_MODE // Photo capture triggered by GPIO pin rising/falling
#define TIMED_MODE // Photo capture automated according to regular delay

// Wi-Fi settings
#define WIFI_SSID "YOUR WIFI SSID"
#define WIFI_PASSWORD "YOUR WIFI PASSWORD"
#define SMTP_HOST "smtp.office365.com"
#define SMTP_PORT 25
#define AUTHOR_EMAIL "xxxx@outlook.com"
#define AUTHOR_PASSWORD "xxxxx"

// CONSTANTS
// GPIO Pin 33 is small red LED near to RESET button on the back of the board
const byte ledPin = GPIO_NUM_33;
// GPIO Pin 4 is bright white front-facing LED 
const byte flashPin = GPIO_NUM_4;
// When using TRIGGER_MODE, this pin will be used to initiate photo capture
const byte triggerPin = GPIO_NUM_13;
// Flash strength (0=Off, 255=Max Brightness)
// Setting a low flash value can provide a useful visual indicator of when a photo is being taken
const byte flashPower = 1;
#ifdef TIMED_MODE
  const int timeLapseInterval = 30; // seconds between successive shots in TIMELAPSE mode
#endif
const int startupDelayMillis = 3000; // time to wait after initialising  camera before taking photo

// GLOBALS
// Keep track of number of pictures taken for incremental file naming
int pictureNumber = 0;
// Full path of filename of the last photo saved
String path;
#ifdef SEND_EMAIL
  // SMTP session used for eMail sending
  SMTPSession smtp;
  // Function fired on email success/failure
  void smtpCallback(SMTP_Status status);

  // Callback function after eMail sending
void smtpCallback(SMTP_Status status) {
  // Print the current status
  Serial.println(status.info());
  // Show details of successful delivery
  if (status.success())   {
    Serial.println("----------------");
    Serial.printf("Message sent success: %d\n", status.completedCount());
    Serial.printf("Message sent failed: %d\n", status.failedCount());
    Serial.println("----------------\n");
    struct tm dt;
    for (size_t i=0; i<smtp.sendingResult.size(); i++) {
      SMTP_Result result = smtp.sendingResult.getItem(i);
      time_t ts = (time_t)result.timestamp;
      localtime_r(&ts, &dt);
      Serial.printf("Message No: %d\n", i + 1);
      Serial.printf("Status: %s\n", result.completed ? "success" : "failed");
      Serial.printf("Date/Time: %d/%d/%d %d:%d:%d\n", dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday, dt.tm_hour, dt.tm_min, dt.tm_sec);
      Serial.printf("Recipient: %s\n", result.recipients);
      Serial.printf("Subject: %s\n", result.subject);
    }
    Serial.println("----------------");
  }
}
#endif

void sleep() {
  // IMPORTANT - we define pin mode for the trigger pin at the end of setup, because most pins on the ESP32-CAM
  // have dual functions, and may have previously been used by the camera or SD card access. So we overwrite them here
  pinMode(triggerPin, INPUT_PULLDOWN);
  // Ensure the flash stays off while we sleep
  rtc_gpio_hold_en(GPIO_NUM_4);
  // Turn off the LED
  digitalWrite(ledPin, HIGH);
  delay(1000);
  #ifdef TRIGGER_MODE
    // Use this to wakeup when trigger pin goes HIGH
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 1);
    // Use this to wakeup when trigger pin goes LOW
    // esp_sleep_enable_ext0_wakeup(GPIO_NUM_13, 0);
  #elif defined(TIMED_MODE)
    // Or, use this to wakeup after a certain amount of time has elapsed (parameter specified in uS, so multiply secs by 1000000)
    esp_sleep_enable_timer_wakeup(timeLapseInterval * 1000000);
  #endif
  Serial.println("Going to sleep now");
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}


void setup() {
  // Light up the discrete red LED on the back of the board to show the device is active
  pinMode(ledPin, OUTPUT);
  // It's an active low pin, so we write a LOW value to turn it on
  digitalWrite(ledPin, LOW);

  // CAUTION - We'll disable to the brownout detection
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  // Start serial connection for debugging purposes
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);
  
  // Pin definition for CAMERA_MODEL_AI_THINKER
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // If the board has additional "pseudo RAM", we can create larger images
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA; // UXGA=1600x1200. Alternative values: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // Use PWM channel 7 to control the white on-board LED (flash) connected to GPIO 4
  ledcSetup(7, 5000, 8);
  ledcAttachPin(4, 7);
  // Turn the LED on at specified power
  ledcWrite(7, flashPower);

  // Initialise the camera
  // Short pause helps to ensure the I2C interface has initialised properly before attempting to detect the camera
  delay(250);
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    sleep();
  }

  // Image settings
  sensor_t * s = esp_camera_sensor_get();
  // Gain
  s->set_gain_ctrl(s, 1);      // Auto-Gain Control 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // Manual Gain 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  // Exposure
  s->set_exposure_ctrl(s, 1);  // Auto-Exposure Control 0 = disable , 1 = enable
  s->set_aec_value(s, 300);    // Manual Exposure 0 to 1200
  // Exposure Correction
  s->set_aec2(s, 0);           // Automatic Exposure Correction 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // Manual Exposure Correction -2 to 2
  // White Balance
  s->set_awb_gain(s, 1);       // Auto White Balance 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // White Balance Mode 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_whitebal(s, 1);       // White Balance 0 = disable , 1 = enable
  s->set_bpc(s, 0);            // Black Pixel Correction 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // White Pixel Correction 0 = disable , 1 = enable
  s->set_brightness(s, 0);     // Brightness -2 to 2
  s->set_contrast(s, 0);       // Contrast -2 to 2
  s->set_saturation(s, 0);     // Saturation -2 to 2
  s->set_special_effect(s, 0); // (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  // Additional settings
  s->set_lenc(s, 1);           // Lens correction 0 = disable , 1 = enable
  s->set_hmirror(s, 0);        // Horizontal flip image 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // Vertical flip image 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // Colour Testbar 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  
  // We want to take the picture as soon as possible after the sensor has been triggered, so we'll do that first, before
  // setting up the SD card, Wifi etc.
  // Initialise a framebuffer 
  camera_fb_t *fb = NULL;
  // But... we still need to give the camera a few seconds to adjust the auto-exposure before taking the picture
  // Otherwise you get a green-tinged image as per https://github.com/espressif/esp32-camera/issues/55
  // Two seconds should be enough
  delay(startupDelayMillis);
  // Take picture
  fb = esp_camera_fb_get();
  // Check it was captured ok  
  if(!fb) {
    Serial.println("Camera capture failed");
    sleep();
  }

  // Turn flash off after taking picture
  ledcWrite(7, 0);

  // Build up the string of the filename we'll use to save the file
  path = "/pic";

  // Following section creates filename based on increment value saved in EEPROM
  #ifdef USE_INCREMENTAL_FILE_NUMBERING
    // We only need 2 bytes of EEPROM to hold a single int value, but according to
    // https://arduino-esp8266.readthedocs.io/en/latest/libraries.html#eeprom
    // Minimum reserved size is 4 bytes, so we'll use that
    EEPROM.begin(4);
    // Read the value from the EEPROM cache
    EEPROM.get(0, pictureNumber);
    pictureNumber += 1;
    // Path where new picture will be saved in SD Card
    path += String(pictureNumber) + "_";
    // Update the EEPROM cache
    EEPROM.put(0, pictureNumber);
    // And then actually write the modified cache values back to EEPROM
    EEPROM.commit();
  #endif

  // Connect to Wi-Fi if required
  #if defined(SEND_EMAIL) || defined(USE_TIMESTAMP)
    WiFi.mode(WIFI_STA);
    WiFi.setHostname("BirdFeederCam");
    int connAttempts = 0;
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED && connAttempts < 10) {
      Serial.print(".");
      delay(500);
      connAttempts++;
    }
    if(WiFi.isConnected()){
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print(" Signal Level: ");
      Serial.println(WiFi.RSSI());
      Serial.println();
    }
    else {
      Serial.println(F("Failed to connect to Wi-Fi"));
      sleep();
    }
  #endif

  #ifdef USE_TIMESTAMP
    // Following section creates filename based on timestamp
    const long gmtOffset_sec = 0;
    const int daylightOffset_sec = 0;
    // Synchronise time from specified NTP server - e.g. "pool.ntp.org", "time.windows.com", "time.nist.gov"
    // From https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Time/SimpleTime/SimpleTime.ino
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org");
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
      sleep();
    }
    else {
      Serial.print("Current time is ");
      Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
      char timeStringBuff[50]; //50 chars should be enough
      strftime(timeStringBuff, sizeof(timeStringBuff), "%Y%m%d_%H%M%S", &timeinfo);
      path += (String)timeStringBuff;
    }
  #endif

  // Add the file extension
  path += ".jpg";
  
  // Next, we need to start the SD card
  Serial.println("Starting SD Card");
  if(!MailClient.sdBegin(14, 2, 15, 13)) {
    Serial.println("SD Card Mount Failed");
    sleep();
  }

  // Access the file system on the SD card
  fs::FS &fs = SD;
  // Attempt to save the image to the specified path
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.printf("Failed to save to path: %s\n", path.c_str());
    sleep();
  }
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
  }
  file.close();

  // Now that we've written the file to SD card, we can release the framebuffer memory of the camera
  esp_camera_fb_return(fb);
  // And breathe for a moment...
  delay(1000);

  #ifdef SEND_EMAIL
    // Get verbose output of emailing process
    smtp.debug(1); 
    // Assign the callback function called after sending
    smtp.callback(smtpCallback);
    // Define the session config data which used to store the TCP session configuration
    ESP_Mail_Session session;
    session.server.host_name = SMTP_HOST;
    session.server.port = SMTP_PORT;
    session.login.email = AUTHOR_EMAIL;
    session.login.password = AUTHOR_PASSWORD;
    session.login.user_domain = "mydomain.net";

    // Define the SMTP_Message class variable to hold the config of the eMail itself
    SMTP_Message message;
    //message.enable.chunking = true; // Enable chunked data transfer for large messages if server supported
    message.sender.name = "ESP32-CAM";
    message.sender.email = AUTHOR_EMAIL;
    message.subject = "Motion Detected - ESP32-CAM";
    message.addRecipient("Me", "nameofrecipient@gmail.com");
    //message.addRecipient("name2", "email2");
    //message.addCc("email3");
    //message.addBcc("email4");
    // Set the message content
    message.text.content = "This is simple plain text message";
    message.text.transfer_encoding = Content_Transfer_Encoding::enc_base64;

    // Now define the attachment properties
    SMTP_Attachment att;
    att.descr.filename = "photo.jpg";
    att.descr.mime = "application/octet-stream"; //binary data
    att.file.path = path.c_str();
    att.file.storage_type = esp_mail_file_storage_type_sd;
    att.descr.transfer_encoding = Content_Transfer_Encoding::enc_base64;
    // Add attachment to the message
    message.addAttachment(att);

    // Connect to server with the session config
    Serial.println("Connecting to SMTP");
    if(!smtp.connect(&session)) {
      Serial.println("Couldn't connect");
      sleep();
    }
    // Start sending Email and close the session
    Serial.println("Sending Mail");
    if(!MailClient.sendMail(&smtp, &message)) {
      Serial.println("Error sending Email, " + smtp.errorReason());
      sleep();
    }
  #endif

  // Now that email is sent, we can turn the Wi-Fi off
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // And go to bed until the next time we are triggered to take a photo
  sleep();
} 
 
void loop() {
}
