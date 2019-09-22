#include <Arduino.h>   // required before wiring_private.h
#include "wiring_private.h" // pinPeripheral() function
#include <IridiumSBD.h>
#include <Adafruit_GPS.h>
#include <Adafruit_ZeroTimer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define DIAGNOSTICS true // Change this to see diagnostics
#define GPS_FIX_REQUIRED false
#define POT_PIN A2
#define BUTTON1_PIN 2
#define BUTTON2_PIN 12
#define LED_RED_PIN 5
#define LED_BLUE_PIN 6
#define LED_GREEN_PIN 7
#define SerialMon SerialUSB

void log(const char *s, boolean newline=true);
void log(int n, boolean newline=true);


boolean modemInitialized = false;
boolean gettingInput = false;
boolean sending = false;
int inputValue;
boolean lastLedOn = false;

// RX = D11 = PA16
// TX = D10 = PA18
// pads 0, 2 on SERCOM (SERCOM1)
Uart Serial2 (&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
IridiumSBD modem(Serial2);

void SERCOM1_Handler() {
  Serial2.IrqHandler();
}

// RX = D3 = PA09
// TX = D4 = PA08
// pads 1, 0 on SERCOM_ALT (SERCOM2)
Uart Serial3 (&sercom2, 3, 4, SERCOM_RX_PAD_1, UART_TX_PAD_0);
Adafruit_GPS GPS(&Serial3);
Adafruit_SSD1306 display(-1);
static const unsigned char nootropicdesign_logo [] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x37, 0xF0, 0x07, 0xF0, 0x0F, 0xE0, 0xFF, 0xE7, 0x7F, 0x01, 0xFE, 0x0D, 0xFE, 0x06, 0x0F, 0xF0,
    0x3C, 0x1E, 0x1C, 0x1C, 0x38, 0x38, 0x70, 0x07, 0xC1, 0xC7, 0x83, 0x8F, 0x83, 0x86, 0x3C, 0x3C,
    0x30, 0x06, 0x30, 0x06, 0x60, 0x0C, 0x30, 0x07, 0x00, 0x66, 0x01, 0x8E, 0x01, 0xC6, 0x30, 0x0C,
    0x30, 0x07, 0x30, 0x06, 0x60, 0x0C, 0x30, 0x07, 0x00, 0x66, 0x01, 0xCC, 0x00, 0xC6, 0x70, 0x00,
    0x30, 0x07, 0x30, 0x06, 0x60, 0x0C, 0x30, 0x07, 0x00, 0x06, 0x01, 0xCC, 0x00, 0xC6, 0x70, 0x00,
    0x30, 0x07, 0x30, 0x06, 0x60, 0x0C, 0x30, 0x07, 0x00, 0x06, 0x01, 0xCC, 0x00, 0xC6, 0x70, 0x00,
    0x30, 0x07, 0x30, 0x06, 0x60, 0x0C, 0x30, 0x37, 0x00, 0x06, 0x01, 0x8C, 0x00, 0xC6, 0x30, 0x0C,
    0x30, 0x07, 0x18, 0x0C, 0x30, 0x18, 0x38, 0x77, 0x00, 0x07, 0x03, 0x8F, 0x03, 0x86, 0x38, 0x1C,
    0x30, 0x07, 0x0F, 0xF8, 0x1F, 0xF0, 0x1F, 0xE7, 0x00, 0x01, 0xFF, 0x0D, 0xFF, 0x06, 0x1F, 0xF8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xF6, 0x07, 0xF0, 0x1F, 0xF0, 0xC0, 0xFF, 0x63, 0xBF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x3C, 0x1E, 0x1C, 0x1C, 0x70, 0x38, 0xC3, 0xC1, 0xE3, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x30, 0x06, 0x30, 0x0E, 0x60, 0x0C, 0xC3, 0x00, 0x63, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x06, 0x30, 0x06, 0x60, 0x00, 0xC7, 0x00, 0x63, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x06, 0x3F, 0xFE, 0x3F, 0xF8, 0xC7, 0x00, 0x63, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x70, 0x06, 0x30, 0x00, 0x00, 0x1C, 0xC7, 0x00, 0x63, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x30, 0x06, 0x30, 0x06, 0x60, 0x0C, 0xC3, 0x00, 0x63, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x38, 0x0E, 0x38, 0x0C, 0x70, 0x1C, 0xC3, 0x80, 0xE3, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0F, 0xFE, 0x0F, 0xF8, 0x3F, 0xF0, 0xC0, 0xFF, 0xE3, 0x80, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void SERCOM2_Handler() {
  Serial3.IrqHandler();
}

Adafruit_ZeroTimer gpsReadTimer = Adafruit_ZeroTimer(4);

int getMemory();

char input[128];
#define MAX_MESSAGE_LEN 128


boolean gpsFix = false;
boolean gpsFixOutput = false;
uint8_t buf[50]; // max payload size for 2 credit message

// current GPS timestamp and position
uint8_t year = 0;
uint8_t month = 0;
uint8_t day = 0;
uint8_t hour = 0;
uint8_t minute = 0;
uint8_t seconds = 0;
int32_t lat = 0;
int32_t lon = 0;

boolean initSatModem() {
  int signalQuality = -1;
  int err;

  // Begin satellite modem operation
  log("  starting modem...", false);
  err = modem.begin();
  if (err != ISBD_SUCCESS) {
    log("");
    digitalWrite(LED_RED_PIN, HIGH);
    log("  error=", false);
    log(err);
    if (err == ISBD_NO_MODEM_DETECTED) {
      log("  no modem detected");
    }
    return false;
  }
  log("OK");

  // Example: Print the firmware revision
  char version[12];
  log("  fw version=", false);
  err = modem.getFirmwareVersion(version, sizeof(version));
  if (err != ISBD_SUCCESS) {
    log("");
    log("  error=");
    log(err);
    digitalWrite(LED_RED_PIN, HIGH);
    return false;
  }
  log(version);
  // Example: Test the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  log("  signal quality=", false);
  err = modem.getSignalQuality(signalQuality);
  if (err != ISBD_SUCCESS)
  {
    log("");
    log("  error=");
    log(err);
    digitalWrite(LED_RED_PIN, HIGH);
    return false;
  }
  log(signalQuality);
  modemInitialized = true;
  return true;
}

void initGPS() {
  GPS.begin(9600);

  // Assign pins 2 & 3 ALT_SERCOM functionality
  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);
  delay(200);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_1HZ);
}

void TC4_Handler() {
  Adafruit_ZeroTimer::timerHandler(4);
}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
void gpsReadCallback() {
  GPS.read();
}

void setup() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_RED_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_GREEN_PIN, LOW);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C, false);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.drawBitmap(0, 15, nootropicdesign_logo, 128, 32, WHITE);
  display.display(); delay(1);
  delay(1000);

  // animation
  int mid = 30;
  display.drawLine(0, mid, display.width()-1, mid, BLACK);
  display.display(); delay(1);
  for(int y=1;y<=mid;y++) {
    display.drawLine(0, mid-y, display.width()-1, mid-y, BLACK);
    display.drawLine(0, mid+y, display.width()-1, mid+y, BLACK);
    display.display(); delay(1);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.display(); delay(1);

  log("init GPS...", false);
  initGPS();
  log("OK");

  gpsReadTimer.configure(TC_CLOCK_PRESCALER_DIV64, TC_COUNTER_SIZE_16BIT, TC_WAVE_GENERATION_MATCH_FREQ);
  int ticks_per_ms = F_CPU / 64 / 1000;
  gpsReadTimer.setCompare(0, ticks_per_ms-1);
  gpsReadTimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, gpsReadCallback);
  gpsReadTimer.enable(true);


  // Start the serial port connected to the satellite modem
  Serial2.begin(19200);
  // Assign pins 10 & 11 SERCOM functionality
  pinPeripheral(11, PIO_SERCOM);
  pinPeripheral(10, PIO_SERCOM);
  log("init satellite modem");
  boolean success = initSatModem();
  if (!success) {
    for(;;);
  }

  delay(2000);
  processGPS();
  if (gpsFix) {
    log("GPS fix acquired");
  } else {
    log("No GPS fix");
  }

  log("READY");
  gpsFixOutput = true;

  for(int i=0;i<3;i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_GREEN_PIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    delay(50);
  }

}

boolean sendMessage(int data) {
  int error;
  int length;

  if ((!gpsFix) && (GPS_FIX_REQUIRED)) {
    log("No GPS fix! Not sending.");
    return false;
  }

  log("Sending data: ", false);
  log(data, false);

  buf[0] = year;
  buf[1] = month;
  buf[2] = day;
  buf[3] = hour;
  buf[4] = minute;
  buf[5] = seconds;

  buf[6] = (lat >> 24) & 0xFF;
  buf[7] = (lat >> 16) & 0xFF;
  buf[8] = (lat >> 8) & 0xFF;
  buf[9] = lat & 0xFF;

  buf[10] = (lon >> 24) & 0xFF;
  buf[11] = (lon >> 16) & 0xFF;
  buf[12] = (lon >> 8) & 0xFF;
  buf[13] = lon & 0xFF;

  buf[14] = (data >> 8) & 0xFF;
  buf[15] = data & 0xFF;

  length = 16;

  sending = true;
  error = modem.sendSBDBinary(buf, length);
  sending = false;
  if (error != ISBD_SUCCESS) {
    log("send failed: error=", false);
    log(error);
    if (error == ISBD_SENDRECEIVE_TIMEOUT) {
      log("timeout");
    }
    return false;
  } else {
    return true;
  }

}


void processGPS() {
  if (GPS.newNMEAreceived()) {
    if (GPS.parse(GPS.lastNMEA())) {
      if (GPS.fix) {
        year = GPS.year;
        month = GPS.month;
        day = GPS.day;
        hour = GPS.hour;
        minute = GPS.minute;
        seconds = GPS.seconds;

        lat = GPS.latitude_fixed;
        if (GPS.lat == 'S') lat *= -1;
        lon = GPS.longitude_fixed;
        if (GPS.lon == 'W') lon *= -1;
      }
    }
  }

  if (gpsFixOutput) {
    if ((!gpsFix) && (GPS.fix)) {
      log("GPS fix acquired");
    }
    if ((gpsFix) && (!GPS.fix)) {
      log("GPS fix lost");
    }
  }
  gpsFix = GPS.fix;
}



void loop() {
  processGPS();

  if (gettingInput) {
    inputValue = analogRead(POT_PIN);
    inputValue = map(inputValue, 10, 1023, 0, 100);
    int cursorY = 20;
    int cursorX = 24;
    if (inputValue < 100) {
      cursorX += 24;
    }
    if (inputValue < 10) {
      cursorX += 24;
    }


    display.clearDisplay();
    display.setCursor(cursorX, cursorY);
    display.print(inputValue);
    display.display(); delay(1);
  }

  if (digitalRead(BUTTON1_PIN) == LOW) {
    delay(20);
    while (digitalRead(BUTTON1_PIN) == LOW);
    if (!gettingInput) {
      gettingInput = true;
      display.clearDisplay();
      display.display(); delay(1);
      display.setTextSize(4);
      digitalWrite(LED_GREEN_PIN, LOW);
      digitalWrite(LED_RED_PIN, LOW);
    } else {
      gettingInput = false;
      display.clearDisplay();
      display.display(); delay(1);
      display.setCursor(0, 0);
      display.setTextSize(1);
      digitalWrite(LED_RED_PIN, LOW);
      digitalWrite(LED_BLUE_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, LOW);
      boolean success = sendMessage(inputValue);
      digitalWrite(LED_BLUE_PIN, LOW);
      if (!success) {
        digitalWrite(LED_RED_PIN, HIGH);
      } else {
        digitalWrite(LED_GREEN_PIN, HIGH);
        log("");
        log("SUCCESS!");
      }
    }
  }

  if (digitalRead(BUTTON2_PIN) == LOW) {
    delay(20);
    while (digitalRead(BUTTON2_PIN) == LOW);
    gettingInput = false;
    display.clearDisplay();
    display.display(); delay(1);
    display.setCursor(0, 0);
    display.setTextSize(1);
  }
}

bool ISBDCallback() {
  unsigned ledOn = (bool)((millis() / 250) % 2);
  if (modemInitialized) {
    digitalWrite(LED_BLUE_PIN, ledOn);
    if (!lastLedOn && ledOn) {
      log(".", false);
    }
    lastLedOn = ledOn;
  }

  if (digitalRead(BUTTON2_PIN) == LOW) {
    delay(20);
    while (digitalRead(BUTTON2_PIN) == LOW);
    // cancel operation
    return false;
  }

  return true;
}


void log(const char *s, boolean newline) {
  SerialMon.print(s);
  if (newline) {
    SerialMon.println();
  }
  if (display.getCursorY() > 58) {
    display.clearDisplay();
    display.setCursor(0, 0);
  }
  display.print(s);
  if (newline) {
    display.println();
  }
  display.display(); delay(1);
}

void log(int n, boolean newline) {
  char s[8];
  sprintf(s, "%d", n);
  log(s, newline);
}

#if DIAGNOSTICS

void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  SerialMon.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  SerialMon.write(c);
}

#endif
