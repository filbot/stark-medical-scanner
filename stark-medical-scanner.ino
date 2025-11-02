/*
 * Blood Toxicity Monitor Display
 * For Seeed Studio XIAO ESP32-C6 + Waveshare 1.9" LCD
 * 
 * Features:
 * - Dot matrix display showing "GRAVY BLOOD LVL:XX%"
 * - Green static label, red blinking percentage
 * - Button-activated reading
 * - Piezoelectric beeper (rapid beeps on reading acquisition)
 * - Green and red indicator boxes at bottom
 * - Display turns off after 10 seconds (device stays awake)
 * 
 * Hardware Connections:
 * Display VCC  → 3V3
 * Display GND  → GND
 * Display SCK  → D8  (GPIO19)
 * Display MOSI → D10 (GPIO18)
 * Display CS   → D0  (GPIO0)
 * Display DC   → D1  (GPIO1)
 * Display RST  → D2  (GPIO2)
 * Display BL   → D3  (GPIO21)
 * 
 * Button       → D4  (GPIO7)  - Momentary push button (S to GND, has external 10kΩ pull-up)
 * Buzzer (+)   → D5  (GPIO22) - Piezo buzzer positive
 * Buzzer (-)   → GND          - Piezo buzzer negative
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "xiao_esp32c6_pins.h"
#include <esp_sleep.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#if defined(ARDUINO_USB_MODE) || defined(ARDUINO_USB_CDC_ON_BOOT)
#include <HWCDC.h>
#endif
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 1) && defined(CONFIG_TINYUSB_ENABLED)
#include "esp32-hal-tinyusb.h"
#endif

// Pin definitions - XIAO ESP32-C6 outer header pins only
#define TFT_CS 0     // D0  = GPIO0  - Chip Select
#define TFT_DC 1     // D1  = GPIO1  - Data/Command
#define TFT_RST 2    // D2  = GPIO2  - Reset
#define TFT_BL 21    // D3  = GPIO21 - Backlight
#define TFT_SCLK 19  // D8  = GPIO19 - SCK (Hardware SPI)
#define TFT_MOSI 18  // D10 = GPIO18 - MOSI (Hardware SPI)

// User input/output pins
#define BUTTON_PIN LP_GPIO4  // A4 header -> LP_GPIO4 (RTC-capable) - Button with external 10kΩ pull-up
#define BUZZER_PIN 23  // D5  = GPIO23 - Piezo buzzer (OUTPUT)
constexpr gpio_num_t BUTTON_WAKE_GPIO = GPIO_NUM_4;
constexpr uint64_t BUTTON_WAKE_MASK = 1ULL << BUTTON_WAKE_GPIO;

// Persisted state across deep sleep
RTC_DATA_ATTR bool storedThreeColorMode = false;

// Forward declarations for power management helpers
static bool stayAwakeForUsb();
static void enterDeepSleep();
static void displayEnterSleep();

// Display object - using hardware SPI for optimal performance
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Display dimensions (physical: 170x320, landscape: 320x170)
const uint16_t PANEL_W = 170;
const uint16_t PANEL_H = 320;

// Colors (RGB565 format)
const uint16_t COLOR_BLACK = ST77XX_BLACK;
const uint16_t COLOR_GREEN = 0x07E0;
const uint16_t COLOR_YELLOW = 0xFFE0;  // Yellow (full red + full green)
const uint16_t COLOR_RED = 0xF800;

// 5x7 dot matrix font data stored in flash memory
const char GLYPH_MAP[] PROGMEM = " 0123456789:%ABCDEFGHIJKLMNOPQRSTUVWXYZ";
const uint8_t GLYPH_DATA[][5] PROGMEM = {
  { 0x00, 0x00, 0x00, 0x00, 0x00 },
  { 0x3E, 0x51, 0x49, 0x45, 0x3E },
  { 0x00, 0x42, 0x7F, 0x40, 0x00 },
  { 0x42, 0x61, 0x51, 0x49, 0x46 },
  { 0x21, 0x41, 0x45, 0x4B, 0x31 },
  { 0x18, 0x14, 0x12, 0x7F, 0x10 },
  { 0x27, 0x45, 0x45, 0x45, 0x39 },
  { 0x3C, 0x4A, 0x49, 0x49, 0x30 },
  { 0x01, 0x71, 0x09, 0x05, 0x03 },
  { 0x36, 0x49, 0x49, 0x49, 0x36 },
  { 0x06, 0x49, 0x49, 0x29, 0x1E },
  { 0x00, 0x00, 0x24, 0x00, 0x00 },
  { 0x62, 0x64, 0x08, 0x13, 0x23 },
  { 0x7E, 0x11, 0x11, 0x11, 0x7E },
  { 0x7F, 0x49, 0x49, 0x49, 0x36 },
  { 0x3E, 0x41, 0x41, 0x41, 0x22 },
  { 0x7F, 0x41, 0x41, 0x22, 0x1C },
  { 0x7F, 0x49, 0x49, 0x49, 0x41 },
  { 0x7F, 0x09, 0x09, 0x09, 0x01 },
  { 0x3E, 0x41, 0x49, 0x49, 0x7A },
  { 0x7F, 0x08, 0x08, 0x08, 0x7F },
  { 0x00, 0x41, 0x7F, 0x41, 0x00 },
  { 0x20, 0x40, 0x41, 0x3F, 0x01 },
  { 0x7F, 0x08, 0x14, 0x22, 0x41 },
  { 0x7F, 0x40, 0x40, 0x40, 0x40 },
  { 0x7F, 0x02, 0x04, 0x02, 0x7F },
  { 0x7F, 0x04, 0x08, 0x10, 0x7F },
  { 0x3E, 0x41, 0x41, 0x41, 0x3E },
  { 0x7F, 0x09, 0x09, 0x09, 0x06 },
  { 0x3E, 0x41, 0x51, 0x21, 0x5E },
  { 0x7F, 0x09, 0x19, 0x29, 0x46 },
  { 0x46, 0x49, 0x49, 0x49, 0x31 },
  { 0x01, 0x01, 0x7F, 0x01, 0x01 },
  { 0x3F, 0x40, 0x40, 0x40, 0x3F },
  { 0x1F, 0x20, 0x40, 0x20, 0x1F },
  { 0x7F, 0x20, 0x18, 0x20, 0x7F },
  { 0x63, 0x14, 0x08, 0x14, 0x63 },
  { 0x07, 0x08, 0x70, 0x08, 0x07 },
  { 0x61, 0x51, 0x49, 0x45, 0x43 }
};

// Layout configuration (must be defined before functions that use them)
const int16_t BASE_X = 2;                    // Starting X position for text
const uint8_t DOT_R = 1;                     // Dot radius in pixels
const uint8_t PITCH = 3;                     // Spacing between dots
const uint8_t GAP = 1;                       // Gap between characters
const uint8_t CHAR_WIDTH = 5 * PITCH + GAP;  // Precomputed character width (16 pixels)

// Position adjustment offsets - ADJUST THESE VALUES TO MOVE ELEMENTS
const int16_t TEXT_Y_OFFSET = 68;            // Vertical position for text (0 = top of display, positive = down)
const int16_t GREEN_BOX_X_OFFSET = -10;     // Horizontal offset for green box (negative = left, positive = right)
const int16_t RED_BOX_X_OFFSET = 0;          // Horizontal offset for red box (negative = left, positive = right)

// Fast glyph lookup - converts character to font array index
int8_t glyphIndex(char c) {
  // Convert lowercase to uppercase
  if (c >= 'a' && c <= 'z') {
    c -= 32;  // Faster than c - 'a' + 'A'
  }

  // Fast lookup for common characters
  if (c == ' ') return 0;
  if (c >= '0' && c <= '9') return c - '0' + 1;
  if (c == ':') return 11;
  if (c == '%') return 12;
  if (c >= 'A' && c <= 'Z') return c - 'A' + 13;

  return -1;  // Character not found
}

// Draw a single character using dot matrix font
void drawDotChar(int16_t x, int16_t y, char c, uint16_t color) {
  int8_t idx = glyphIndex(c);
  if (idx < 0) return;

  // Read 5 columns of glyph data from flash
  uint8_t cols[5];
  for (uint8_t i = 0; i < 5; i++) {
    cols[i] = pgm_read_byte(&GLYPH_DATA[idx][i]);
  }

  // Draw each dot in the 5x7 matrix
  for (uint8_t cx = 0; cx < 5; cx++) {
    uint8_t col = cols[cx];
    for (uint8_t ry = 0; ry < 7; ry++) {
      if (col & (1 << ry)) {  // Optimized bit check
        int16_t px = x + cx * PITCH;
        int16_t py = y + ry * PITCH;
        tft.fillCircle(px, py, DOT_R, color);
      }
    }
  }
}

// Draw a string using dot matrix font
// Returns the width in pixels of the rendered string
int16_t drawDotString(int16_t x, int16_t y, const char* str, uint16_t color) {
  int16_t curX = x;

  for (const char* p = str; *p; ++p) {
    drawDotChar(curX, y, *p, color);
    curX += CHAR_WIDTH;
  }

  return curX - x;
}

// Feature flags
bool ENABLE_THREE_COLOR_MODE = false;  // false = green label + red value, true = value color based on range

// Timing configuration
const uint32_t BLINK_INTERVAL_MS = 500;     // Blink toggle interval
const uint32_t DISPLAY_ON_TIME_MS = 10000;  // Display stays on for 10 seconds
const uint32_t LONG_PRESS_DURATION_MS = 3000;  // 3 seconds for long press

// Beep pattern settings
const uint8_t RAPID_BEEP_COUNT = 20;   // Number of rapid beeps
const uint32_t RAPID_BEEP_MS = 15;     // Duration of each rapid beep
const uint32_t RAPID_GAP_MS = 15;      // Gap between rapid beeps
const uint32_t FINAL_BEEP_MS = 30;     // Duration of final longer beep
const uint16_t BEEP_FREQUENCY = 2000;  // 2 kHz tone

// State variables
uint32_t lastBlink = 0;
uint32_t displayOnTime = 0;
bool showValue = true;
bool displayActive = false;
bool readingInProgress = false;

// Long-press detection for color mode switch
uint32_t buttonPressStartTime = 0;
bool buttonHeldDown = false;
bool longPressTriggered = false;

// Display text configuration - SINGLE SOURCE OF TRUTH
const char LABEL_TEXT[] = "GRAVY BLOOD LVL:";      // Static label text (green)
char valueText[5] = "89%";                          // Mutable value string (format: "XX%")
int currentValue = 89;                              // Current numeric value (10-99)

// Calculated positioning variables (computed at runtime)
int16_t labelLen = 0;                               // Length of label string
int16_t valueLen = 0;                               // Length of value string
int16_t valueX = 0;                                 // X position of value text
int16_t valueY = 0;                                 // Y position (computed with offset)
int16_t valueWidth = 0;                             // Width of value area for clearing
const int16_t valueHeight = 7 * PITCH + DOT_R * 2;  // Height (constant)

// Indicator box configuration
int16_t greenBoxX = 0;
int16_t redBoxX = 0;
const int16_t BOX_SIZE = 23;
const int16_t BOX_Y = PANEL_W - BOX_SIZE;  // Bottom of display (landscape mode)

// Calculate all text positions based on label length (call this once at startup)
void calculateTextPositions() {
  labelLen = strlen(LABEL_TEXT);
  valueLen = strlen(valueText);
  
  // Use TEXT_Y_OFFSET directly as the Y position (0 = top of display)
  valueY = TEXT_Y_OFFSET;
  
  // Value starts immediately after the label
  valueX = BASE_X + (labelLen * CHAR_WIDTH);
  
  // Width needed to clear the value area
  valueWidth = valueLen * CHAR_WIDTH;
  
  // Position boxes at bottom-right corner of display with offsets
  // Red box aligned to bottom-right corner + offset
  redBoxX = PANEL_H - BOX_SIZE + RED_BOX_X_OFFSET;
  
  // Green box to the left of red box with gap + offset
  greenBoxX = redBoxX - BOX_SIZE - 3 + GREEN_BOX_X_OFFSET;
}

// Get color based on value range
uint16_t getValueColor(int value) {
  if (!ENABLE_THREE_COLOR_MODE) {
    // Two-color mode: always red for values
    return COLOR_RED;
  }

  // Three-color mode: color based on value range
  if (value >= 10 && value <= 59) {
    return COLOR_GREEN;
  } else if (value >= 60 && value <= 79) {
    return COLOR_YELLOW;
  } else {  // 80-99
    return COLOR_RED;
  }
}

// Draw indicator box at bottom of display with current value color
void drawValueBox() {
  uint16_t color = getValueColor(currentValue);
  tft.fillRect(redBoxX, BOX_Y, BOX_SIZE, BOX_SIZE, color);
}

// Draw green indicator box at bottom of display
void drawGreenBox() {
  tft.fillRect(greenBoxX, BOX_Y, BOX_SIZE, BOX_SIZE, COLOR_GREEN);
}

// Clear both indicator boxes
void clearIndicatorBoxes() {
  tft.fillRect(greenBoxX, BOX_Y, BOX_SIZE, BOX_SIZE, COLOR_BLACK);
  tft.fillRect(redBoxX, BOX_Y, BOX_SIZE, BOX_SIZE, COLOR_BLACK);
}

// Initialize display and draw static elements
void drawLabel() {
  tft.fillScreen(COLOR_BLACK);

  // Draw green label text
  drawDotString(BASE_X, valueY, LABEL_TEXT, COLOR_GREEN);

  // Draw only green box initially (red box appears with value)
  drawGreenBox();
}

// Draw label progressively, one character at a time
void drawLabelProgressive(int charCount) {
  static int lastCharCount = 0;  // Track how many chars were drawn last time

  // Clear screen and show green box on first character
  if (charCount == 1) {
    tft.fillScreen(COLOR_BLACK);
    lastCharCount = 0;

    // Show green box at the start
    drawGreenBox();
  }

  // Draw only the NEW characters since last call
  if (charCount <= labelLen && charCount > lastCharCount) {
    for (int i = lastCharCount; i < charCount; i++) {
      int16_t x = BASE_X + i * CHAR_WIDTH;
      drawDotChar(x, valueY, LABEL_TEXT[i], COLOR_GREEN);
    }
    lastCharCount = charCount;
  }
}

// Draw or clear the blinking value text
void drawValue(bool visible) {
  // Clear the value area with extra margin to eliminate any stray pixels
  // Start clearing 4 pixels before valueX to catch any anti-aliasing or overlap
  int16_t clearX = valueX - 4;
  int16_t clearWidth = valueWidth + 8;  // Extra margin on both sides
  tft.fillRect(clearX, valueY - DOT_R - 1, clearWidth, valueHeight + 2, COLOR_BLACK);

  // Draw text with color based on value if visible
  if (visible) {
    uint16_t color = getValueColor(currentValue);
    drawDotString(valueX, valueY, valueText, color);
  }
}

// Generate random percentage value (10-99)
void updateRandomValue() {
  currentValue = random(10, 100);
  snprintf(valueText, sizeof(valueText), "%d%%", currentValue);
  
  // Recalculate value width in case length changed (e.g., "9%" vs "99%")
  valueLen = strlen(valueText);
  valueWidth = valueLen * CHAR_WIDTH;
}

// Play rapid beeps followed by one longer tone, drawing value on final beep
void playAcquisitionBeeps() {
  int halfBeeps = RAPID_BEEP_COUNT / 2;  // Complete text sweep in first half of beeps

  // Draw label progressively during rapid beeps
  for (int i = 0; i < RAPID_BEEP_COUNT; i++) {
    // Calculate how many characters to show (complete in first half)
    int charsToShow;
    if (i < halfBeeps) {
      // First half: spread all characters across half the beeps
      charsToShow = ((i + 1) * labelLen) / halfBeeps;
    } else {
      // Second half: all characters already shown
      charsToShow = labelLen;
    }
    drawLabelProgressive(charsToShow);

    tone(BUZZER_PIN, BEEP_FREQUENCY, RAPID_BEEP_MS);
    delay(RAPID_BEEP_MS);
    noTone(BUZZER_PIN);
    if (i < RAPID_BEEP_COUNT - 1) {  // No gap after last rapid beep
      delay(RAPID_GAP_MS);
    }
  }

  // Make sure all characters are drawn
  drawLabelProgressive(labelLen);

  // Short pause before final beep
  delay(50);

  // Clear green box and show value box with color based on value
  tft.fillRect(greenBoxX, BOX_Y, BOX_SIZE, BOX_SIZE, COLOR_BLACK);  // Clear green
  drawValueBox();                                                   // Show box with color matching value

  // Reset blink timer and show value right before the final beep
  lastBlink = millis();  // Reset blink timer so value stays visible
  showValue = true;      // Ensure value is visible
  drawValue(true);

  // One longer final beep
  tone(BUZZER_PIN, BEEP_FREQUENCY, FINAL_BEEP_MS);
  delay(FINAL_BEEP_MS);
  noTone(BUZZER_PIN);
}

// Simulate acquiring a reading (generates random value)
void acquireReading() {
  readingInProgress = true;

  // Generate new random value
  updateRandomValue();

  // Play beeps to indicate acquisition
  playAcquisitionBeeps();

  readingInProgress = false;
}

// Returns true when a USB host connection is active (Serial monitor/open CDC)
static bool stayAwakeForUsb() {
#if defined(ARDUINO_USB_MODE) && (ARDUINO_USB_MODE == 1) && defined(CONFIG_TINYUSB_ENABLED)
  return TinyUSBDevice.mounted();
#elif defined(ARDUINO_USB_MODE) || defined(ARDUINO_USB_CDC_ON_BOOT)
  return HWCDC::isConnected();
#else
  return false;
#endif
}

static void displayEnterSleep() {
  tft.writeCommand(ST77XX_SLPIN);
  delay(10);
  tft.writeCommand(ST77XX_DISPOFF);
}

// Prepare peripherals for deep sleep and enter low-power mode unless USB keeps us awake
static void enterDeepSleep() {
  bool usbConnected = stayAwakeForUsb();

  noTone(BUZZER_PIN);
  pinMode(BUZZER_PIN, INPUT);  // High-impedance to stop any leakage

  digitalWrite(TFT_BL, LOW);

  // Keep the wake pin pulled up regardless so it is ready to wake us
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  delay(1);  // Allow GPIO configuration to settle

  if (usbConnected) return;

  // Ensure RTC domain keeps the button pulled up during deep sleep
  rtc_gpio_pulldown_dis(BUTTON_WAKE_GPIO);
  rtc_gpio_pullup_en(BUTTON_WAKE_GPIO);
  rtc_gpio_hold_en(BUTTON_WAKE_GPIO);
  gpio_hold_en(static_cast<gpio_num_t>(TFT_BL));

  if (digitalRead(BUTTON_PIN) == LOW) return;

  displayEnterSleep();

  // Ensure SPI transactions are idle before shutting down
  SPI.end();

  esp_sleep_enable_ext1_wakeup(BUTTON_WAKE_MASK, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  delay(10);
  esp_deep_sleep_start();
}

// Turn on display and backlight (clears screen, ready for progressive drawing)
void displayOn() {
  digitalWrite(TFT_BL, HIGH);
  displayActive = true;
  displayOnTime = millis();
  lastBlink = millis();
  showValue = true;
  tft.fillScreen(COLOR_BLACK);  // Clear screen, ready for progressive drawing
  // Don't draw label yet - will be drawn progressively during beeps
}

// Turn on display with value (used for initial boot and when display already on)
void displayOnWithValue() {
  digitalWrite(TFT_BL, HIGH);
  displayActive = true;
  displayOnTime = millis();
  lastBlink = millis();
  showValue = true;
  drawLabel();
  drawValue(true);
}

// Turn off display and backlight, then enter deep sleep unless USB keeps us awake
void displayOff() {
  digitalWrite(TFT_BL, LOW);
  tft.fillScreen(COLOR_BLACK);
  displayActive = false;
  enterDeepSleep();
}

// Display mode switch message
void displayModeMessage(const char* message) {
  digitalWrite(TFT_BL, HIGH);
  tft.fillScreen(COLOR_BLACK);
  
  // Center the message on screen
  // Each character is CHAR_WIDTH pixels wide
  int16_t messageLen = strlen(message);
  int16_t messageWidth = messageLen * CHAR_WIDTH;
  int16_t centerX = (PANEL_H - messageWidth) / 2;  // PANEL_H = 320 (width in landscape)
  int16_t centerY = (PANEL_W / 2) - 10;  // PANEL_W = 170 (height in landscape), center vertically
  
  // Draw message in green
  drawDotString(centerX, centerY, message, COLOR_GREEN);
  
  // Show for 2 seconds
  delay(2000);
}

// Toggle color mode
void toggleColorMode() {
  ENABLE_THREE_COLOR_MODE = !ENABLE_THREE_COLOR_MODE;
  storedThreeColorMode = ENABLE_THREE_COLOR_MODE;

  displayModeMessage(ENABLE_THREE_COLOR_MODE ? "THREE COLOR" : "SCREEN ACCURATE");

  // Turn off display after showing message
  displayOff();
}

void setup() {
  bool usbConnected = stayAwakeForUsb();
  delay(usbConnected ? 200 : 10);

  // Initialize ESP32-C6 hardware random number generator
  randomSeed(esp_random());
  ENABLE_THREE_COLOR_MODE = storedThreeColorMode;

  // Configure button pin with internal pull-up (backup for external pull-up)
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  rtc_gpio_hold_dis(BUTTON_WAKE_GPIO);
  rtc_gpio_init(BUTTON_WAKE_GPIO);
  rtc_gpio_set_direction(BUTTON_WAKE_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_en(BUTTON_WAKE_GPIO);
  rtc_gpio_pulldown_dis(BUTTON_WAKE_GPIO);
  delay(10);  // Give pull-up time to stabilize

  // Configure buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Configure hardware SPI with optimal settings for ESP32-C6
  // Parameters: SCK, MISO (unused), MOSI, CS
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);

  // Configure backlight
  pinMode(TFT_BL, OUTPUT);
  gpio_hold_dis(static_cast<gpio_num_t>(TFT_BL));

  // Initialize display once, otherwise wake it from panel sleep
  tft.init(PANEL_W, PANEL_H, SPI_MODE0);
  tft.setSPISpeed(40000000);  // 40 MHz - ESP32-C6 can handle high SPI speeds
  tft.setRotation(3);         // Landscape orientation rotated 180 degrees (320x170)

  // Calculate text positions based on label string
  calculateTextPositions();

  // Initial boot - display on and acquire reading with beeps
  updateRandomValue();     // Generate initial value
  displayOn();             // Turn on display (label only)
  playAcquisitionBeeps();  // Play beeps and show value on final beep
}

void loop() {
  uint32_t now = millis();

  // If display is active, handle blinking and timeout
  if (displayActive) {
    // Handle blink timing (non-blocking)
    if (now - lastBlink >= BLINK_INTERVAL_MS) {
      lastBlink = now;
      showValue = !showValue;
      drawValue(showValue);
      
      // Check if we should turn off display after this blink
      // Only turn off when timeout is reached AND value just blinked off
      if (!showValue && (now - displayOnTime >= DISPLAY_ON_TIME_MS)) {
        displayOff();
      }
    }
  }

  // Check for button press with debouncing
  static bool lastButtonState = HIGH;
  static bool buttonPressed = false;
  static uint32_t lastDebounceTime = 0;
  const uint32_t debounceDelay = 50;  // 50ms debounce

  bool buttonReading = digitalRead(BUTTON_PIN);

  // If button state changed, reset debounce timer
  if (buttonReading != lastButtonState) {
    lastDebounceTime = now;
    lastButtonState = buttonReading;
  }

  // Check if button has been stable for debounce period
  if ((now - lastDebounceTime) > debounceDelay) {
    // Button is stable - check for press event
    if (buttonReading == LOW && !buttonPressed) {
      // Button pressed DOWN
      buttonPressed = true;
      buttonHeldDown = true;
      buttonPressStartTime = now;
      longPressTriggered = false;
      
    } else if (buttonReading == HIGH && buttonPressed) {
      // Button released UP
      uint32_t pressDuration = now - buttonPressStartTime;
      buttonPressed = false;
      buttonHeldDown = false;
      
      // Process short press if it wasn't a long press
      if (!longPressTriggered && pressDuration < LONG_PRESS_DURATION_MS && !readingInProgress) {
        if (displayActive) {
          // Display is on - take new reading and reset timeout
          updateRandomValue();
          tft.fillScreen(COLOR_BLACK);
          playAcquisitionBeeps();
          displayOnTime = millis();
        } else {
          // Display is off - turn on, show label, beep, then show value
          updateRandomValue();
          displayOn();
          playAcquisitionBeeps();
        }
      }
    }
  }
  
  // Check for long press (3 seconds) while button is held
  if (buttonHeldDown && !longPressTriggered && (now - buttonPressStartTime) >= LONG_PRESS_DURATION_MS) {
    longPressTriggered = true;
    toggleColorMode();
    buttonPressed = false;
    buttonHeldDown = false;
    return;
  }

  // Small delay to prevent excessive CPU usage
  delay(10);
}
