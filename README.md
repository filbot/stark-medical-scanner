# Stark “Medical Scanner” — Iron Man 2 Prop

This repository contains an Arduino sketch that mimics the handheld **Stark Medical Scanner** from *Iron Man 2*.  
It doesn’t measure anything real — it’s a **visual prop** that displays a green label and a blinking red percentage when you “take a reading,” complete with a short beep sequence and status indicators.

---

## What the Code Does

- Boots a **1.9” ST7789 TFT** and draws a 5×7 dot-matrix UI  
- Displays a static green label: `BLOOD TOXICITY:`  
- On button press:
  - Plays a short beep burst, then a final beep  
  - Animates the label drawing in  
  - Generates a random **10–99%** reading  
  - Displays the percentage in **blinking red**  
  - Draws two small status boxes at the bottom (green + red) as light guides for the prop  
  - Returns to idle after ~10s of inactivity  

> No external sensors are used — the “reading” is purely for show.

---

## Hardware

| Component | Description |
|------------|-------------|
| **MCU** | Seeed Studio XIAO ESP32-C6 |
| **Display** | Waveshare 1.9” 170×320 ST7789 SPI TFT |
| **Input** | 1 × momentary push button (with 10kΩ pull-up) |
| **Audio** | Piezo buzzer (for scan beeps) |
| **Libraries** | Adafruit_GFX, Adafruit_ST7789, SPI |

If you’re new to Arduino:  
Install the libraries listed above from **Library Manager**, select **XIAO ESP32-C6** as the board, and upload the sketch.

---

## Wiring (XIAO ESP32-C6 → Display & IO)

| Function | XIAO Pin | Notes |
|-----------|-----------|-------|
| **TFT SCK** | D8 | Hardware SPI (GPIO19) |
| **TFT MOSI** | D10 | Hardware SPI (GPIO18) |
| **TFT CS** | D0 | Chip Select (GPIO0) |
| **TFT DC** | D1 | Data/Command (GPIO1) |
| **TFT RST** | D2 | Reset (GPIO2) |
| **TFT BL** | D3 | Backlight (GPIO21) |
| **VCC / GND** | 3V3 / GND | Power |
| **Button** | D4 | To GND (with 10kΩ pull-up) |
| **Buzzer +** | D5 | Piezo signal |
| **Buzzer −** | GND | — |

The sketch defines these same pins.  
If you wire differently, update the `#define` statements near the top of the code.

---

## How This Maps to the Movie Prop

- **Screen window:** The TFT shows the green label and red percent in the same layout as the movie.  
- **“BLOOD TOXICITY 53%” display:** Label is green; value blinks red to match the on-screen aesthetic.  
- **Button:** Mapped to a momentary button, press it to “scan.”  
- **Tiny indicator LEDs:** Rendered as two small boxes (green + red) at the bottom of the display.  
- **Audible feedback:** The beep pattern simulates a completed scan.  

> This is a display prop, not a medical device.

---

## Getting Started

1. Install **ESP32 board support** for *Seeed XIAO ESP32-C6*.  
2. Install **Adafruit_GFX** and **Adafruit_ST7789** libraries.  
3. Wire the hardware according to the table above.  
4. Open `stark-medical-scanner.ino`, build, and upload.  
5. Press the button to “scan.”

---

## Credits & Disclaimer

Inspired by the *Stark Medical Scanner* from **Iron Man 2**.  
This project is for **cosplay and prop display only**, it does **not** measure blood toxicity.
