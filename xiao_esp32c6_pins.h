#pragma once

// ============================================================================
// Seeed Studio XIAO ESP32-C6 Pin Definitions
// Supports Arduino core & gives visibility into all available pads
// ============================================================================

// ---------------------------- Digital / Arduino D-Pins ----------------------
#define PIN_D0          0   // LP_GPIO0 / A0
#define PIN_D1          1   // LP_GPIO1 / A1
#define PIN_D2          2   // LP_GPIO2 / A2
#define PIN_D3          21  // SDIO_DATA1 / A3
#define PIN_D4          22  // SDA / SDIO_DATA2
#define PIN_D5          23  // SCL / SDIO_DATA3
#define PIN_D6          16  // TX
#define PIN_D7          17  // RX
#define PIN_D8          19  // SPI SCK / SDIO_CLK
#define PIN_D9          20  // SPI MISO / SDIO_DATA0
#define PIN_D10         18  // SPI MOSI / SDIO_CMD

// ---------------------------- Analog Input Aliases --------------------------
#define PIN_A0          PIN_D0
#define PIN_A1          PIN_D1
#define PIN_A2          PIN_D2
#define PIN_A3          PIN_D3
#define PIN_A4          4
#define PIN_A5          5
#define PIN_A6          6 // LP_GPIO6

// ---------------------------- UART -----------------------------------------
#define PIN_TX          PIN_D6
#define PIN_RX          PIN_D7

// ---------------------------- I2C ------------------------------------------
#define PIN_I2C_SDA     PIN_D4   // GPIO22
#define PIN_I2C_SCL     PIN_D5   // GPIO23

// ---------------------------- SPI ------------------------------------------
#define PIN_SPI_MOSI    PIN_D10  // GPIO18
#define PIN_SPI_MISO    PIN_D9   // GPIO20
#define PIN_SPI_SCK     PIN_D8   // GPIO19
// No fixed CS — choose your own
#define PIN_SPI_CS      -1

// ---------------------------- Low Power GPIOs -------------------------------
#define LP_GPIO0        0
#define LP_GPIO1        1
#define LP_GPIO2        2
#define LP_GPIO6        6
#define LP_GPIO7        7  // Available pad
#define LP_GPIO4        4  // (Dual name: also normal GPIO4)
#define LP_GPIO5        5  // (Dual name: also normal GPIO5)

// ---------------------------- JTAG & System Pins ---------------------------
#define PIN_MTDI        5   // Shares with A5 / LP_GPIO5
#define PIN_MTDO        7   // LP_GPIO7
#define PIN_MTCK        6   // LP_GPIO6 / A6
#define PIN_MTMS        4   // LP_GPIO4 / A4

// ---------------------------- Boot Button / Strap --------------------------
#define PIN_BOOT        9   // BOOT — avoid using unless you know boot logic

// ---------------------------- Power / System -------------------------------
#define PIN_VBUS        0xFF // USB 5V sense (not a GPIO)
#define PIN_5V          0xFE
#define PIN_3V3         0xFD
#define PIN_GND         0xFC
#define PIN_BAT         0xFB

// ---------------------------- Notes ----------------------------------------
// * LP_GPIO = low-power capable pins (best for wake sources)
// * Avoid driving PIN_BOOT (GPIO9) — can affect boot mode
// * SPI CS must be user-assigned (no dedicated CS pad)
// * Power pins are constants and NOT usable in pinMode()
// ============================================================================