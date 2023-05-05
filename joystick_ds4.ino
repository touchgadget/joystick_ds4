/*********************************************************************
MIT License

Copyright (c) 2023 touchgadgetdev@gmail.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*********************************************************************/

/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* Demostrate reading from a Logitech Extreme 3D Pro flight joystick.
 *
 * Some code is taken from an Adafruit example program so Adafruit's
 * copyright is included.
 *
 * - Device run on the native usb controller with type C USB connector.
 * - Host run on bit-banging 2 GPIOs with the help of Pico-PIO-USB library
 *   with type A USB connector.
 *
 * Requirements:
 * - [Pico-PIO-USB](https://github.com/sekigon-gonnoc/Pico-PIO-USB) library
 * - 2 consecutive GPIOs: D+ is defined by PIN_PIO_USB_HOST_DP, D- = D+ +1
 * - Provide VBus (5v) and GND for peripheral
 * - CPU Speed must be either 120 or 240 Mhz. Selected via "Menu -> CPU Speed"
 */

// Set this to 0 for best performance.
#define USB_DEBUG 1

#if USB_DEBUG
#define DBG_print(...)    Serial.print(__VA_ARGS__)
#define DBG_println(...)  Serial.println(__VA_ARGS__)
#define DBG_printf(...)   Serial.printf(__VA_ARGS__)
#else
#define DBG_print(...)
#define DBG_println(...)
#define DBG_printf(...)
#endif

enum DPAD_DIRECTIONS {
  DPAD_NORTH = 0,
  DPAD_NORTH_EAST = 1,
  DPAD_EAST = 2,
  DPAD_SOUTH_EAST = 3,
  DPAD_SOUTH = 4,
  DPAD_SOUTH_WEST = 5,
  DPAD_WEST = 6,
  DPAD_NORTH_WEST = 7,
};

// Reference: https://www.psdevwiki.com/ps4/DS4-USB
typedef struct __attribute__((packed)) {
  uint8_t reportid;       // 1
  uint8_t leftx;          // Left thumb joystick
  uint8_t lefty;
  uint8_t rightx;         // Right thumb joystick
  uint8_t righty;

  uint8_t dpad:4;         // Direction pad
  uint8_t square:1;
  uint8_t cross:1;
  uint8_t circle:1;
  uint8_t triangle:1;

  uint8_t l1:1;
  uint8_t r1:1;
  uint8_t l2:1;           // Also are reported as analog values
  uint8_t r2:1;           // Also are reported as analog values
  uint8_t share:1;
  uint8_t options:1;
  uint8_t l3:1;           // the joysticks are also buttons
  uint8_t r3:1;

  uint8_t ps_logo:1;
  uint8_t touchpad:1;     // the touchpad is also a button
  uint8_t reportCount:6;

  uint8_t l2_analog;
  uint8_t r2_analog;

  uint16_t timestamp;     // ?
  uint8_t battery_level;
  uint16_t gyrox;
  uint16_t gyroy;
  uint16_t gyroz;
  uint16_t accelx;
  uint16_t accely;
  uint16_t accelz;
  uint8_t unknown1[5];
  uint8_t headset;
  uint8_t tbd[33];
} DS4Report_t;

typedef struct {
  const uint16_t USB_VID = 0x054c;
  const uint16_t USB_PID = 0x09cc;
  DS4Report_t report;
  uint8_t speed = 1;
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} DS4_state_t;

volatile DS4_state_t DS4; // Sony Dual Shock 4

// pio-usb is required for rp2040 usb host
#include "pio_usb.h"
#include "pio-usb-host-pins.h"
#include "Adafruit_TinyUSB.h"

// USB Host object for Logitech joystick
Adafruit_USBH_Host USBHost;

void print_DS4_controls()
{
  DBG_printf("Count:%d,", DS4.report.reportCount);
  DBG_printf("LX:%d,LY:%d,RX:%d,RY:%d,dpad:%d,",
      DS4.report.leftx, DS4.report.lefty, DS4.report.rightx, DS4.report.righty, DS4.report.dpad);
  DBG_printf("L2:%d,R2:%d,", DS4.report.l2_analog, DS4.report.r2_analog);
  if (DS4.report.square) DBG_print("Square,");
  if (DS4.report.cross) DBG_print("Cross,");
  if (DS4.report.circle) DBG_print("Circle,");
  if (DS4.report.triangle) DBG_print("Triangle,");
  if (DS4.report.l1) DBG_print("L1,");
  if (DS4.report.r1) DBG_print("R1,");
  if (DS4.report.l3) DBG_print("L3,");
  if (DS4.report.r3) DBG_print("R3,");
  if (DS4.report.options) DBG_print("Option,");
  if (DS4.report.share) DBG_print("Share,");
  if (DS4.report.ps_logo) DBG_print("PS logo,");
  if (DS4.report.touchpad) DBG_print("Touchpad");
  DBG_println();
  // Ignoring gyros, etc.
}

//--------------------------------------------------------------------+
// Setup and Loop on Core0
//--------------------------------------------------------------------+

void setup()
{
  Serial.begin(115200);
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif

  DBG_println("Sony Dual Shock 4 Demo");
}

void loop() {
  if (DS4.connected) {
    if (DS4.available) {
      if (sizeof(DS4.report) == DS4.report_len) {
#if USB_DEBUG
        // Hex dump the HID report
        uint8_t *rpt = (uint8_t *)&DS4.report;
        DBG_printf("DS4 report(%d): ", DS4.report_len);
        for (uint16_t i = 0; i < DS4.report_len; i++) {
          DBG_printf("%02X", rpt[i]);
        }
        DBG_println();
        print_DS4_controls();
#endif
      }
      DS4.available = false;
    }
  }
}

//--------------------------------------------------------------------+
// Setup and Loop on Core1
//--------------------------------------------------------------------+

void setup1() {
#if USB_DEBUG
  while (!Serial) { delay(1); }
#endif
  DBG_println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
#if USB_DEBUG
    while (!Serial) { delay(1); }
#endif
    DBG_printf("Error: CPU Clock = %lu, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    DBG_println("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed");
    while(1) delay(1);
  }

#ifdef PIN_PIO_USB_HOST_VBUSEN
  pinMode(PIN_PIO_USB_HOST_VBUSEN, OUTPUT);
  digitalWrite(PIN_PIO_USB_HOST_VBUSEN, PIN_PIO_USB_HOST_VBUSEN_STATE);
#endif

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = PIN_PIO_USB_HOST_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1()
{
  USBHost.task();
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use.
// tuh_hid_parse_report_descriptor() can be used to parse common/simple enough
// descriptor. Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE,
// it will be skipped therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *desc_report, uint16_t desc_len) {
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  DBG_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  DBG_printf("VID = %04x, PID = %04x\r\n", vid, pid);
  if ((vid == DS4.USB_VID) && (pid == DS4.USB_PID)) {
    DBG_println("Sony Dual Shock 4 connected");
    DS4.connected = true;
    DS4.available = false;
    DS4.dev_addr = dev_addr;
    DS4.instance = instance;
    memset((DS4Report_t *)&DS4.report, 0, sizeof(DS4.report));
  }
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance) {
  DBG_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  if ((DS4.dev_addr == dev_addr) && (DS4.instance == instance)) {
    if (DS4.connected) {
      DS4.connected = false;
      DS4.available = false;
      DBG_printf("Dual Shock 4 (%d) disconnected\r\n", instance);
    }
  }
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const *report, uint16_t len) {
  if (DS4.connected && (DS4.dev_addr == dev_addr) && (DS4.instance == instance)) {
    memcpy((DS4Report_t *)&DS4.report, report, min(sizeof(DS4.report), len));
    DS4.report_len = len;
    DS4.available = true;
  }

  // continue to request to receive report
  if (!tuh_hid_receive_report(dev_addr, instance)) {
    DBG_printf("Error: cannot request to receive report\r\n");
  }
}
