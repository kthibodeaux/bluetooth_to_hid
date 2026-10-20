# Bluetooth to HID

ESP32-S3 firmware that bridges a BLE keyboard to a wired USB keyboard. The
keyboard pairs with the S3 over BLE, and the S3 shows up on its native USB
port as a plain boot-protocol keyboard, so KVM switches and BIOS screens
accept it like any other wired keyboard.

> Status: milestone 5. Bridging, bonded reconnect after reset or power loss,
> and bond reset via the BOOT button all work. Tested with a ZMK Corne.

> **Disclaimer:** this project is vibe coded. It was written almost entirely
> by an AI assistant (Claude), directed and tested on real hardware, but not
> line-by-line reviewed. It works on one ESP32-S3 dev board with one ZMK
> keyboard. Treat it as a starting point, not production firmware.

## Layout

- `components/ble_keyboard`: BLE central / HID-over-GATT host (NimBLE).
  Handles scanning, pairing, bonding (in NVS), and reconnects, and delivers
  raw 8-byte boot reports plus connection-state callbacks. It knows nothing
  about USB.
- `components/usb_keyboard`: a TinyUSB (`espressif/esp_tinyusb`) device with
  a single boot keyboard interface. Provides `usb_keyboard_send_report()` and
  `usb_keyboard_release_all()`. It knows nothing about Bluetooth.
- `main`: wiring only. Initializes NVS and both components and forwards
  reports through a FreeRTOS queue. On BLE disconnect it sends an
  all-keys-released report. If a USB send fails while a host is attached, it
  retries the latest key state until it goes through, so a lost key-up can't
  leave a key stuck down on the KVM.

## Board ports

| Port | Connects to | Used for |
|---|---|---|
| UART (USB-to-serial bridge) | your computer | flashing and the log console (UART0, 115200) |
| Native USB (GPIO19/20) | the KVM / target computer | the emulated keyboard (TinyUSB) |

Keystrokes only come out of the **native** port. The UART port can be left
unplugged in normal use, as long as the board is powered from the native port.
Some S3 boards need a `USB-OTG` / `IN-OUT` solder jumper bridged for that.

The console is on UART0 only. USB-Serial-JTAG is disabled because TinyUSB
takes over the native port.

## Toolchain

ESP-IDF **v6.1**, installed at `~/esp/esp-idf`. `esp_tinyusb` is pulled
by the component manager, and `dependencies.lock` pins the resolved version
(currently 3.0.0), so commit that file.

One-time setup on a new machine:

```
git clone -b v6.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
~/esp/esp-idf/install.sh esp32s3
python3 ~/esp/esp-idf/tools/idf_tools.py install cmake ninja
```

## Build and flash

```
. ~/esp/esp-idf/export.sh
idf.py set-target esp32s3     # first time only
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Use the serial device for the board's **UART** port. Depending on the bridge
chip, that is `/dev/ttyUSB0` or `/dev/ttyACM0`. Exit the monitor with
`Ctrl+]`.

USB self-test (off by default): enabling *Bluetooth to HID bridge* → *Type a
test string over USB at boot* makes the S3 type `esp32s3 usb keyboard ok `
once, 5 s after the host configures it, with no Bluetooth involved. It sends
no Enter. Use it to check a KVM or host on its own.

Verbose per-report logging (off by default): `idf.py menuconfig` → *Component config* →
*BLE keyboard* → *Log every received keyboard report as hex*.

## Pairing

With no stored bond, the S3 scans for anything advertising the HID service or
a keyboard appearance, and pairs with the first one it finds. Put the keyboard
in pairing mode first. For multi-device keyboards, pick a free channel.

If the keyboard asks for a passkey, the console prints a highlighted line:

```
W (…) ble_keyboard:   PAIRING: type 123456 on the keyboard, then Enter
```

Type those digits on the keyboard and press Enter. "Just works" keyboards
pair without a prompt. The bond is saved in NVS, and after that the S3 only
ever connects to that keyboard.

If the keyboard has a Boot Keyboard Input Report (0x2A22), the S3 subscribes
to it. Otherwise the S3 uses report mode:
- It reads the keyboard's Report Map to find the keyboard report and its layout.
- It subscribes to that report and converts each one to the 8-byte boot format.

Both 6-key keycode arrays and NKRO bitmaps (for example ZMK with NKRO
enabled) are handled. Media keys and other non-keyboard reports are ignored.

The bridge uses its own random static Bluetooth address, saved in NVS, rather
than the chip's fixed one. Resetting bonds (below) also picks a new address,
so the keyboard sees a new device instead of refusing to re-pair with an
address it still holds keys for.

## Resetting bonds

While the bridge is running, hold **BOOT** (GPIO0) for 3 seconds. The console
logs the countdown, then the S3 restarts with its bonds erased and a new
Bluetooth address, and goes back into pairing mode. Release BOOT early to
cancel. Without a reset, the bridge only ever reconnects to its bonded
keyboard.

Don't hold BOOT while powering on or pressing RESET. GPIO0 is a strapping
pin, so that boots the chip into its ROM download mode instead of this
firmware.

You can also wipe everything, firmware settings and bonds alike, with
`idf.py -p /dev/ttyACM0 erase-flash` followed by a fresh flash.

**Always reset both sides together.** The keyboard keeps its bond for the
bridge's old address. ZMK won't pair a new device on a profile that's already
bonded, so clear that profile (`&bt BT_CLR`) or switch to a free one, then
let the bridge pair. Re-pairing after clearing only one side has led to bonds
that work until the bridge's next reset, then fail to reconnect.

## Notes

- **Reconnect and keyboard security requests:** many keyboards (ZMK included)
  send a Security Request the moment they connect, and NimBLE starts
  encryption on its own in response. Calling `ble_gap_security_initiate()`
  while that's in flight makes NimBLE drop its own procedure, so the "link
  encrypted" event never arrives. `ble_keyboard` checks for a running
  procedure first (see the comment on `ble_sm_num_procs` in
  `ble_keyboard.c`).
- **Key repeat:** the host repeats a key for as long as reports say it's held.
  A key that doesn't repeat is usually a hold-tap in the keyboard's keymap.
