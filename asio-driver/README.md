# Deluge ASIO Driver

A Windows ASIO driver for the Synthstrom Deluge that bypasses `usbaudio2.sys` and handles
USB Audio Class 2.0 isochronous transfers directly via libusbK. This enables **asynchronous
mode with implicit feedback** — the spec-correct approach for fixed-clock devices — which the
standard Windows USB audio driver does not support.

## Why?

The Deluge's audio codec runs from a fixed crystal oscillator. The USB spec says such devices
should declare their endpoints as **asynchronous** and provide a feedback mechanism so the host
can match the device's actual sample rate. The Deluge uses **implicit feedback** — the microphone
IN endpoint, clocked by the same crystal, tells the host the true rate (USB 2.0 §5.12.4.2).

Linux and macOS support implicit feedback natively. Windows' built-in `usbaudio2.sys` does not.
This ASIO driver fills that gap by:

1. Using **libusbK** for direct USB isochronous I/O (no kernel driver development needed)
2. Reading the mic IN endpoint to derive the device's actual sample rate (implicit feedback)
3. Adjusting speaker OUT packet sizes to match, preventing buffer drift
4. Presenting a standard **ASIO** interface to any DAW (Ableton, FL Studio, Reaper, etc.)

## Prerequisites

- Windows 10 or later (x64)
- CMake 3.20+
- Visual Studio 2022 (or Build Tools) with C++ desktop workload
- [libusbK SDK](https://github.com/mcuee/libusbk/releases) installed

## Building

```powershell
cmake -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

## Installation

1. Use [Zadig](https://zadig.akeo.ie/) to install the **libusbK** driver for the Deluge's audio interface (Deluge Audio).
   - The CDC (serial) and MIDI interfaces should remain on their default drivers.

2. Register the ASIO driver:
   ```powershell
   regsvr32 build\Release\DelugeASIO.dll
   ```

3. Select "Deluge ASIO" in your DAW's audio settings.

## Uninstallation

```powershell
regsvr32 /u build\Release\DelugeASIO.dll
```

## Architecture

```
DelugeASIO.dll (COM DLL)
  ├── ASIO Interface (asio_driver.cpp)
  │   └── Implements IASIO COM object
  │       - createBuffers / disposeBuffers
  │       - start / stop
  │       - bufferSwitch callback on audio thread
  │
  ├── USB Transport (usb_transport.cpp)
  │   └── libusbK isochronous I/O
  │       - ISO OUT: speaker audio (PC → Deluge)
  │       - ISO IN:  microphone audio (Deluge → PC) + implicit feedback
  │       - Rate adaptation from IN endpoint timing
  │
  └── COM Registration (dll_main.cpp)
      └── DllRegisterServer / DllUnregisterServer
          - CLSID in HKCR\CLSID
          - ASIO entry in HKLM\SOFTWARE\ASIO
```

## License

Same as the Deluge firmware (GPL-3.0).
