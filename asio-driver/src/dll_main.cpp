// DelugeASIO — DLL Entry Point and COM Factory
// Provides the CFactoryTemplate and getDriver() that the SDK's dllentry.cpp needs.
//
// Licensed under GPL-3.0 (same as Deluge firmware and ASIO SDK GPL option).

#include <windows.h>
#include <objbase.h>

#include "asio_driver.h"

// The SDK's dllentry.cpp defines DllEntryPoint; the CRT needs DllMain.
extern BOOL WINAPI DllEntryPoint(HINSTANCE, ULONG, LPVOID);

extern "C" BOOL WINAPI DllMain(HINSTANCE hInstance, ULONG ulReason, LPVOID pv) {
    return DllEntryPoint(hInstance, ulReason, pv);
}

// ----------------------------------------------------------------------------
// Factory template — the SDK's dllentry.cpp uses this to create instances
// ----------------------------------------------------------------------------

CFactoryTemplate g_Templates[] = {
    {
        L"Deluge ASIO",       // Name
        &CLSID_DelugeASIO,    // CLSID
        DelugeASIO::CreateInstance, // Factory function
        nullptr               // Init routine
    }
};

int g_cTemplates = sizeof(g_Templates) / sizeof(g_Templates[0]);

// ----------------------------------------------------------------------------
// COM DLL Registration
// ----------------------------------------------------------------------------

// Forward declare from SDK's register.cpp
LONG RegisterAsioDriver(CLSID, char*, char*, char*, char*);
LONG UnregisterAsioDriver(CLSID, char*, char*);

static const char kDllName[]  = "DelugeASIO.dll";
static const char kRegName[]  = "Deluge ASIO";
static const char kRegDesc[]  = "Deluge USB ASIO";
static const char kThreadModel[] = "Apartment";

extern "C" HRESULT STDAPICALLTYPE DllRegisterServer() {
    LONG rc = RegisterAsioDriver(
        CLSID_DelugeASIO,
        const_cast<char*>(kDllName),
        const_cast<char*>(kRegName),
        const_cast<char*>(kRegDesc),
        const_cast<char*>(kThreadModel)
    );
    return (rc == 0) ? S_OK : E_FAIL;
}

extern "C" HRESULT STDAPICALLTYPE DllUnregisterServer() {
    LONG rc = UnregisterAsioDriver(
        CLSID_DelugeASIO,
        const_cast<char*>(kDllName),
        const_cast<char*>(kRegName)
    );
    return (rc == 0) ? S_OK : E_FAIL;
}
