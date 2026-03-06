#include "controller.h"
#include "controller_util.h"
#include "display_control.h"
#include "drivers/pic/pic.h"
#include "hardware_events.h"
#include "tusb.h"
#include "usb_audio.h"
#include "usb_midi.h"
#include "usb_serial.h"

extern "C" {
#include "RTT/SEGGER_RTT.h"
#include "RZA1/cpu_specific.h"
#include "RZA1/gpio/gpio.h"
#include "RZA1/intc/devdrv_intc.h"
#include "drivers/oled/oled_low_level.h"
#include "RZA1/rspi/rspi.h"
#include "RZA1/ssi/ssi.h"
#include "RZA1/system/iobitmasks/usb_iobitmask.h"
#include "RZA1/system/iodefine.h"
#include "RZA1/uart/sio_char.h"
#include "definitions.h"
#include "drivers/ssi/ssi.h"
#include "drivers/uart/uart.h"
#include "util/cfunctions.h"

// TinyUSB interrupt handler (from tinyusb)
void dcd_int_handler(uint8_t rhport);

// USB interrupt handler wrapper
void usb_interrupt_handler(uint32_t int_sense) {
	(void)int_sense;
	dcd_int_handler(0); // rhport 0
}

// External functions from OLED driver (oled_init.c is C)
extern void oledDMAInit(void);
extern void setup_oled(void);
extern void oledRoutine(void); // Process OLED transfers

}

// ---------------------------------------------------------------------------
// SSI0 / audio-clock pin mux assignments (RZA1L hardware manual, Table 54-7)
// ---------------------------------------------------------------------------
static constexpr struct { uint8_t port, pin, mux; } kSSIPins[] = {
	{7, 11, 6}, // AUDIO_XOUT  — master clock output to codec
	{6,  9, 3}, // SSISCK0     — SSI0 BCLK (bit clock)
	{6, 10, 3}, // SSIWS0      — SSI0 LRCK (word select)
	{6,  8, 3}, // SSITXD0     — SSI0 TX data (to DAC)
	{6, 11, 3}, // SSIRXD0     — SSI0 RX data (from ADC)
};

int32_t controller_main(void) {
	// Initialize RTT for debug output
	// Use NO_BLOCK_SKIP so RTT never stalls the main loop — critical for
	// real-time audio.  BLOCK_IF_FIFO_FULL caused a death spiral where
	// underrun log messages blocked the loop, causing more underruns.
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
	CDBG_STR("\n=== Deluge USB Controller Starting ===\n");

	// Assume OLED is always present in controller mode
	CDBG_STR("Configuring PIC32...\n");

	// Configure PIC32 for pad/button scanning
	PIC::enableOLED();               // Enable OLED (PIC command 247)
	PIC::setDebounce(5);             // Set debounce time to 20ms (value * 4)
	PIC::setRefreshTime(23);         // Set pad refresh time to 23ms
	PIC::setMinInterruptInterval(8); // Set min interrupt interval to 8ms
	PIC::setFlashLength(6);          // Set flash length to 6ms
	PIC::setUARTSpeed();             // Set UART speed to high speed (200kHz)
	PIC::flush();

	// Give PIC time to switch baud rates
	CDBG_STR("Waiting for PIC32 to switch to 200kHz...\n");
	delay_ms(50);

	// Now switch OUR side to 200kHz to match the PIC
	PIC::setupForPads(); // This calls uartSetBaudRate(UART_CHANNEL_PIC, 200000)
	CDBG_STR("Host UART switched to 200kHz\n");

	// Request PIC firmware version and button states (important for PIC initialization)
	CDBG_STR("Requesting PIC firmware version and button states...\n");
	PIC::requestFirmwareVersion();
	PIC::resendButtonStates();
	PIC::flush();
	delay_ms(50); // Give PIC time to process and respond

	// Drain any responses from PIC (firmware version, button states, etc.)
	int drain_count = 0;
	PIC::Response response;
	while ((response = PIC::read()) != PIC::Response::NONE && drain_count < 100) {
		drain_count++;
	}
	CDBG_FMT("Drained %d bytes from PIC\n", drain_count);

	CDBG_STR("Setting up GPIO pins...\n");
	// GPIO setup for audio codec and peripherals
	// Hold codec in power-down/reset until SSI clocks are stable
	setPinAsOutput(CODEC.port, CODEC.pin);
	setOutputState(CODEC.port, CODEC.pin, 0); // Keep codec off for now

	setPinAsOutput(SPEAKER_ENABLE.port, SPEAKER_ENABLE.pin);
	setOutputState(SPEAKER_ENABLE.port, SPEAKER_ENABLE.pin, 0); // Speaker off initially

	// Battery and sync LEDs
	setOutputState(BATTERY_LED.port, BATTERY_LED.pin, 1); // Off (open-drain)
	setPinAsOutput(BATTERY_LED.port, BATTERY_LED.pin);

	setOutputState(SYNCED_LED.port, SYNCED_LED.pin, 0); // Off
	setPinAsOutput(SYNCED_LED.port, SYNCED_LED.pin);

	// Audio input detection pins
	setPinAsInput(HEADPHONE_DETECT.port, HEADPHONE_DETECT.pin);
	setPinAsInput(LINE_IN_DETECT.port, LINE_IN_DETECT.pin);
	setPinAsInput(MIC_DETECT.port, MIC_DETECT.pin);
	setPinAsInput(LINE_OUT_DETECT_L.port, LINE_OUT_DETECT_L.pin);
	setPinAsInput(LINE_OUT_DETECT_R.port, LINE_OUT_DETECT_R.pin);

	// Analog voltage sense
	setPinMux(VOLT_SENSE.port, VOLT_SENSE.pin, 1);

	CDBG_STR("Setting up SPI for CV/OLED...\n");
	// Setup SPI for CV/OLED (OLED always present, limited to 10MHz)
	R_RSPI_Create(SPI_CHANNEL_CV, 10000000, 0, 32);
	R_RSPI_Start(SPI_CHANNEL_CV);
	setPinMux(SPI_CLK.port, SPI_CLK.pin, 3);   // CLK
	setPinMux(SPI_MOSI.port, SPI_MOSI.pin, 3); // MOSI

	// OLED shares SPI - manually control SSL pin
	setOutputState(SPI_SSL.port, SPI_SSL.pin, 1);
	setPinAsOutput(SPI_SSL.port, SPI_SSL.pin);

	CDBG_STR("Initializing OLED...\n");
	setupSPIInterrupts();
	oledDMAInit();
	setup_oled();

	CDBG_STR("Initializing CV DAC and gate outputs...\n");
	cv_gate_init();

	// Drain PIC responses from OLED init
	// The PIC echoes back: SET_DC_LOW, ENABLE_OLED, SELECT_OLED, SET_DC_HIGH (from init), DESELECT_OLED
	CDBG_STR("Draining OLED init responses...\n");
	delay_ms(10); // Let responses arrive
	int oled_drain = 0;
	PIC::Response oled_resp;
	while ((oled_resp = PIC::read()) != PIC::Response::NONE && oled_drain < 500) {
		oled_drain++;
	}
	CDBG_FMT("Drained %d OLED responses\n", oled_drain);

	CDBG_STR("Setting up SSI audio pins...\n");
	// Configure SSI0 pins using the named pin table at the top of this file
	for (auto& p : kSSIPins) {
		setPinMux(p.port, p.pin, p.mux);
	}

	CDBG_STR("Initializing SSI audio...\n");
	// Initialize SSI0 with DMA for audio (44.1kHz stereo I2S)
	ssiInit(0, 1);

	// Now that SSI clocks are running and stable, enable the codec
	// The codec needs stable I2S clocks present at power-on to initialize correctly
	CDBG_STR("Enabling audio codec...\n");
	delay_ms(5); // Let SSI clocks stabilize
	setOutputState(CODEC.port, CODEC.pin, 1); // Power on codec
	delay_ms(10); // Let codec initialize with stable clocks

	CDBG_STR("Setting up USB...\n");
	// Enable USB0 module clock (STBCR7 bit 1 = 0)
	CPG.STBCR7 &= 0xFD;
	volatile uint8_t dummy_read = CPG.STBCR7; // Dummy read for write completion
	(void)dummy_read;

	// Register USB interrupt handler BEFORE initializing TinyUSB
	// TinyUSB will enable the interrupt via dcd_int_enable(), so we just register and set priority
	CDBG_STR("Registering USB interrupt handler...\n");
	R_INTC_Disable(INTC_ID_USBI0);
	R_INTC_RegistIntFunc(INTC_ID_USBI0, usb_interrupt_handler);
	R_INTC_SetPriority(INTC_ID_USBI0, 9);
	// Note: TinyUSB's tud_init() will call dcd_int_enable() to enable the interrupt
	CDBG_STR("USB interrupt handler registered (will be enabled by TinyUSB)\n");

	// Initialize TinyUSB device stack
	// dcd_init() (called by tud_init) enables USBE, waits 10ms, then sets DPRPU=1
	// if VBUS is present. The host may start enumeration within milliseconds.
	CDBG_STR("Initializing TinyUSB...\n");
	tud_init(0);
	tud_connect();

	// RZA1L hardware quirk: INTENB0/BEMPENB/BRDYENB registers are NOT writable
	// immediately after USBE=1, for some reason.
	// CRITICAL: Do NOT clear INTSTS0 — pending flags must survive so they
	// trigger an interrupt the moment INTENB0 is armed.
	for (int i = 0; i < 5; i++) {
		tud_task();   // process USB events by polling
		delay_ms(5);
	}

	// Now set interrupt enables (registers should be writable after ~25 ms)
	USB200.INTENB0 = USB_INTENB0_VBSE  // VBus interrupt
	               | USB_INTENB0_BRDYE // Buffer Ready
	               | USB_INTENB0_BEMPE // Buffer Empty
	               | USB_INTENB0_DVSE  // Device State change
	               | USB_INTENB0_CTRE  // Control Transfer Stage Transition
	               | USB_INTENB0_RSME; // Resume
	USB200.BEMPENB = 1;
	USB200.BRDYENB = 1;

	// Read-back verify
	uint16_t intenb0_rb = USB200.INTENB0;
	CDBG_FMT("INTENB0 write verify: 0x%04X\n", intenb0_rb);

	if (intenb0_rb == 0) {
		// Retry with longer settle time
		CDBG_STR("INTENB0 write failed \u2014 retrying with longer delay...\n");
		for (int i = 0; i < 20; i++) {
			tud_task();
			delay_ms(10);
		}
		USB200.INTENB0 = USB_INTENB0_VBSE | USB_INTENB0_BRDYE | USB_INTENB0_BEMPE
		               | USB_INTENB0_DVSE | USB_INTENB0_CTRE  | USB_INTENB0_RSME;
		USB200.BEMPENB = 1;
		USB200.BRDYENB = 1;
		intenb0_rb = USB200.INTENB0;
		CDBG_FMT("INTENB0 retry verify: 0x%04X\n", intenb0_rb);
	}

	// Verify USB interrupt is enabled in GIC
	{
		volatile uint32_t* icdiser = (volatile uint32_t*)&INTC.ICDISER0;
		uint32_t mask = 1u << (73 & 0x1f);
		uint32_t enabled = (*(icdiser + (73 >> 5))) & mask;
		CDBG_FMT("USB GIC int enabled: %s\n", enabled ? "YES" : "NO");
		CDBG_FMT("SYSCFG0=0x%04X INTSTS0=0x%04X INTENB0=0x%04X\n",
		         USB200.SYSCFG0, USB200.INTSTS0, USB200.INTENB0);
	}

	CDBG_STR("Initializing hardware event scanning...\n");
	// Initialize hardware event scanning (will configure PIC for pad reading)
	hardware_events_init();

	CDBG_STR("Initializing USB subsystems...\n");
	// Initialize USB serial protocol
	usb_serial_init();

	// Initialize USB audio for audio I/O
	usb_audio_init();

	// Initialize USB MIDI for MIDI I/O
	usb_midi_init();

	CDBG_STR("Clearing all LEDs and display...\n");

	// Clear all pad LEDs
	pad_led_clear_all();

	// Turn off all indicator LEDs
	for (int i = 0; i < 36; i++) {
		hardware_led_set(i, false);
	}

	// Ensure all LED commands are sent
	uartFlushIfNotSending(UART_ITEM_PIC);
	delay_ms(50);

	// Clear OLED display
	display_clear();

	CDBG_STR("All outputs cleared\n");
	CDBG_STR("=== Controller initialization complete, entering main loop ===\n");

	// Main loop
	while (true) {
		// Process OLED transfer queue (handles select/deselect/DMA)
		oledRoutine();

		// Flush PIC UART if needed
		uartFlushIfNotSending(UART_ITEM_PIC);

		// Process USB tasks (handles non-interrupt USB events)
		tud_task();

		// Scan hardware for events
		hardware_events_scan();

		// Send any pending events over USB serial
		usb_serial_task();

		// Drain any pad LED column pairs left pending by backpressure in pad_led_flush_dirty().
		// Called here (not only inside usb_serial_task) so dirty pairs are sent as soon as the
		// PIC UART ring buffer has room, even when no USB message arrives this iteration.
		pad_led_flush_dirty();

		// Process USB again before audio — minimise latency for set_itf responses
		tud_task();

		// Handle USB audio streaming
		usb_audio_task();

		// Handle USB MIDI I/O
		usb_midi_task();


	}

	return 0;
}

void midiAndGateTimerGoneOff(void) {
	// Intentional no-op in controller mode.
	// The controller firmware does not use the RZA1 MTU-based MIDI/gate timer;
	// all MIDI output is handled through the USB-MIDI task in the main loop.
	// This symbol must exist because it is referenced by the interrupt
	// dispatch table inherited from the shared RZA1 startup code.
}
