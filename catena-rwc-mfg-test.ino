/*

Module:  catena-rwc-mfg-test.ino

Function:
        Demonstration of RWC 5020A MFG test with Catena

Copyright notice:
        See accompanying license file.

Author:
        Terry Moore, MCCI Corporation	June 2018

*/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>
#include <cstring>

#define TX_INTERVAL 2000        // milliseconds
#define RX_RSSI_INTERVAL 100    // milliseconds

// Pin mapping for Adafruit Feather M0 LoRa, etc.
#if defined(ARDUINO_SAMD_FEATHER_M0)
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_CATENA_4551)
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = { 25,    // DIO0 (IRQ) is D25
                 26,    // DIO1 is D26
                 27,    // DIO2 is D27
               },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000     // 8MHz
};
#else
# error "Unknown target"
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// this gets callled by the library but we choose not to display any info;
// and no action is required.
void onEvent (ev_t ev) {
}

extern "C" {
void lmic_printf(const char *fmt, ...);
};

void lmic_printf(const char *fmt, ...) 
        {
        if (!Serial.dtr())
                return;

        char buf[256];
        va_list ap;

        va_start(ap, fmt);
        (void)vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
        va_end(ap);

        // in case we overflowed:
        buf[sizeof(buf) - 1] = '\0';
        if (Serial.dtr()) Serial.print(buf);
        }

osjob_t txjob;
osjob_t timeoutjob;
static void txdone_func(osjob_t* job);
static void rxdone_func(osjob_t* job);
static void timerdone_func(osjob_t *job);

class Fsm
	{
public:
	enum class State 
		{
		NoChange = 0,
		Initial,
		InitialDelay,
		SendStart,
		RxPackets,
                RxClose,
		SendReport1,
		SendReport2,
		SendReport3,
		Idle
		};

	Fsm() : state(State::Initial) {};

	State state;
	bool fActive;
	bool fEvent;
	bool fTimerDone;
	bool fTxDone;
	bool fRxDone;

	void update(void);

	void evTimerDone(void)
		{
		this->fTimerDone = true;
		this->update();
		}

	void evTxDone(void)
		{
		this->fTxDone = true;
		this->update();
		}

	void evRxDone(void)
		{
		++this->msgCount;
		size_t nMsg = LMIC.dataLen;
		if (nMsg > sizeof(this->rxMsg))
			nMsg = sizeof(this->rxMsg);
		std::memcpy(this->rxMsg, LMIC.frame, nMsg);

		this->fRxDone = true;
		this->update();
		}

	void startTimer(uint32_t ms)
		{
		fTimerDone = false;
		os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(ms), timerdone_func);
		}

	void tx(const uint8_t *pBuf, size_t nBuf);
	void rx(void);

        enum    {
                INITIAL_DELAY = 2000,
                RXTIMEOUT_DELAY = 1000,
                RXCLOSE_DELAY = 500,
                REPORT_DELAY = 1000,
                };
private:
	State evaluate(bool);
	uint8_t reportMsg[2];
	uint16_t msgCount;
	uint8_t rxMsg[64];
	} fsm;

static void timerdone_func(osjob_t *job)
	{
	fsm.evTimerDone();
	}

void Fsm::update()
	{
	if (this->fActive)
		{
		this->fEvent = true;
		return;
		}

	this->fActive = true;
	State newState;

	this->fEvent = false;
	newState = this->evaluate(false);

	while (true)
		{
		if (newState == State::NoChange)
			break;

		this->state = newState;
		this->fEvent = false;
		newState = this->evaluate(true);
		}

	this->fActive = false;
	}

Fsm::State Fsm::evaluate(bool fEntry)
	{
	State result;

	result = State::NoChange;

	switch (this->state)
		{
	case State::Initial:
                lmic_printf("Initial\n");
		result = State::InitialDelay;
		break;

	case State::InitialDelay:
		if (fEntry)
                        {
                        lmic_printf("Initial Delay\n");
			this->startTimer(INITIAL_DELAY);
                        }
		if (this->fTimerDone)
			result = State::SendStart;
		break;

	case State::SendStart:
		static const char startFlag_msg[] = "\xFF" "\xFF" "Hello world";

		if (fEntry)
			{
			lmic_printf("send start message\n");
			tx((const uint8_t *)startFlag_msg, sizeof(startFlag_msg));
			}
		if (this->fTxDone)
			result = State::RxPackets;
		break;

	case State::RxPackets:
		if (fEntry)
			{
			lmic_printf("start rx\n");
			this->rx();
			this->startTimer(RXTIMEOUT_DELAY);
			}
		else if (this->fRxDone)
			{
			if (this->rxMsg[0] == 0xFF && this->rxMsg[1] == 0xFF)
                                {
				/* don't restart RX, it's a close */
                                --this->msgCount;
                                result = State::RxClose;
                                }
			else
                                {
				this->rx();
        			this->startTimer(RXTIMEOUT_DELAY);
	        		}
                        }
		else if (this->fTimerDone)
			result = State::SendReport1;
		break;

        case State::RxClose:
                if (fEntry)
                        {
                        this->startTimer(RXCLOSE_DELAY);
                        }
                if (this->fTimerDone)
                        result = State::SendReport1;
                break;

	case State::SendReport1:
		if (fEntry)
			{
			lmic_printf("send report 1 (numRx=%u)\n", this->msgCount);

			this->reportMsg[0] = uint8_t(this->msgCount & 0xFF);
			this->reportMsg[1] = uint8_t(this->msgCount >> 8);
			this->tx(this->reportMsg, sizeof(this->reportMsg));
			this->startTimer(REPORT_DELAY);
			}
		if (this->fTimerDone)
			result = State::SendReport2;
		break;

	case State::SendReport2:
		if (fEntry)
			{
                        lmic_printf("send report 2 (numRx=%u)\n", this->msgCount);
                        this->tx(this->reportMsg, sizeof(this->reportMsg));
			this->startTimer(REPORT_DELAY);
			}
		if (this->fTimerDone)
			result = State::SendReport3;
		break;

	case State::SendReport3:
		if (fEntry)
			{
                        lmic_printf("send report 3 (numRx=%u)\n", this->msgCount);
                        this->tx(this->reportMsg, sizeof(this->reportMsg));
			this->startTimer(REPORT_DELAY);
			}
		if (this->fTimerDone)
			result = State::Idle;
		break;

	case State::Idle:
		if (fEntry)
			lmic_printf("done!\n");
		break;

	default:
		break;
		}

	return result;
	}

// Transmit the given string
void Fsm::tx(const uint8_t *pBuf, size_t nBuf)
	{
	// the radio is probably in RX mode; stop it.
	os_radio(RADIO_RST);

	// wait a bit so the radio can come out of RX mode
	delay(1);

	// prepare data
        if (nBuf > sizeof(LMIC.frame))
                nBuf = sizeof(LMIC.frame);

	std::memcpy(LMIC.frame, pBuf, nBuf);
	LMIC.dataLen = static_cast<uint8_t>(nBuf);

	// set completion function.
	LMIC.osjob.func = txdone_func;
	this->fTxDone = false;

	// start the transmission
	os_radio(RADIO_TX);
	}

static void txdone_func(osjob_t *job)
	{
	fsm.evTxDone();
	}

// Enable rx mode.
void Fsm::rx()
	{
	LMIC.osjob.func = rxdone_func;
	LMIC.rxtime = os_getTime(); // RX _now_
	// Enable "continuous" RX (e.g. without a timeout, still stops after
	// receiving a packet)
	this->fRxDone = false;
	os_radio(RADIO_RXON);
	}

static void rxdone_func(osjob_t *job)
	{
	fsm.evRxDone();
	}


// application entry point
void setup()
	{
	// delay(3000) makes recovery from botched images much easier, as it
	// gives the host time to break in to start a download. Without it,
	// you get to the crash before the host can break in.
	delay(3000);

	// even after the delay, we wait for the host to open the port. operator
	// bool(Serial) just checks dtr(), and it tosses in a 10ms delay.
	while (!Serial.dtr())
		/* wait for the PC */;

	Serial.begin(115200);
	Serial.println("Starting");

	pinMode(LED_BUILTIN, OUTPUT);

	// initialize runtime env
	os_init();

	// Set up these settings once, and use them for both TX and RX
#ifdef ARDUINO_ARCH_STM32
	LMIC_setClockError(10 * 65536 / 100);
#endif

#if defined(CFG_eu868)
	// Use a frequency in the g3 which allows 10% duty cycling.
	LMIC.freq = 869525000;
	// Use a medium spread factor. This can be increased up to SF12 for
	// better range, but then, the interval should be (significantly)
	// raised to comply with duty cycle limits as well.
	LMIC.datarate = DR_SF9;
	// Maximum TX power
	LMIC.txpow = 27;
#elif defined(CFG_us915)
	// make it easier for test, by pull the parameters up to the top of the
	// block. Ideally, we'd use the serial port to drive this; or have
	// a voting protocol where one side is elected the controller and
	// guides the responder through all the channels, powers, ramps
	// the transmit power from min to max, and measures the RSSI and SNR.
	// Even more amazing would be a scheme where the controller could
	// handle multiple nodes; in that case we'd have a way to do
	// production test and qualification. However, using an RWC5020A
	// is a much better use of development time.

	// set fDownlink true to use a downlink channel; false
	// to use an uplink channel. Generally speaking, uplink
	// is more interesting, because you can prove that gateways
	// *should* be able to hear you.
	const static bool fDownlink = false;

	// the downlink channel to be used.
	const static uint8_t kDownlinkChannel = 3;

	// the uplink channel to be used.
	const static uint8_t kUplinkChannel = 0;

	// this is automatically set to the proper bandwidth in kHz,
	// based on the selected channel.
	uint32_t uBandwidth;

	if (!fDownlink)
		{
		if (kUplinkChannel < 64)
			{
			LMIC.freq = US915_125kHz_UPFBASE +
				    kUplinkChannel * US915_125kHz_UPFSTEP;
			uBandwidth = 125;
			}
		else
			{
			LMIC.freq = US915_500kHz_UPFBASE +
				    (kUplinkChannel - 64) * US915_500kHz_UPFSTEP;
			uBandwidth = 500;
			}
		}
	else
		{
		// downlink channel
		LMIC.freq = US915_500kHz_DNFBASE +
			    kDownlinkChannel * US915_500kHz_DNFSTEP;
		uBandwidth = 500;
		}

	// Use a suitable spreading factor
	if (uBandwidth < 500)
		LMIC.datarate = US915_DR_SF7; // DR4
	else
		LMIC.datarate = US915_DR_SF12CR; // DR8

	// default tx power for US: 21 dBm
	LMIC.txpow = 21;
#elif defined(CFG_au921)
	// make it easier for test, by pull the parameters up to the top of the
	// block. Ideally, we'd use the serial port to drive this; or have
	// a voting protocol where one side is elected the controller and
	// guides the responder through all the channels, powers, ramps
	// the transmit power from min to max, and measures the RSSI and SNR.
	// Even more amazing would be a scheme where the controller could
	// handle multiple nodes; in that case we'd have a way to do
	// production test and qualification. However, using an RWC5020A
	// is a much better use of development time.

	// set fDownlink true to use a downlink channel; false
	// to use an uplink channel. Generally speaking, uplink
	// is more interesting, because you can prove that gateways
	// *should* be able to hear you.
	const static bool fDownlink = false;

	// the downlink channel to be used.
	const static uint8_t kDownlinkChannel = 3;

	// the uplink channel to be used.
	const static uint8_t kUplinkChannel = 8 + 3;

	// this is automatically set to the proper bandwidth in kHz,
	// based on the selected channel.
	uint32_t uBandwidth;

	if (!fDownlink)
		{
		if (kUplinkChannel < 64)
			{
			LMIC.freq = AU921_125kHz_UPFBASE +
				    kUplinkChannel * AU921_125kHz_UPFSTEP;
			uBandwidth = 125;
			}
		else
			{
			LMIC.freq = AU921_500kHz_UPFBASE +
				    (kUplinkChannel - 64) * AU921_500kHz_UPFSTEP;
			uBandwidth = 500;
			}
		}
	else
		{
		// downlink channel
		LMIC.freq = AU921_500kHz_DNFBASE +
			    kDownlinkChannel * AU921_500kHz_DNFSTEP;
		uBandwidth = 500;
		}

	// Use a suitable spreading factor
	if (uBandwidth < 500)
		LMIC.datarate = AU921_DR_SF7; // DR4
	else
		LMIC.datarate = AU921_DR_SF12CR; // DR8

	// default tx power for AU: 30 dBm
	LMIC.txpow = 30;
#elif defined(CFG_as923)
	// make it easier for test, by pull the parameters up to the top of the
	// block. Ideally, we'd use the serial port to drive this; or have
	// a voting protocol where one side is elected the controller and
	// guides the responder through all the channels, powers, ramps
	// the transmit power from min to max, and measures the RSSI and SNR.
	// Even more amazing would be a scheme where the controller could
	// handle multiple nodes; in that case we'd have a way to do
	// production test and qualification. However, using an RWC5020A
	// is a much better use of development time.
	const static uint8_t kChannel = 0;
	uint32_t uBandwidth;

	LMIC.freq = AS923_F1 + kChannel * 200000;
	uBandwidth = 125;

	// Use a suitable spreading factor
	if (uBandwidth == 125)
		LMIC.datarate = AS923_DR_SF7; // DR7
	else
		LMIC.datarate = AS923_DR_SF7B; // DR8

	// default tx power for AS: 21 dBm
	LMIC.txpow = 16;

	if (LMIC_COUNTRY_CODE == LMIC_COUNTRY_CODE_JP)
		{
		LMIC.lbt_ticks = us2osticks(AS923JP_LBT_US);
		LMIC.lbt_dbmax = AS923JP_LBT_DB_MAX;
		}
#elif defined(CFG_in866)
	// make it easier for test, by pull the parameters up to the top of the
	// block. Ideally, we'd use the serial port to drive this; or have
	// a voting protocol where one side is elected the controller and
	// guides the responder through all the channels, powers, ramps
	// the transmit power from min to max, and measures the RSSI and SNR.
	// Even more amazing would be a scheme where the controller could
	// handle multiple nodes; in that case we'd have a way to do
	// production test and qualification. However, using an RWC5020A
	// is a much better use of development time.
	const static uint8_t kChannel = 0;
	uint32_t uBandwidth;

	LMIC.freq = IN866_F1 + kChannel * 200000;
	uBandwidth = 125;

	LMIC.datarate = IN866_DR_SF7; // DR7
	// default tx power for IN: 30 dBm
	LMIC.txpow = IN866_TX_EIRP_MAX_DBM;
#else
# error Unsupported LMIC regional configuration.
#endif

	// This sets CR 4/5, BW from LMIC.datarate
	LMIC.rps = updr2rps(LMIC.datarate);

	lmic_printf("Starting transmission test:\n");
	lmic_printf("Frequency: %d.%d MHz LMIC.datarate: %u LMIC.txpow: %d\n",
		    LMIC.freq / 1000000, (LMIC.freq / 100000) % 10,
		    LMIC.datarate,
		    LMIC.txpow
		    );

	// setup initial job
	fsm.update();
	}

void loop() 
	{
	// execute scheduled jobs and events
	os_runloop_once();
	}
