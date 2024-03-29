/*
 * StrainEffector-v2 (ATTiny44A processor)
 *
 * Created: 06/01/2019
 * Author : David Crocker, Escher Technologies Limited
 * License: GNU GPL version 3
 */ 

// 2019-11-07: Merged pull request from user dgrat to fix retention of custom settings

#include "ecv.h"

#ifdef __ECV__
#define __attribute__(_x)
#define __volatile__
#pragma ECV noverifyincludefiles
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/fuse.h>

#ifdef __ECV__
#pragma ECV verifyincludefiles
#undef cli
#undef sei
extern void cli();
extern void sei();
#endif

#if !defined(LINEAR_PCB)
# error LINEAR_PCB must be defined as 0 or 1
#endif

// Additional options supported by the Linear PCB only
#if LINEAR_PCB
constexpr uint16_t ThresholdMultiplier = 2;			// pretend the gain is this when calculating the threshold. Additional gain is 20 divided by this value.
# define CLOCK_500K			(0)
#else
constexpr uint16_t ThresholdMultiplier = 6;			// pretend the gain is this when calculating the threshold. Additional gain is 20 divided by this value.
# define CLOCK_500K			(0)
#endif

#define ISR_DEBUG			(0)

#define BITVAL(_x) static_cast<uint8_t>(1u << (_x))

// ATTINY44A pin assignments:
// PA0/AREF/ADC0		bridge voltage
// PA1/ADC1				strain sensor output
// PA2					unused
// PA3					smoothed version of ADC1, allowing us to use differential mode with gain
// PA4/SCK				input (Z_PROBE_MOD), also used for programming
// PA5/MISO				output (Z_PROBE_OUT), also used for programming
// PA6/MOSI				debug output, unused (except for programming) if debug disabled
// PA7					LED, high to turn on
// PB0					unused
// PB1					unused
// PB2/INT0				unused
// PB3/RESET			reset, used for programming

constexpr unsigned int PortADuetOutputBitNum = 5;
constexpr unsigned int PortADuetInputBitNum = 4;
constexpr unsigned int PinChange0InputBitNum = PortADuetInputBitNum;	// bit number in the Pin Change 0 register				
constexpr unsigned int PortALedBitNum = 7;				

#if ISR_DEBUG
constexpr unsigned int PortADebugBitNum = 6;					// use PA6/MOSI for debugging
constexpr uint8_t PortAUnusedBitMask = BITVAL(2);				// bit mask for unused port A pins so we can enable pullup resistors on them
#else
constexpr uint8_t PortAUnusedBitMask = BITVAL(2) | BITVAL(6);	// bit mask for unused port A pins so we can enable pullup resistors on them
#endif

constexpr uint8_t PortBUnusedBitMask = BITVAL(0) | BITVAL(1) | BITVAL(2);	// bit mask for unused port B pins so we can enable pullup resistors on them

constexpr unsigned int AdcBridgePowerInputChan = 0;				// ADC channel for the bridge power
constexpr unsigned int AdcStrainInputChan = 1;					// ADC channel for the strain amplifier output
constexpr unsigned int AdcSmoothedStrainInputChannel = 3;
constexpr uint8_t Didr0BitMask = BITVAL(AdcBridgePowerInputChan) | BITVAL(AdcStrainInputChan) | BITVAL(AdcSmoothedStrainInputChannel);		// bit mask of all used ADC inputs

// Approximate MPU frequency (8MHz internal RC oscillator)
constexpr uint32_t F_CPU = 8000000uL;

constexpr uint32_t fastTickFrequency = F_CPU/8;					// nominal TC0 clock rate 1MHz using prescaler = 8
constexpr uint16_t slowTickFrequency = fastTickFrequency/256;	// nominal timer 0 interrupt frequency, about 3906.25Hz (rounds down to 3906)

#if CLOCK_500K
constexpr uint16_t strainReadingsAveraged = 32;					// 32 readings @ 2 bytes each = 64 bytes
#else
constexpr uint16_t strainReadingsAveraged = 16;					// 16 readings @ 2 bytes each = 32 bytes
#endif

// The percentage imbalance between top and bottom trace resistance is defined as 100*(Rt-Rl)/(Rt+Rl). We plan to allow up to 4% from the manufacturer
constexpr float MaxImbalancePercentInTesting = 4.5;
constexpr float MaxImbalancePercentInNormalUse = 8.0;
constexpr uint16_t MaxImbalanceReadingInTesting = (uint16_t)((MaxImbalancePercentInTesting * 1024.0)/200.0 + 0.5);
constexpr uint16_t MaxImbalanceReadingInNormalUse = (uint16_t)((MaxImbalancePercentInNormalUse * 1024.0)/200.0 + 0.5);
static_assert(MaxImbalanceReadingInTesting == 23, "unexpected value");
static_assert(MaxImbalanceReadingInNormalUse == 41, "unexpected value");

constexpr uint16_t kickFrequency = 10;							// how often we kick the watchdog
constexpr uint16_t kickIntervalTicks = slowTickFrequency/kickFrequency;

constexpr unsigned int avShift = 12;							// a shift of 12 allows a 50% decay in 4096 cycles which is about 0.21 seconds
constexpr unsigned int defaultThresholdMilliVolts = 51;			// trigger threshold in mV (51 here corresponds to programming a value of 50)
constexpr uint16_t ledOnTicks = slowTickFrequency/10;			// leave LED on for 100ms
constexpr uint16_t ledFlashTicks = slowTickFrequency/4;			// flash LED on for 0.25s

constexpr unsigned int EdgeCaptureBufferEntries = 20;			// the size of our edge capture buffer, must be an even number
constexpr uint16_t BitsPerSecond = 1000;						// the nominal bit rate that the data is transmitted at
constexpr uint32_t NominalBitLength = fastTickFrequency/BitsPerSecond;
constexpr uint16_t MinBitLength = (NominalBitLength * 10)/13;	// allow 30% clock speed tolerance
constexpr uint16_t MaxBitLength = (NominalBitLength * 13)/10;	// allow 30% clock speed tolerance
constexpr uint16_t ErrorRecoveryDelayBits = 25;					// after an error we wait for the line to be low for this long
static_assert(NominalBitLength * ErrorRecoveryDelayBits < (1ul << 16), "Potential overflow in bit timing calculations");
constexpr uint16_t ErrorRecoveryTime = NominalBitLength * ErrorRecoveryDelayBits;

// Received commands
constexpr unsigned int MaxBytesReceived = 3;					// maximum number of received bytes in a programming message
constexpr uint8_t ProgByte = 105;								// magic byte to start programming custom sensitivity
constexpr uint8_t EraseByte = 131;								// magic byte to clear custom sensitivity
constexpr uint8_t FactoryResetByte = 65;						// magic byte to clear EEPROM
constexpr uint8_t ReceiveTestByte = 99;							// magic byte to report default or custom sensitivity
constexpr uint8_t NoiseMeasurementByte = 47;					// magic byte to measure the noise level
constexpr uint8_t NoisePassFailByte = 56;						// magic byte to measure the noise level and report pass/fail

constexpr uint8_t NoiseMeasurementFailThreshold = 20;			// noise level must be below this in pass/fail test
constexpr uint16_t NoiseMeasurementSeconds = 8;
constexpr uint16_t NoiseMeasurementTicks = (slowTickFrequency * NoiseMeasurementSeconds);

// LED flash codes
enum class FlashCode : uint8_t
{
	StartupOkNoNvData = 1,
	StartupOkStandardSensitivity = 2,
	StartupOkCustomSensitivity = 3,
	CustomSensitivitySet = 4,
	DefaultSensitivitySet = 5,
	BridgeOutputTooLow = 6,
	BridgeOutputTooHigh = 7,
	BadBridgeSupply = 9,			// version 2 hardware only
	VccTooLow = 9,					// version 3 and 4 hardware only
	VccTooHigh = 10					// version 3 and 4 hardware only
};

enum class NoiseMeasurementMode : uint8_t
{
	off = 0,
	reportPeak,
	reportPassFail
};

constexpr float Vref = 2.8;
constexpr float VccMin = 2.95;
constexpr float VccMax = 5.5;
constexpr uint16_t VccMaxAdcReading = (uint16_t)(1024 * strainReadingsAveraged * Vref / VccMax);
constexpr uint16_t VccMinAdcReading = (uint16_t)(1024 * strainReadingsAveraged * Vref / VccMin);

struct NvData
{
	uint16_t flags;
	uint16_t sensitivity;
	uint16_t checksum;
	static const uint16_t magic = 0x56A2;
	static const uint16_t FlagCustomSensitivity = 0x0001;

	bool IsValid() const;
};

bool NvData::IsValid() const { return (sensitivity ^ flags ^ checksum) == magic; }

NvData nvData;

#ifndef __ECV__
FUSES = {0xE2u, 0xDFu, 0xFFu};		// 8MHz RC clock
#endif

// Filter class. This maintains the sum of the last 'strainReadingsAveraged' values passed to it.
class Filter
{
public:
	uint16_t readings[strainReadingsAveraged];
	volatile uint16_t sum;
	uint8_t index;

	// Initialise the filter
	void Init(uint16_t initialValue)
	post(FilterInvariant());

	// Add a reading to the filter
	void AddReading(uint16_t adcVal)
	pre(FilterInvariant())
	post(FilterInvariant())
	{
		sum = sum - readings[index] + adcVal;
		readings[index] = adcVal;
		index = static_cast<uint8_t>((index + 1u) % strainReadingsAveraged);
	}

	ghost(
		bool FilterInvariant() const
		returns((forall r in readings :- r <= 1023) && sum == + over readings);
	)
};

void Filter::Init(uint16_t initialValue)
{
	for (uint16_t& r : readings)
	{
		r = initialValue;
	}
	index = 0;
	sum = initialValue * strainReadingsAveraged;
}

Filter strainFilter;
volatile uint16_t tickCounter = 0;
uint16_t lastKickTicks = 0;									// when we last kicked the watchdog
uint16_t timeOn = 0;										// when we last turned the LED on
volatile uint32_t rollingAverage = 0;
volatile uint16_t shiftedRollingAverage = 0;				// this is always equal to (rollingAverage >> avShift)

uint16_t threshold;											// this sets the sensitivity
bool signedAdc = false;

volatile uint16_t edgeCaptures[EdgeCaptureBufferEntries];
typedef uint8_t invariant(value <= EdgeCaptureBufferEntries) edgeCaptureIndex_t;
volatile edgeCaptureIndex_t numberOfEdgesCaptured = 0;

uint8_t bytesReceived[MaxBytesReceived];
typedef uint8_t invariant(value <= MaxBytesReceived) numBytesReceived_t;
numBytesReceived_t numBytesReceived = 0;
uint16_t noiseMeasurementStartTicks = 0;
uint16_t noiseLevel;
NoiseMeasurementMode noiseMeasureMode = NoiseMeasurementMode::off;

ghost(
	bool AverageInvariant() const
		returns(shiftedRollingAverage == rollingAverage >> avShift);
)

template<class X> inline X min(X _a, X _b)
{
	return (_a < _b) ? _a : _b;
}

template<class X> inline X max(X _a, X _b)
{
	return (_a > _b) ? _a : _b;
}

// EEPROM access functions
void readEEPROM(uint8_t ucAddress, uint8_t * array p, uint8_t len)
pre(len != 0)
{
	do
	writes(p.all; volatile)
	{
		while ((EECR & (1u << EEPE)) != 0) { }	// Wait for completion of previous write
		EEAR = ucAddress++;						// Set up address register
		EECR |= (1u << EERE);					// Start eeprom read by writing EERE
		*p++ = EEDR;							// Return data from data register
	} while (--len != 0);
}

void writeEEPROM(uint8_t ucAddress, const uint8_t * array p, uint8_t len)
pre(len != 0)
{
	do
	writes(volatile)
	{
		while ((EECR & (1u << EEPE)) != 0) { }	// Wait for completion of previous write
		EECR = (0u << EEPM1) | (0u << EEPM0);	// Set Programming mode
		EEAR = ucAddress++;						// Set up address and data registers
		EEDR = *p++;
		EECR |= (1u << EEMPE);					// Write logical one to EEMPE
		EECR |= (1u << EEPE);					// Start eeprom write by setting EEPE
	} while (--len != 0);
}

// ISR for the ADC interrupt
#ifdef __ECV__
void ADC_vect()
#else
ISR(ADC_vect)
#endif
writes(strainFilter; volatile)
pre(strainFilter.FilterInvariant(); AverageInvariant())
post(strainFilter.FilterInvariant(); AverageInvariant())
{
#if ISR_DEBUG
	PORTA |= BITVAL(PortADebugBitNum);
#endif

	const uint16_t adcVal = ((signedAdc) ? (ADC ^ 0x0200) : ADC) & 1023u;						// get the ADC reading from the previous conversion
	strainFilter.AddReading(adcVal);
	rollingAverage = rollingAverage + strainFilter.sum - shiftedRollingAverage;
	shiftedRollingAverage = (uint16_t)(rollingAverage >> avShift);

#if ISR_DEBUG
	PORTA &= ~BITVAL(PortADebugBitNum);
#endif
}

// Get the time now - must be called with interrupts disabled
inline uint16_t IGetTimeNow()
{
	uint8_t tim = TCNT0;
	const bool ovf = (TIFR0 & BITVAL(TOV0)) != 0;
	if (ovf)										// check for consistent overflow flag
	{
		tim = TCNT0;								// counter overflowed around the time we read it, so read it again
	}
	return (((ovf) ? tickCounter + 1 : tickCounter) << 8) | tim;
}

// Get the time now
uint16_t GetTimeNow()
{
	cli();
	const uint16_t ret = IGetTimeNow();
	sei();
	return ret;
}

// ISR for handling changes to the state of the input pin
// We store the times of transitions of the input pin in a buffer, with low to high transitions at even indices
// The only pin change interrupt we enable is the input pin, so we don't need to check which pin caused the interrupt
#ifdef __ECV__
void PCINT0_vect()
#else
ISR(PCINT0_vect)
#endif
{
	const uint8_t numEdgesCaptured = numberOfEdgesCaptured;			// capture volatile variable
	if (numEdgesCaptured < EdgeCaptureBufferEntries && ((numEdgesCaptured ^ (PINA >> PortADuetInputBitNum)) & 1u) != 0)		// low-to-high transitions must be stored at even indices
	{
		edgeCaptures[numEdgesCaptured] = IGetTimeNow();			// record the time at which this edge was detected
		numberOfEdgesCaptured = numEdgesCaptured + 1;
	}
}

// ISR for handling timer 0 overflow interrupt
#ifdef __ECV__
void TIM0_OVF_vect()
#else
ISR(TIM0_OVF_vect)
#endif
{
	++tickCounter;
}

// Get a 16-bit volatile value from outside the ISR. As it's more than 8 bits long, we need to disable interrupts while fetching it.
inline uint16_t GetVolatileWord(const volatile uint16_t& val)
writes(volatile)
{
	cli();
	const uint16_t locVal = val;
	sei();
	return locVal;
}

// Check whether we need to kick the watchdog
void CheckWatchdog()
writes(lastKickTicks; volatile)
{
	if (GetVolatileWord(tickCounter) - lastKickTicks >= kickIntervalTicks)
	{
#ifndef __ECV__
		wdt_reset();											// kick the watchdog
#endif
		lastKickTicks += kickIntervalTicks;
	}
}

// Delay for a specified number of ticks, kicking the watchdog as needed
void DelayTicks(uint16_t ticks)
writes(lastKickTicks; volatile)
{
	const uint16_t startTicks = GetVolatileWord(tickCounter);
	for (;;)
	{
		CheckWatchdog();
		if (GetVolatileWord(tickCounter) - startTicks >= ticks)
		{
			break;
		}
	}
}

inline void SetOutputOn()
writes(volatile)
{
	PORTA |= BITVAL(PortADuetOutputBitNum);
}

inline void SetOutputOff()
writes(volatile)
{
	PORTA &= ~(BITVAL(PortADuetOutputBitNum));
}

inline void SetLedOn()
writes(volatile)
{
	PORTA |= BITVAL(PortALedBitNum);
}

inline void SetLedOff()
writes(volatile)
{
	PORTA &= ~(BITVAL(PortALedBitNum));
}

void UpdateThreshold(uint16_t mv)
{
	threshold = ((uint32_t)mv * ((uint32_t)strainReadingsAveraged * 1024u * ThresholdMultiplier))/2800u;	// we use the 2.8V voltage reference for the ADC
}

// Flash the LED the specified number of times
void FlashLedNum(uint8_t numFlashes)
{
	SetLedOff();
	DelayTicks(ledFlashTicks);
	while (numFlashes != 0)
	{
		SetLedOn();
		DelayTicks(ledFlashTicks);
		SetLedOff();
		DelayTicks(ledFlashTicks);
		--numFlashes;
	}
	DelayTicks(ledFlashTicks);					// one more off time in case the sensor triggers immediately
}

inline void FlashLed(FlashCode fc)
{
	FlashLedNum((uint8_t)fc);
}

void UpdateEEPROM()
{
	nvData.checksum = NvData::magic ^ nvData.sensitivity ^ nvData.flags;
	writeEEPROM(0, reinterpret_cast<const uint8_t* array>(&nvData), sizeof(nvData));
}

// Check whether we have received a valid message.
void CheckReceivedMessage()
pre(numBytesReceived != 0; numBytesReceived <= MaxBytesReceived)
post(numBytesReceived < MaxBytesReceived)
{
	switch (bytesReceived[0])
	{
	case EraseByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == EraseByte)
			{
				nvData.flags &= ~NvData::FlagCustomSensitivity;
				UpdateThreshold(defaultThresholdMilliVolts);
				UpdateEEPROM();
				FlashLed(FlashCode::DefaultSensitivitySet);
			}
			numBytesReceived = 0;
		}
		break;

	case FactoryResetByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == FactoryResetByte)
			{
				nvData.flags = nvData.sensitivity = nvData.checksum = 0xFFFF;
				writeEEPROM(0, reinterpret_cast<const uint8_t* array>(&nvData), sizeof(nvData));
				UpdateEEPROM();
				FlashLed(FlashCode::StartupOkNoNvData);
				// We do not clear nvDataValid here, because if we did then the main loop would see that MOD is low and initialize the nvData again
			}
			numBytesReceived = 0;
		}
		break;

	case ReceiveTestByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == ReceiveTestByte)
			{
				FlashLed((nvData.flags & NvData::FlagCustomSensitivity) ? FlashCode::StartupOkCustomSensitivity : FlashCode::StartupOkStandardSensitivity);
			}
			numBytesReceived = 0;
		}
		break;

	case ProgByte:
		if (numBytesReceived >= 3)
		{
			if ((bytesReceived[2] ^ bytesReceived[1]) == 0x00FF)
			{
				// Valid programming message received
				nvData.flags |= NvData::FlagCustomSensitivity;
				nvData.sensitivity = (uint16_t)bytesReceived[1] + 1u;		// received values of 0-255 give sensitivity of 1-256mV
				UpdateThreshold(nvData.sensitivity);
				UpdateEEPROM();
				FlashLed(FlashCode::CustomSensitivitySet);
			}
			numBytesReceived = 0;
		}
		break;

	case NoiseMeasurementByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == NoiseMeasurementByte)
			{
				cli();
				noiseMeasurementStartTicks = tickCounter;
				sei();
				noiseLevel = 0;
				SetOutputOff();
				SetLedOff();
				noiseMeasureMode = NoiseMeasurementMode::reportPeak;
			}
			numBytesReceived = 0;
		}
		break;

	case NoisePassFailByte:
		if (numBytesReceived >= 2)
		{
			if (bytesReceived[1] == NoisePassFailByte)
			{
				cli();
				noiseMeasurementStartTicks = tickCounter;
				sei();
				noiseLevel = 0;
				SetOutputOff();
				SetLedOff();
				noiseMeasureMode = NoiseMeasurementMode::reportPassFail;
			}
			numBytesReceived = 0;
		}
		break;

	default:
		numBytesReceived = 0;			// invalid first byte
		break;
	}
}

// Check for programming commands. The protocol for each byte sent is:
//  0 0 1 0 b7 b6 b5 b4 /b4 b3 b2 b1 b0 /b0
// with a return to 0 after the last data byte. So the pattern with the minimum number of edges (4) is:
//  0 0 1 0 0 0 0 0 1 1 1 1 1 0
//      ^ ^         ^         ^
// A pattern with the maximum number of edges (12) is:
//  0 0 1 0 1 0 1 0 1 0 1 0 1 0
// The final transition to zero may occur after the end of the data, 12 bit times after the initial transition, e.g.:
//  0 0 1 0 1 1 1 1 0 0 0 0 0 1 0
// The maximum interval between edges in a single transmitted byte is 5 bit times.
void CheckForReceivedData()
pre(numBytesReceived < MaxBytesReceived)
post(numBytesReceived < MaxBytesReceived)
{
	enum class RxdState : uint8_t
	{
		waitingForStartBit,
		waitingForHighNibble,
		waitingForLowNibble,
		errorRecovery
	};

	static RxdState state = RxdState::waitingForStartBit;
	static uint16_t startBitLength;
	static uint16_t stuffingBitStart;
	static uint16_t errorRecoveryStartTime;
	static uint8_t highNibble;
	static uint8_t bitChangeIndex;

	const uint8_t numEdgesCaptured = numberOfEdgesCaptured;			// capture volatile variable
	const uint16_t now = GetTimeNow();
	switch (state)
	{
	case RxdState::waitingForStartBit:
		if (numEdgesCaptured >= 2)
		{
			// Check for a valid start bit
			startBitLength = edgeCaptures[1] - edgeCaptures[0];
			state = (startBitLength >= MinBitLength && startBitLength <= MaxBitLength) ? RxdState::waitingForHighNibble : RxdState::errorRecovery;
		}
		break;

	case RxdState::waitingForHighNibble:
		if (now - edgeCaptures[1] > (13 * startBitLength)/2)
		{
			// 6.5 bit times have passed since the end of the start bit, so we should have the high nibble and the following stuffing bit
			uint16_t samplePoint = (startBitLength * 3)/2;		// sampling time after the end of the start bit for bit 7 (MSB)
			bitChangeIndex = 2;
			highNibble = 0;
			for (uint8_t numBits = 0; numBits < 5; ++numBits)
			{
				if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
				{
					++bitChangeIndex;
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - edgeCaptures[1] < samplePoint)
					{
						state = RxdState::errorRecovery;		// there should be at most 1 transition per bit
						return;
					}
				}
				highNibble <<= 1;
				if ((bitChangeIndex & 1u) != 0)
				{
					highNibble |= 1u;
				}
				samplePoint += startBitLength;
			}

			// The 5th bit we received should be the inverse of the 4th bit
			if ((((highNibble >> 1u) ^ highNibble) & 0x01u) == 0)
			{
				state = RxdState::errorRecovery;
				return;
			}

			stuffingBitStart = edgeCaptures[bitChangeIndex - 1];
			state = RxdState::waitingForLowNibble;
		}
		break;

	case RxdState::waitingForLowNibble:
		if (now - stuffingBitStart > (13 * startBitLength)/2)
		{
			// 6.5 bit times have passed since the start of the first stuffing bit, so we should have the high nibble and the second stuffing bit
			uint16_t samplePoint = (3 * startBitLength)/2;			// resync on the start of the stuffing bit
			uint8_t lowNibble = 0;
			for (uint8_t numBits = 0; numBits < 5; ++numBits)
			{
				if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - stuffingBitStart < samplePoint)
				{
					++bitChangeIndex;
					if (bitChangeIndex < numEdgesCaptured && edgeCaptures[bitChangeIndex] - stuffingBitStart < samplePoint)
					{
						state = RxdState::errorRecovery;
						return;
					}
				}
				lowNibble <<= 1;
				if ((bitChangeIndex & 1u) != 0)
				{
					lowNibble |= 1u;
				}
				samplePoint += startBitLength;
			}

			// The 10th bit we received should be the inverse of the 9th bit
			if ((((lowNibble >> 1u) ^ lowNibble) & 0x01u) == 0)
			{
				state = RxdState::errorRecovery;
				return;
			}

			numberOfEdgesCaptured = 0;				// ready for a new byte
			bytesReceived[numBytesReceived++] = ((highNibble << 3u) & 0xF0u) | (lowNibble >> 1u);
			CheckReceivedMessage();
			state = RxdState::waitingForStartBit;
		}
		break;

	case RxdState::errorRecovery:
		if ((PINA & BITVAL(PortADuetInputBitNum)) != 0 || numEdgesCaptured != 0)	// when we first enter this state, numEdgesCaptured is always nonzero
		{
			errorRecoveryStartTime = now;
			numberOfEdgesCaptured = 0;
			numBytesReceived = 0;
		}
		else if (now - errorRecoveryStartTime >= ErrorRecoveryTime)
		{
			state = RxdState::waitingForStartBit;
		}
		break;
	}
}

// Run a self test on the hardware and flash the LED the appropriate number of times.
// On entry, nvDataValid must have been set correctly.
// Return true if there was a serous error. Otherwise return false with the ADC set up to read the strain gauge amplifier output.
bool SelfTest()
writes(volatile)
{
	ADMUX = (uint8_t)AdcBridgePowerInputChan;								// select the input from the bridge supply, Vcc as reference
	DelayTicks(slowTickFrequency/100);										// wait 10ms to accumulate a full filter of bridge supply voltage readings
	const uint16_t bridgeVoltage = GetVolatileWord(strainFilter.sum);		// get the bridge voltage supply reading

	// Check that Vcc is in range by measuring the voltage regulator output using Vcc as the reference
	if (bridgeVoltage > VccMinAdcReading)
	{
		FlashLed(FlashCode::VccTooLow);
		return true;
	}
	else if (bridgeVoltage < VccMaxAdcReading)
	{
		FlashLed(FlashCode::VccTooHigh);
		return true;
	}

	// Check the strain gauge output voltage. It should be close to half the bridge supply voltage.
	ADMUX = (uint8_t)(AdcStrainInputChan | BITVAL(REFS0));					// select the input from the bridge output, external reference
	DelayTicks(slowTickFrequency/100);										// wait 10ms to accumulate a full filter of strain output readings
	const uint16_t bridgeOutput = GetVolatileWord(strainFilter.sum);

	const uint16_t expectedOutput = 1024u/2u * strainReadingsAveraged;
			
	// We allow about 5% difference between the resistances during the initial self-test before the nvData is set up, and 10% after that. 5% error will change the ADC reading by 12.5
	const uint16_t allowedError = (nvData.IsValid()) ? MaxImbalanceReadingInNormalUse * strainReadingsAveraged : MaxImbalanceReadingInTesting * strainReadingsAveraged;
	FlashCode fc;
	bool error = true;
	if (bridgeOutput > expectedOutput + allowedError)
	{
		fc = FlashCode::BridgeOutputTooHigh;
	}
	else if (bridgeOutput < expectedOutput - allowedError)
	{
		 fc = FlashCode::BridgeOutputTooLow;
	}
	else
	{
		error = false;
		fc = (!nvData.IsValid()) ? FlashCode::StartupOkNoNvData
				: (nvData.flags & NvData::FlagCustomSensitivity) ? FlashCode::StartupOkCustomSensitivity
					: FlashCode::StartupOkStandardSensitivity;
	}

	FlashLed(fc);
	return error;
}

int main(void)
writes(strainFilter; volatile)
{
	cli();

	strainFilter.Init(512);

	// Initialise the I/O ports
	DIDR0 = Didr0BitMask;									// disable digital input buffers on ADC pins
	PORTA = PortAUnusedBitMask | BITVAL(PortADuetInputBitNum);	// enable pullup on inputs and on unused I/O pins, set the output off
	PORTB = PortBUnusedBitMask;
#if ISR_DEBUG
	DDRA = BITVAL(PortADuetOutputBitNum) | BITVAL(PortALedBitNum) | BITVAL(PortADebugBitNum);
#else
	DDRA = BITVAL(PortADuetOutputBitNum) | BITVAL(PortALedBitNum);
#endif

#ifndef __ECV__												// eCv++ doesn't understand gcc assembler syntax
	wdt_enable(WDTO_500MS);									// enable the watchdog
#endif
	
	// Set up the ADC in free-running mode
	ADMUX = (uint8_t)AdcBridgePowerInputChan;				// select the input from the bridge supply, Vcc reference
	ADCSRB = 0;												// free running, unipolar input mode

#if CLOCK_500K
	// Try 500kHz ADC clock
	ADCSRA = BITVAL(ADEN) | BITVAL(ADATE) | BITVAL(ADIF) | BITVAL(ADIE) | BITVAL(ADPS2);
	// enable ADC, auto trigger enable, clear interrupt, enable interrupt, prescaler = 16 (ADC clock ~= 500kHz)
#else
	ADCSRA = BITVAL(ADEN) | BITVAL(ADATE) | BITVAL(ADIF) | BITVAL(ADIE) | BITVAL(ADPS2) | BITVAL(ADPS0);
															// enable ADC, auto trigger enable, clear interrupt, enable interrupt, prescaler = 32 (ADC clock ~= 250kHz)
#endif
	// Set up timer 0 to generate the tick interrupt
	TCCR0A = 0;												// Mode 0, no outputs enabled
	TCCR0B = BITVAL(CS01);									// Prescaler = 8, it clocks at 1MHz
	TIMSK0 |= BITVAL(TOIE0);								// Enable timer 0 overflow interrupt

	sei();
	ADCSRA |= BITVAL(ADSC);									// start first conversion

	do
	{
		DelayTicks(3 * slowTickFrequency);					// wait 3 seconds to allow the supply and the sense amplifier to stabilise
		readEEPROM(0u, reinterpret_cast<uint8_t* array>(&nvData), sizeof(nvData));
	} while (SelfTest());

	const uint16_t thresholdMillivolts = (nvData.IsValid() && (nvData.flags & NvData::FlagCustomSensitivity) != 0) ? nvData.sensitivity : defaultThresholdMilliVolts;
	UpdateThreshold(thresholdMillivolts);

	if (!nvData.IsValid())
	{
		nvData.flags = 0;
		UpdateEEPROM();
	}

	cli();

	// Set up pin change interrupt for detecting programming signals
	GIFR = BITVAL(PCIF0);									// clear any existing interrupt
	GIMSK = BITVAL(PCIE0);									// enable pin change interrupt 0
	PCMSK0 = BITVAL(PinChange0InputBitNum);					// enable pin change interrupt on just the input pin

	// Switch the ADC into differential mode
	ADCSRA &= ~BITVAL(ADEN);
	ADMUX = 0b001111 | BITVAL(REFS0);						// differential mode, gain 20
	ADCSRB = BITVAL(BIN);
	signedAdc = true;
	strainFilter.Init(512);
	shiftedRollingAverage = 512 * strainReadingsAveraged;
	rollingAverage = (uint32_t)(512 * strainReadingsAveraged) << avShift;
	ADCSRA |= BITVAL(ADEN);
	ADCSRA |= BITVAL(ADSC);

	sei();

	// Start main loop
	for (;;)
	{
		// Set the output on if we are triggered
		// Fetch values of volatile variables more than 1 byte long with interrupts disabled to prevent getting inconsistent values
		cli();
		const uint16_t locShiftedRollingAverage = shiftedRollingAverage;
		const uint16_t locSum = strainFilter.sum;
		const uint16_t now = tickCounter;
		sei();

		if (noiseMeasureMode != NoiseMeasurementMode::off)
		{
			// Measure the peak reading over the number of measurement ticks
			if (locSum > noiseLevel + locShiftedRollingAverage)
			{
				noiseLevel = locSum - locShiftedRollingAverage;
			}

			if (now - noiseMeasurementStartTicks >= NoiseMeasurementTicks)
			{
				// Convert the peak reading to an equivalent sensitivity
				const uint16_t convertedNoiseLevel = ((uint32_t)noiseLevel * 2800u)/((uint32_t)strainReadingsAveraged * 1024u * ThresholdMultiplier);
				const uint8_t standardisedNoiseLevel = (convertedNoiseLevel > 255) ? 255
														: (convertedNoiseLevel > 1) ? convertedNoiseLevel - 1
															: 1;
				if (noiseMeasureMode == NoiseMeasurementMode::reportPeak)
				{
					// Report the peak reading as a number of flashes
					FlashLedNum(standardisedNoiseLevel);
				}
				else
				{
					// Report whether the peak reading is below the allowed threshold
					SetLedOn();
					SetOutputOn();
					if (standardisedNoiseLevel >= NoiseMeasurementFailThreshold)
					{
						DelayTicks(5 * ledFlashTicks);					// one 5-second flash
					}
					else
					{
						DelayTicks(ledFlashTicks);
					}
					SetLedOff();
					SetOutputOff();
				}
				DelayTicks(2 * slowTickFrequency);						// leave the LED off for 2 seconds
				noiseMeasureMode = NoiseMeasurementMode::off;
			}
		}
		else if (locSum >= locShiftedRollingAverage + threshold)
		{
			SetOutputOn();
			SetLedOn();
			timeOn = now;
		}
		else if (now - timeOn > ledOnTicks)
		{
			SetOutputOff();
			SetLedOff();
		}

		CheckForReceivedData();
		CheckWatchdog();
	}

#ifdef __ECV__
	return 0;				// to keep eCv happy
#endif
}

// End
