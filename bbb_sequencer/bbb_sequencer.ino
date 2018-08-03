#include <TimerOne.h>
#include <stdint.h>


constexpr int latchPin = 3; //Pin connected to ST_CP of 74HC595
constexpr int clockPin = 12; //Pin connected to SH_CP of 74HC595
constexpr int dataPin = 10; ////Pin connected to DS of 74HC595
constexpr int switchPin = 8;
constexpr int modePin = 13;
constexpr int syncSwitch = 9;

enum SyncMode {
	InternalBpm,
	ExternalSync,
};
SyncMode syncMode;


#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

enum {
	TOTAL_STEP = 16,
};


uint8_t stepPattern[TOTAL_STEP] =
{
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
};

uint8_t lastSwitchState[TOTAL_STEP] =
{
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
};

uint16_t analogValues[TOTAL_STEP] =
{
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
};

int32_t mappedbpm = 120;
int8_t currentStep = 0;
int32_t stepCounter = 0;
uint32_t stepCountMax;

void changeBpm(const float newBpm)
{
	float tmp = 60 * 1000.0 / newBpm;
	tmp /= 4.0;
	stepCountMax = tmp;
}

int currentScanStep = 0;

enum {
	ADDR_0 = 7,
	ADDR_1 = 6,
	ADDR_2 = 5,
	ADDR_3 = 4,
};

constexpr uint8_t loHiTables[TOTAL_STEP][4] = {
// ADDR	 3  2  1  0
		{0, 0, 0, 0},
	  {0, 0, 0, 1},
		{0, 0, 1, 0},
		{0, 0, 1, 1},
		
		{0, 1, 0, 0},
		{0, 1, 0, 1},
		{0, 1, 1, 0},
		{0, 1, 1, 1},
		
		{1, 0, 0, 0},
		{1, 0, 0, 1},
		{1, 0, 1, 0},
		{1, 0, 1, 1},
		
		{1, 1, 0, 0},
		{1, 1, 0, 1},
		{1, 1, 1, 0},
		{1, 1, 1, 1},
};


void scanInputs()
{
	const auto scannedSwitch = digitalRead(switchPin);
	if (lastSwitchState[currentScanStep] == 0
		&& scannedSwitch) {
		stepPattern[currentScanStep] ^= scannedSwitch;
	}
	lastSwitchState[currentScanStep] = scannedSwitch;

	analogValues[currentScanStep] = analogRead(0);

	currentScanStep++;
	if (currentScanStep >= TOTAL_STEP) {
		currentScanStep = 0;
	}

	const uint8_t* table = loHiTables[currentScanStep];

	digitalWrite(ADDR_0, table[3]);
	digitalWrite(ADDR_1, table[2]);
	digitalWrite(ADDR_2, table[1]);
	digitalWrite(ADDR_3, table[0]);
}


void handleLed()
{
	uint32_t bitPattern1 = 1 << currentStep; // for step led
	uint32_t bitPattern2 = 0;
	
	for (auto i = 0; i < TOTAL_STEP; ++i) {
		bitPattern2 |= (stepPattern[i] << i);
	}
	
	uint32_t bitPattern = bitPattern1 ^ bitPattern2;
	
    digitalWrite(latchPin, 0);
    shiftOut(dataPin, clockPin, bitPattern); 
    digitalWrite(latchPin, 1);	
}

ISR(TIMER2_OVF_vect) {
}

bool updateAnalog = false;

void timerInterrupt()
{
	scanInputs();
	
	if (syncMode == InternalBpm) {
		stepCounter++;
		if (stepCounter >= stepCountMax) {
			stepCounter = 0;
			
			currentStep++;
			updateAnalog = true;
			
			if (currentStep >= 16) {
				currentStep = 0;
			}
		}
	}
	
	handleLed();
}

void syncReceived()
{
	currentStep++;
	if (currentStep >= TOTAL_STEP) {
		currentStep = 0;
	}

	updateAnalog = true;
}

TimerOne timer;
void setup()
{
  Serial.begin(115200);

  // set adc prescaler  to 64 for 19kHz sampling frequency
  cbi(ADCSRA, ADPS2);
  sbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);
  sbi(ADMUX,ADLAR);  // 8-Bit ADC in ADCH Register
  sbi(ADMUX,REFS0);  // VCC Reference
  cbi(ADMUX,REFS1);
  cbi(ADMUX,MUX0);   // Set Input Multiplexer to Channel 0
  cbi(ADMUX,MUX1);
  cbi(ADMUX,MUX2);
  cbi(ADMUX,MUX3);
  
  // Timer2 PWM Mode set to fast PWM 
  cbi (TCCR2A, COM2A0);
  sbi (TCCR2A, COM2A1);
  sbi (TCCR2A, WGM20);
  sbi (TCCR2A, WGM21);
  cbi (TCCR2B, WGM22);

  // Timer2 Clock Prescaler to : 1 
  sbi (TCCR2B, CS20);
  cbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);

  // Timer2 PWM Port Enable
  sbi(DDRB,3);                     // set digital pin 11 to output
  cbi (TIMSK0,TOIE0);              // disable Timer0 !!! delay is off now
  sbi (TIMSK2,TOIE2);              // enable Timer2 Interrupt
	
  pinMode(latchPin, OUTPUT);
  pinMode(ADDR_0, OUTPUT);
  pinMode(ADDR_1, OUTPUT);
  pinMode(ADDR_2, OUTPUT);
  pinMode(ADDR_3, OUTPUT);

  timer.initialize(2000); // 2ms resolution
  timer.attachInterrupt(timerInterrupt);
  timer.start();
  
  // configure sync mode(only once so far!!!)
  syncMode = digitalRead(syncSwitch) == 1 ? InternalBpm : ExternalSync;

  if (syncMode == ExternalSync) {
	  attachInterrupt(0, syncReceived, RISING);
  }
}



void loop()
{
	// update output cv
	if (updateAnalog) {
		updateAnalog = false;
		OCR2A = stepPattern[currentStep] * (analogValues[currentStep] >> 2);
	}
	
	
	// update bpm
	const auto bpmVolume = analogRead(1) >> 2 << 2; 
	const auto mapped = map(bpmVolume, 0, 1023, 200, 3000);
	if (mappedbpm != mapped) {
		mappedbpm = mapped;
		changeBpm(mapped / 10.0);
	}

	// update mode.. not implemented yet..
	const auto modeSwitch = digitalRead(modePin);
}

void shiftOut(int myDataPin, int myClockPin, int32_t myDataOut)
{
	int i=0;
	int pinState;
	pinMode(myClockPin, OUTPUT);
	pinMode(myDataPin, OUTPUT);
	
	digitalWrite(myDataPin, 0);
	digitalWrite(myClockPin, 0);
	
	for (i=TOTAL_STEP; i>=0; i--)  {
		digitalWrite(myClockPin, 0);
		if ( myDataOut & (1<<i) ) {
			pinState= 1;
		}
		else {	
			pinState= 0;
		}
	  
		digitalWrite(myDataPin, pinState);
		digitalWrite(myClockPin, 1);
		digitalWrite(myDataPin, 0);
	}
	digitalWrite(myClockPin, 0);
}
