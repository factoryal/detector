#include <SoftwareSerial.h>

// 이 선언이 있으면 부저가 울리기 시작한 1초 뒤에 자동으로 부저가 꺼집니다.
#define ALARM_AUTO_OFF

// Vector3 클래스
// 3차원 벡터값을 관리하는 클래스입니다.
class Vector3 {
private:
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;

public:
	Vector3() {};
	Vector3(int16_t x, int16_t y, int16_t z) {
		this->x = x; this->y = y; this->z = z;
	}

	void setxyz(int16_t x, int16_t y, int16_t z) {
		this->x = x; this->y = y; this->z = z;
	}
	uint16_t getAbs() {
		return sqrt((uint32_t)x*x + (uint32_t)y*y + (uint32_t)z*z);
	}
	Vector3 getDir() {
		uint16_t abs = getAbs();
		return Vector3(x / abs, y / abs, z / abs);
	}
	Vector3 getxyz() { return Vector3(x, y, z); }
	inline int16_t getx() { return x; }
	inline int16_t gety() { return y; }
	inline int16_t getz() { return z; }
};

// Acc 클래스
// 가속도 센서와 측정값을 관리하는 클래스입니다.
class Acc {
private:
	Vector3 raw;
	uint8_t pin[3] = { A0, A1, A2 };
public:
	void update() { raw.setxyz(analogRead(A0), analogRead(A1), analogRead(A2)); }
	Vector3 getDir() { return raw.getDir(); }
	uint16_t getAbs() { return raw.getAbs(); }
	Vector3 getRaw() { return raw.getxyz(); }
	inline int16_t getx() { return raw.getx(); }
	inline int16_t gety() { return raw.gety(); }
	inline int16_t getz() { return raw.getz(); }
} Acc;


// Buzzer 클래스
// 부저의 소리를 관리하는 클래스입니다.
// 선언과 동시에 초기화되어 있어서 따로 초기화해줄 필요는 없습니다.
// alarm.on() 함수로 부저를 켜고
// alarm.off() 함수로 부저를 끌 수 있습니다.
// 클래스 선언 마지막 부분에 클래스 초기화 2번째 인자의 숫자를 바꾸어 주파수를 바꿀 수 있습니다. (기본: 1000)
class Buzzer {
private:
	uint8_t pin = 0;
	uint16_t freq = 0;

public:
	Buzzer(uint8_t pin, uint8_t freq) {
		this->pin = pin; this->freq = freq;
		pinMode(pin, 1);
	}

	void on() {
#ifdef ALARM_AUTO_OFF
		tone(pin, freq, 1000); 
#else
		tone(pin, freq);
#endif
	}
	void off() { noTone(pin); }
} alarm(12, 1000);

// Pressure 클래스
// 압력센서를 관리하는 클래스입니다.
// 선언과 동시에 초기화되어서 따로 초기화해줄 필요는 없습니다.
class Pressure {
private:
	uint8_t pin = A3;
	uint16_t threshold;

public:
	
	bool isPressed() {
		return threshold < analogRead(pin);
	}
	void setThreshold(void) {
		threshold = analogRead(pin);
	}
	void setThreshold(uint16_t t) {
		threshold = t;
	}

	
} Pressure;


// Detector 클래스
// 움직임 감지 모드를 관리하는 클래스입니다.
// 선언과 동시에 초기화되어서 따로 초기화해줄 필요는 없습니다.
// Detector.on() 함수로 감지 모드를 켜고
// Detector.off() 함수로 감지 모드를 끌 수 있습니다.
// 감지 모드가 켜져을 때 움직임이 감지되면 alert() 함수를 실행합니다.
class Detector {
public:
	const uint16_t threshold;
	Vector3 initV;
	void(*callback)(void);

	Detector(uint16_t t, void(*func)(void)) : threshold(t) {
		callback = func;
		cli();
		TCCR1A = 0;
		TCCR1B = 0;
		OCR1A = 15624;
		TCCR1B |= (1 << WGM12);
		TCCR1B |= (1 << CS12);
		TCCR1B |= (1 << CS10);
		TIMSK1 &= 0xFD;
		sei();
	}
	void on() {
		TIMSK1 |= (1 << OCIE1A); 
		Acc.update();
		initV.setxyz(Acc.getx(), Acc.gety(), Acc.getz());
		Pressure.setThreshold();
	}
	void off() {
		TIMSK1 &= 0xFD; alarm.off();
	}
} Detector(10, alert);



uint32_t isr_now_time = 0, isr_old_time = 0, isr_interval = 100;

ISR(TIMER1_COMPA_vect) {
	isr_now_time = millis();
	if (isr_now_time - isr_old_time > isr_interval) {
		isr_old_time = isr_now_time;
		Acc.update();
		Vector3 now = Acc.getRaw();
		/*Serial.print(now.getx());
		Serial.print('\t');
		Serial.print(now.gety());
		Serial.print('\t');
		Serial.print(now.getz());
		Serial.print('\t');
		Serial.print(int32_t(Detector.initV.getAbs()) - now.getAbs());
		Serial.print('\t');
		Serial.println(now.getAbs());*/
		Serial.println(analogRead(A3));
		if (now.getAbs() < Detector.initV.getAbs() - Detector.threshold || now.getAbs()>Detector.initV.getAbs() + Detector.threshold || Pressure.isPressed()) Detector.callback();
	}
}

// 움직임 감지 모드가 ON인 상태에서 움직임이 감지되면 실행되는 함수입니다.
// delay 함수 사용은 자제하도록 합니다.
void alert() {
	alarm.on();
}


SoftwareSerial BT(4, 5);

void setup() {
	Serial.begin(115200);
	BT.begin(9600);
	pinMode(13, OUTPUT);
	delay(100);
	Detector.on();
	
}

void loop() {
}