#include <SoftwareSerial.h>

// 이 선언이 있으면 부저가 울리기 시작한 1초 뒤에 자동으로 부저가 꺼집니다.
#define ALARM_AUTO_OFF


typedef int16_t vt;
// Vector3 클래스
// 3차원 벡터값을 관리하는 클래스입니다.
class Vector3 {
private:
	vt x = 0;
	vt y = 0;
	vt z = 0;

public:
	Vector3() {};
	Vector3(vt x, vt y, vt z) {
		this->x = x; this->y = y; this->z = z;
	}

	void setxyz(vt x, vt y, vt z) {
		this->x = x; this->y = y; this->z = z;
	}

	void setxyz(Vector3 v) {
		x = v.x; y = v.y; z = v.z;
	}

	vt getAbs() {
		return sqrt((uint32_t)x*x + (uint32_t)y*y + (uint32_t)z*z);
	}
	Vector3 getDir() {
		vt abs = getAbs();
		return Vector3(x / abs, y / abs, z / abs);
	}
	Vector3 getxyz() { return Vector3(x, y, z); }
	inline vt getx() { return x; }
	inline vt gety() { return y; }
	inline vt getz() { return z; }
};

// Acc 클래스
// 가속도 센서와 측정값을 관리하는 클래스입니다.
class Acc {
private:
	Vector3 raw;
	uint8_t pin[3] = { A2, A3, A4 };
public:
	void update() { raw.setxyz(analogRead(A0), analogRead(A1), analogRead(A2)); }
	Vector3 getDir() { return raw.getDir(); }
	vt getAbs() { return raw.getAbs(); }
	Vector3 getRaw() { return raw.getxyz(); }
	inline vt getx() { return raw.getx(); }
	inline vt gety() { return raw.gety(); }
	inline vt getz() { return raw.getz(); }
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
	uint8_t pin = A5;
	uint16_t threshold;

public:
	Pressure() {
		pinMode(A5, INPUT_PULLUP);
	}
	bool isPressed() {
		return threshold < analogRead(pin);
		Serial.print(analogRead(pin));
		Serial.print('\t');
	}
	void setThreshold(void) {
		threshold = analogRead(pin)+20;
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
private:
	bool s = 0;
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
		for(int i=0; i<=3; i++) Acc.update();
		initV.setxyz(Acc.getRaw());
		Pressure.setThreshold();
		s = 1;
	}
	void off() {
		TIMSK1 &= 0xFD;
		s = 0;
	}
	bool status() { return s; }
} Detector(10, alert);



uint32_t isr_now_time = 0, isr_old_time = 0, isr_interval = 100;
Vector3 now, old;

ISR(TIMER1_COMPA_vect) {
	isr_now_time = millis();
	if (isr_now_time - isr_old_time > isr_interval) {
		isr_old_time = isr_now_time;
		Acc.update();
		now = Acc.getRaw();
		Serial.print(now.getx());
		Serial.print('\t');
		Serial.print(now.gety());
		Serial.print('\t');
		Serial.print(now.getz());
		Serial.print('\t');
		Serial.print(vt(old.getAbs()-now.getAbs()));
		Serial.print('\t');
		Serial.print(now.getAbs());
		Serial.print('\t');
		Serial.println(analogRead(A5));
		if (now.getAbs() < old.getAbs() - Detector.threshold || now.getAbs()>old.getAbs() + Detector.threshold || Pressure.isPressed()) Detector.callback();
		old = now;
	}
}

// 움직임 감지 모드가 ON인 상태에서 움직임이 감지될 때 실행할 코드를
// alert() 함수 안에 작성하세요.
// delay 함수 사용은 자제하도록 합니다.
void alert() {
	alarm.on();
	/*Detector.off();*/
}

// 버튼이 눌릴 때 실행할 코드를
// button() 함수 안에 작성하세요.
void button() {
	alarm.off();
	Detector.on();
}


SoftwareSerial BT(4, 5);

void setup() {
	Serial.begin(115200);
	BT.begin(9600);
	pinMode(13, OUTPUT);
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(2), button, FALLING);
	delay(100);

	Detector.on();
	// 기타 초기에 한 번 실행할 코드를 아래에 작성하세요.
	
}

void loop() {
	// 블루투스로 데이터가 들어온 경우에 실행할 코드를
	// 이 if문 안에 작성하세요.
	if (BT.available()) {

	}
}