#include <SoftwareSerial.h>

// �� ������ ������ ������ �︮�� ������ 1�� �ڿ� �ڵ����� ������ �����ϴ�.
#define ALARM_AUTO_OFF

// Vector3 Ŭ����
// 3���� ���Ͱ��� �����ϴ� Ŭ�����Դϴ�.
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

// Acc Ŭ����
// ���ӵ� ������ �������� �����ϴ� Ŭ�����Դϴ�.
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


// Buzzer Ŭ����
// ������ �Ҹ��� �����ϴ� Ŭ�����Դϴ�.
// ����� ���ÿ� �ʱ�ȭ�Ǿ� �־ ���� �ʱ�ȭ���� �ʿ�� �����ϴ�.
// alarm.on() �Լ��� ������ �Ѱ�
// alarm.off() �Լ��� ������ �� �� �ֽ��ϴ�.
// Ŭ���� ���� ������ �κп� Ŭ���� �ʱ�ȭ 2��° ������ ���ڸ� �ٲپ� ���ļ��� �ٲ� �� �ֽ��ϴ�. (�⺻: 1000)
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

// Pressure Ŭ����
// �з¼����� �����ϴ� Ŭ�����Դϴ�.
// ����� ���ÿ� �ʱ�ȭ�Ǿ ���� �ʱ�ȭ���� �ʿ�� �����ϴ�.
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


// Detector Ŭ����
// ������ ���� ��带 �����ϴ� Ŭ�����Դϴ�.
// ����� ���ÿ� �ʱ�ȭ�Ǿ ���� �ʱ�ȭ���� �ʿ�� �����ϴ�.
// Detector.on() �Լ��� ���� ��带 �Ѱ�
// Detector.off() �Լ��� ���� ��带 �� �� �ֽ��ϴ�.
// ���� ��尡 ������ �� �������� �����Ǹ� alert() �Լ��� �����մϴ�.
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

// ������ ���� ��尡 ON�� ���¿��� �������� �����Ǹ� ����Ǵ� �Լ��Դϴ�.
// delay �Լ� ����� �����ϵ��� �մϴ�.
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