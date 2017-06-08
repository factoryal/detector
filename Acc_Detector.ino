#include <SoftwareSerial.h>

// �� ������ ������ ������ �︮�� ������ 1�� �ڿ� �ڵ����� ������ �����ϴ�.
#define ALARM_AUTO_OFF


typedef int16_t vt;
// Vector3 Ŭ����
// 3���� ���Ͱ��� �����ϴ� Ŭ�����Դϴ�.
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

// Acc Ŭ����
// ���ӵ� ������ �������� �����ϴ� Ŭ�����Դϴ�.
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


// Detector Ŭ����
// ������ ���� ��带 �����ϴ� Ŭ�����Դϴ�.
// ����� ���ÿ� �ʱ�ȭ�Ǿ ���� �ʱ�ȭ���� �ʿ�� �����ϴ�.
// Detector.on() �Լ��� ���� ��带 �Ѱ�
// Detector.off() �Լ��� ���� ��带 �� �� �ֽ��ϴ�.
// ���� ��尡 ������ �� �������� �����Ǹ� alert() �Լ��� �����մϴ�.
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

// ������ ���� ��尡 ON�� ���¿��� �������� ������ �� ������ �ڵ带
// alert() �Լ� �ȿ� �ۼ��ϼ���.
// delay �Լ� ����� �����ϵ��� �մϴ�.
void alert() {
	alarm.on();
	/*Detector.off();*/
}

// ��ư�� ���� �� ������ �ڵ带
// button() �Լ� �ȿ� �ۼ��ϼ���.
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
	// ��Ÿ �ʱ⿡ �� �� ������ �ڵ带 �Ʒ��� �ۼ��ϼ���.
	
}

void loop() {
	// ��������� �����Ͱ� ���� ��쿡 ������ �ڵ带
	// �� if�� �ȿ� �ۼ��ϼ���.
	if (BT.available()) {

	}
}