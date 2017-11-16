#include <SoftwareSerial.h>
#include "MPU9250.h"
#include "I2Cdev.h"
#include "pin_definition.h"

// �� ������ ������ ������ �︮�� ������ 1�� �ڿ� �ڵ����� ������ �����ϴ�.
//#define ALARM_AUTO_OFF

// �� ������ ������ ���ӵ� ���� �Է¿� Low Pass Filter�� ����˴ϴ�.
#define ENABLE_ACC_LOW_PASS
#if defined(ENABLE_ACC_LOW_PASS)
#define ts 0.03
#define tau 1/(0.8*2*3.14)
#define ttt ts/(tau+ts)
#endif

void alert();
void non_alert();

typedef float vt;
// Vector3 Ŭ����
// 3���� ���Ͱ��� �����ϴ� Ŭ�����Դϴ�.
class Vector3 {
private:
	vt x = 0;
	vt y = 0;
	vt z = 0;

public:
	// constructor
	Vector3() : x(0), y(0), z(0) {};
	Vector3(const Vector3 &v) : x(v.x), y(v.y), z(v.z) {};
	Vector3(const vt &_x, const vt &_y, const vt &_z) : x(_x), y(_y), z(_z) {};


	// setter

	void set(const Vector3 &v) { x = v.x; y = v.y; z = v.z; }
	void set(const vt &_x, const vt &_y, const vt &_z) { x = _x; y = _y; z = _z; }
	void setX(const vt &_x) { x = _x; }
	void setY(const vt &_y) { y = _y; }
	void setZ(const vt &_z) { z = _z; }
	void setdX(const vt &_x) { x += _x; }
	void setdY(const vt &_y) { y += _y; }
	void setdZ(const vt &_z) { z += _z; }


	// getter

	vt getX() { return x; }
	vt getY() { return y; }
	vt getZ() { return z; }
	// ������ ũ�⸦ ���մϴ�.
	float getMagnitude() { return sqrt(x*x + y*y + z*z); }

	// ������ ���� ���͸� ���մϴ�.
	Vector3 getDirection() { return *this / getMagnitude(); }

	// x��� ���� ������ ������ cos ���� ���մϴ�.
	float getDirCosAlpha() { return (float)x / getMagnitude(); }

	// y��� ���� ������ ������ cos ���� ���մϴ�.
	float getDirCosBeta() { return (float)y / getMagnitude(); }

	// z��� ���� ������ ������ cos ���� ���մϴ�.
	float getDirCosGamma() { return (float)z / getMagnitude(); }


	// ���� ����

	Vector3 operator+(const Vector3 &v) { return Vector3(x + v.x, y + v.y, z + v.z); }
	Vector3 operator-(const Vector3 &v) { return Vector3(x - v.x, y - v.y, z - v.z); }

	Vector3 operator+(const vt &d) { return Vector3(x + d, y + d, z + d); }
	Vector3 operator-(const vt &d) { return Vector3(x - d, y - d, z - d); }

	void operator+=(const Vector3 &v) { x += v.x; y += v.y; z += v.z; }
	void operator-=(const Vector3 &v) { x -= v.x; y -= v.y; z -= v.z; }

	void operator+=(const vt &d) { x += d; y += d; z += d; }
	void operator-=(const vt &d) { x -= d; y -= d; z -= d; }

	Vector3 operator*(const vt &d) { return Vector3(x*d, y*d, z*d); }
	Vector3 operator/(const vt &d) { return Vector3(x / d, y / d, z / d); }

	void operator*=(const vt &d) { x *= d; y *= d; z *= d; }
	void operator/=(const vt &d) { x /= d; y /= d; z /= d; }

	vt dotTo(const Vector3 &v) { return x*v.x + y*v.y + z*v.z; }
	Vector3 crossTo(const Vector3 &v) { return Vector3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }


	// Serial print
	void printxyz() {
		Serial.print(x);
		Serial.print(' ');
		Serial.print(y);
		Serial.print(' ');
		Serial.print(z);
	}

	void printxyzln() { printxyz(); Serial.println(); }

	void printcos() {
		Serial.print(getDirCosAlpha());
		Serial.print(' ');
		Serial.print(getDirCosBeta());
		Serial.print(' ');
		Serial.print(getDirCosGamma());
	}


	// destructor(auto)
	// ~Vector3();
};


enum SENSOR {
	ACCEL = 1,
	GYRO = 2,
	MAGNETRO = 4
};

// GY9250 Ŭ����
// 9��(���ӵ�3��, ���̷�3��, �ڱ���3��) ������ �����ϴ� Ŭ�����Դϴ�.
class GY9250 {
private:
	Vector3 a;
	Vector3 ba;
	Vector3 init_a;
	int16_t buffer[9] = { 0 };
	
	MPU9250 M;

	uint8_t sensorEnabled = 0x00;


public:
	int block = 1;
	// constructor
	/*GY9250() {
	}*/

	bool init() {
		Serial.println(F("GY9250 initializing...."));
		M.initialize();
		delay(1000);
		Serial.print(F("Device ID: ")); Serial.println(M.getDeviceID(), HEX);
		//return M.testConnection();
		return 1;
	}

	// ���� ����
	uint8_t activate(uint8_t en) {
		sensorEnabled = en;
		if (sensorEnabled&ACCEL) {
			M.setStandbyXAccelEnabled(0);
			M.setStandbyYAccelEnabled(0);
			M.setStandbyZAccelEnabled(0);
		}
		Serial.print(F("Sensor enabled: ")); Serial.print(sensorEnabled&ACCEL); Serial.println(sensorEnabled&GYRO);
		return sensorEnabled;
	}

	// enable �� ������ ���� �а� ������Ʈ�մϴ�.
	void updateData(void) {
		M.getAcceleration(buffer, buffer + 1, buffer + 2);
		if (sensorEnabled&ACCEL) {
			/*a.set(fmap(buffer[0], 18750, -13950, 9.8, -9.8), fmap(buffer[1], 17800, -15000, 9.8, -9.8), fmap(buffer[2], 12875, -20550, 9.8, -9.8));*/// a /= 16384;
			a.set(buffer[0], buffer[1], buffer[2]);
			a = ba + (a - ba)*ttt;
			ba = a;
		}
	}

	// Serial prints
	void printAccelerometer() {
		/*Serial.print(a.getMagnitude());*/
		Serial.print(a.getX());
		Serial.print(' ');
		Serial.print(a.getY());
		Serial.print(' ');
		Serial.print(a.getZ());
		Serial.print(' ');
		Serial.print(a.getMagnitude());
	}

	Vector3 getA() {
		return a;
	}

} GY9250;


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
	void off() { noTone(pin); Serial.println("alarm off"); }
} alarm(PIN_BUZZER, 1000);

// Pressure Ŭ����
// �з¼����� �����ϴ� Ŭ�����Դϴ�.
// ����� ���ÿ� �ʱ�ȭ�Ǿ ���� �ʱ�ȭ���� �ʿ�� �����ϴ�.
class Pressure {
private:
	uint8_t pin = PIN_PRESS;
	uint16_t threshold;

public:
	Pressure() {
		pinMode(pin, INPUT_PULLUP);
	}
	bool isPressed() {
		return threshold < analogRead(pin);
		Serial.print(analogRead(pin));
		Serial.print('\t');
	}
	void setThreshold(void) {
		threshold = analogRead(pin) + 200;
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
// Detector.on() �Լ��� �����ϸ� ������ ���ӵ� ���� ���� ���� ���Ӱ� ����մϴ�.
// ������� �۵� ���� �� �� ����� ���� ���� �̻� ���̰� ���� alert() �Լ��� ����˴ϴ�.
class Detector {
private:
	bool s = 0;
public:
	const uint16_t threshold;
	Vector3 initV;
	void(*callback1)(void);
	void(*callback2)(void);

	Detector(uint16_t t, void(*func1)(void), void(*func2)(void)) : threshold(t) {
		callback1 = func1;
		callback2 = func2;
#if defined(__AVR_ATmega328P__)
		cli();
		TCCR1A = 0;
		TCCR1B = 0;
		OCR1A = 15624;
		TCCR1B |= (1 << WGM12);
		TCCR1B |= (1 << CS12);
		TCCR1B |= (1 << CS10);
		TIMSK1 &= 0xFD;
		sei();
#else
	#error "Detector service not designed to target microcontroller."
#endif
	}
	void on() {
		TIMSK1 |= (1 << OCIE1A);
		initV = GY9250.getA();
		Pressure.setThreshold();
		s = 1;
	}
	void off() {
		TIMSK1 &= 0xFD;
		s = 0;
	}
	bool status() { return s; }
} Detector(15, alert, non_alert);



uint32_t isr_now_time = 0, isr_old_time = 0, isr_interval = 100;
Vector3 now;

ISR(TIMER1_COMPA_vect) {
	isr_now_time = millis();
	if (isr_now_time - isr_old_time > isr_interval) {
		isr_old_time = isr_now_time;
		GY9250.updateData();
		now = GY9250.getA();
		Serial.print(Detector.initV.getX() - now.getX());
		Serial.print('\t');
		Serial.print(Detector.initV.getY() - now.getY());
		Serial.print('\t');
		Serial.print(Detector.initV.getZ() - now.getZ());
		Serial.print('\t');
		Serial.print(vt(Detector.initV.getMagnitude() - now.getMagnitude()));
		Serial.print('\t');
		Serial.print(now.getMagnitude());
		Serial.print('\t');
		Serial.println(analogRead(A5));
		if (abs(Detector.initV.getX() - now.getX())>Detector.threshold || abs(Detector.initV.getZ() - now.getZ())>Detector.threshold || Pressure.isPressed()) Detector.callback1();
		else Detector.callback2();
	}
}

SoftwareSerial BT(PIN_BT_RX, PIN_BT_TX);

// ������ ���� ��尡 ON�� ���¿��� �������� ������ �� ������ �ڵ带
// alert() �Լ� �ȿ� �ۼ��ϼ���.
// delay �Լ� ����� �����ϵ��� �մϴ�.
void alert() {
	alarm.on();
}

// ������ ���� ��忡�� �������� �������� ���� ������ �� ������ �ڵ带
// non_alert() �Լ� �ȿ� �ۼ��ϼ���.
// delay �Լ� ����� �����ϵ��� �մϴ�.
void non_alert() {
	alarm.off();
}

// ��ư�� ���� �� ������ �ڵ带
// button() �Լ� �ȿ� �ۼ��ϼ���.
// delay �Լ� ����� �����ϵ��� �մϴ�.
void button() {
	alarm.off();
	Detector.on();
}

// float mapping function
float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
	Serial.begin(9600);
	BT.begin(9600);
	pinMode(13, OUTPUT);
	pinMode(2, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(PIN_INT_BTN), button, FALLING);
	delay(100);

	// ��Ÿ �ʱ⿡ �� �� ������ �ڵ带 �Ʒ��� �ۼ��ϼ���.
	Detector.on();
}

void loop() {
	// ��������� �����Ͱ� ���� ��쿡 ������ �ڵ带
	// �� if�� �ȿ� �ۼ��ϼ���.
	if (BT.available()) {
		if (BT.read() == 'B') {
			Detector.off();
			alarm.off();
		}
	}
}