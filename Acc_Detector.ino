#include <SoftwareSerial.h>
#include "MPU9250.h"
#include "I2Cdev.h"
#include "pin_definition.h"

// 이 선언이 있으면 부저가 울리기 시작한 1초 뒤에 자동으로 부저가 꺼집니다.
//#define ALARM_AUTO_OFF

// 이 선언이 있으면 가속도 센서 입력에 Low Pass Filter가 적용됩니다.
#define ENABLE_ACC_LOW_PASS
#if defined(ENABLE_ACC_LOW_PASS)
#define ts 0.03
#define tau 1/(0.8*2*3.14)
#define ttt ts/(tau+ts)
#endif

void alert();
void non_alert();

typedef float vt;
// Vector3 클래스
// 3차원 벡터값을 관리하는 클래스입니다.
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
	// 벡터의 크기를 구합니다.
	float getMagnitude() { return sqrt(x*x + y*y + z*z); }

	// 벡터의 단위 벡터를 구합니다.
	Vector3 getDirection() { return *this / getMagnitude(); }

	// x축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosAlpha() { return (float)x / getMagnitude(); }

	// y축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosBeta() { return (float)y / getMagnitude(); }

	// z축과 벡터 사이의 각도의 cos 값을 구합니다.
	float getDirCosGamma() { return (float)z / getMagnitude(); }


	// 벡터 연산

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

// GY9250 클래스
// 9축(가속도3축, 자이로3축, 자기장3축) 센서를 관리하는 클래스입니다.
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

	// 센서 선택
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

	// enable 된 센서의 값을 읽고 업데이트합니다.
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
	void off() { noTone(pin); Serial.println("alarm off"); }
} alarm(PIN_BUZZER, 1000);

// Pressure 클래스
// 압력센서를 관리하는 클래스입니다.
// 선언과 동시에 초기화되어서 따로 초기화해줄 필요는 없습니다.
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


// Detector 클래스
// 움직임 감지 모드를 관리하는 클래스입니다.
// 선언과 동시에 초기화되어서 따로 초기화해줄 필요는 없습니다.
// Detector.on() 함수로 감지 모드를 켜고
// Detector.off() 함수로 감지 모드를 끌 수 있습니다.
// 감지 모드가 켜져을 때 움직임이 감지되면 alert() 함수를 실행합니다.
// Detector.on() 함수를 실행하면 현재의 가속도 값과 무게 값을 새롭게 기억합니다.
// 감지모드 작동 중일 때 이 기억한 값과 일정 이상 차이가 나면 alert() 함수가 실행됩니다.
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

// 움직임 감지 모드가 ON인 상태에서 움직임이 감지될 때 실행할 코드를
// alert() 함수 안에 작성하세요.
// delay 함수 사용은 자제하도록 합니다.
void alert() {
	alarm.on();
}

// 움직임 감지 모드에서 움직임이 감지되지 않은 상태일 때 실행할 코드를
// non_alert() 함수 안에 작성하세요.
// delay 함수 사용은 자제하도록 합니다.
void non_alert() {
	alarm.off();
}

// 버튼이 눌릴 때 실행할 코드를
// button() 함수 안에 작성하세요.
// delay 함수 사용은 자제하도록 합니다.
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

	// 기타 초기에 한 번 실행할 코드를 아래에 작성하세요.
	Detector.on();
}

void loop() {
	// 블루투스로 데이터가 들어온 경우에 실행할 코드를
	// 이 if문 안에 작성하세요.
	if (BT.available()) {
		if (BT.read() == 'B') {
			Detector.off();
			alarm.off();
		}
	}
}