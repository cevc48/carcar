/* --------------------------------------------------------------------/
This code has the sole aim of training with fusion filters and AHRS.
I make no claim about its fitness of purpose and the lack of errors.
Use it freely as you please, modify it, enhance it. Nonetheless, I take no
liability whatsoever regarding the use and eventual potential damage
from its utilization. In the making, I stood on the shoulders of Kris Winer
Kevin Townsend, Sebastian Madgwick, David McGriffy, Fabio Varesano (sadly, deceased)
and others. They have been my mentors and I'm grateful to them.
It has been tested on an ARDUINO MEGA 2650 with an ADAFRUIT BNO055 9 axis
absolute orientation sensor breakout board. Eventually, another ARDUINO and TEENSY 3.1/3.2
will do the job as well. The latter, demands the use of its own I2C library.
It needs also the NAxisMotion library from Bosch Sensortech Gmbh, modified by
Arduino.org development team and downloaded from this source.
The program has been developed in Visual Studio 2015 + Visual Micro ( www.visualmicro.com) extension.
If the Arduno IDE is used small ajustements must be made.
It outputs to the screen in CSV format the yaw, pitch and roll to
be handled in a Processing 3.0 application for visualization of a moving object.

Carlos Carvalho 02/02/2017
/------------------------------------------------------------------------*/
//MEGA 2650 ---------- ADAFRUIT BNO055
// 20 ----------------> SDA
// 21 ----------------> SCL
// D2 -----------------> INT
// 3.3 V --------------> 3Vd
//GND -----------------> GND

#include <Wire.h>
#include <NAxisMotion.h>

#define HARDFUSION false // BNO055 internal fusion engine disabled
#define LEDPIN 13
#define MADGWICK MadgwickAHRSUpdate(ax, ay, az,DEG_TO_RAD * gx, DEG_TO_RAD * gy, DEG_TO_RAD * gz, mx, my, mz);// Madgwick fusion engine
#define MAHONY MahonyAHRSUpdate(ax, ay, az, DEG_TO_RAD * gx, DEG_TO_RAD * gy, DEG_TO_RAD * gz, mx, my, mz);// Mahony fusion engine

#define ACC_1G (9.81f) // gravity - 9.81 m/s^2

NAxisMotion myAHRS; // Create an object

uint32_t lastUpdateTime = 0; // to hold the last time stamp
uint32_t lastOutputTime = 0;
const uint8_t updatePeriod = 10; // 100 Hz sensor update
const uint8_t outputPeriod = 500;  //2Hz output screen

float Acc[3] = { 0,0,0 }; // to hold raw data
float Gyro[3] = { 0,0,0 };// for acc,gyro
float Mag[3] = { 0,0,0 };// and mag

float gyroOff[3] = { 0,0,0 }; // store the calibration results
float accOff[3] = { 0,0,0 };// for gyro and acc

//---------------------- MAG FUNCTIONS ----------------------

static float magOff[3] = { 0,0,0 }; // to store mag offset values
float magBias[3] = { 0,0,0 };//to hold mag bias
float magTemp[3] = { 0,0,0 }; // to hold mag raw results

float magMax[3] = { 0,0,0 };// to hold mag max value
float magMin[3] = { 0,0,0 };// to hold mag min value
bool magCalibrated = false;
//----------------------------------------------------------
volatile float ax, ay, az, gx, gy, gz, mx, my, mz; // to store sensor values after calibration
 //--------------------- FUSION FILTERS PARAMETERS ----------
float GyroMeasError = (PI / 180.0f) * 40.0f; //gyro measurement error, starting @ 40 deg/s
float GyroMeasDrift = (PI / 180.0f) * 0.0f; // gyro drift in rad/s/s, starting @ 0 deg/s/s
float beta = sqrt(3.0f / 4.0f) * GyroMeasError;//compute beta
//----------------------------- MAHONY---------------------
#define Kp 2.0f * 5.0f //proportional feedback
#define Ki 0.0f //integral feedback
float eInt[3] = { 0.0f,0.0f,0.0f }; // vector to hold integral error for Mahony
//--------------------------------------------------------
const float deltat = ((float)updatePeriod / 1000.0f);//integration time interval (10ms /1000 = 10us)
 //-------------------------------------------------------
volatile float q[4] = { 1.0f,0.0f,0.0f,0.0f };// vector to hold quaternion
float ypr[3];//vector to hold yaw,pitch and roll angles

void setup()
{
	Serial.begin(115200);//initiate serial port
	I2C.begin();// I2C start
	Wire.setClock(400000);// set I2C clock to 400 KHz
	myAHRS.initSensor();//initiate BNO055
	myAHRS.setUpdateMode(AUTO);//default update mode
	pinMode(LEDPIN, OUTPUT);//Led pin

#if HARDFUSION
	myAHRS.resetSensor(BNO055_I2C_ADDR1);
	myAHRS.setOperationMode(OPERATION_MODE_NDOF); // Acc,Gyro, Mag with BNO055 internal fusion engine enabled
	delay(1000);
	uint8_t calib = myAHRS.readSystemCalibStatus();
	if (calib >= 0) digitalWrite(LEDPIN, HIGH);//if calibration OK light the Led!

#else
	myAHRS.setOperationMode(OPERATION_MODE_AMG); // Acc,Gyro, Mag without internal fusion
	CalibrateIMU();// do the acc, gyro calibration
	MagCalibration(); // do the magnetometer calibration
	Serial.println("-> Mag Calibration Done! <-");
	Serial.println("----------------------------");
	delay(3);
	float sensorSum = (ax + ay + (az - ACC_1G)) + (gx + gy + gz);
	if (magCalibrated && abs(sensorSum <= 1)) {
	digitalWrite(LEDPIN, HIGH);//if all the calibration done light the Led!
	}//end if

#endif
}


void loop()

{
	if ((millis() - lastUpdateTime) >= updatePeriod) {//start of processing loop
		if (!HARDFUSION) {//software fusion engine enabled
			ReadIMU(Acc, Gyro);//read acc and gyro raw values
			CalculateIMU();//read Acc,Gyro values after calibration
			CalculateMag();// Mag values after calibration
			// if Madgwick fusion algorithm used supress double slash
			//MADGWICK
			MAHONY // Mahony fusion algorithm used
			QuatToTaitBryanSoft(ypr); // calculate Tait-Bryan angles from quaternion
		}							 

		else {//BNO055 fusion engine enabled
			q[0] = myAHRS.readQuatW() / 1000.0f;//read quaternion value - W
			q[1] = myAHRS.readQuatX() / 1000.0f;//read quaternion value - X
			q[2] = myAHRS.readQuatY() / 1000.0f;//read quaternion value - Y
			q[3] = myAHRS.readQuatZ() / 1000.0f;//read quaternion value - Z

			float normQ = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
			q[0] *= normQ;
			q[1] *= normQ;
			q[2] *= normQ;
			q[3] *= normQ;

			QuatToTaitBryanHard(ypr);//  calculate Tait-Bryan angles from quaternion
		}
		lastUpdateTime = millis();
		//end of processing loop
	}

	if ((millis() - lastOutputTime) >= outputPeriod) {//start of display loop
		Serial.print(ypr[0]);//send output in CSV 
		Serial.print(",");
		Serial.print(ypr[1]);
		Serial.print(",");
		Serial.println(ypr[2]);

		lastOutputTime = millis();//end of display loop
	}
}

void ReadIMU(float *Acc, float *Gyro) {
	Acc[0] = myAHRS.readAccelX(); // read Acc raw values
	Acc[1] = myAHRS.readAccelY();
	Acc[2] = myAHRS.readAccelZ();

	Gyro[0] = myAHRS.readGyroX(); // read Gyro raw values
	Gyro[1] = myAHRS.readGyroY();
	Gyro[2] = myAHRS.readGyroZ();
}

void ReadMag(float *Mag) {
	Mag[0] = myAHRS.readMagX(); // read Mag raw values
	Mag[1] = myAHRS.readMagY();
	Mag[2] = myAHRS.readMagZ();
}

void CalibrateIMU() { // calibrate acc and gyro taking the average readings until vibration is low enough
	const uint8_t SAMPLESIZE = 128; 

	float gyroSum[3] = { 0,0,0 };
	float accSum[3] = { 0,0,0 };
	float sensorSum = 0.0f;

	for (uint8_t i = 0; i < 3; i++) { // clear sum store and wait a second
		gyroSum[i] = 0;
		accSum[i] = 0;
	}// endfor

	sensorSum = 0;
	delay(1000);//wait a second

	ReadIMU(Acc, Gyro); // read raw sensors
	CalculateIMU();

	for (uint8_t i = 0; i < SAMPLESIZE; i++) { // read sample
		delay(3);//wait 3 ms
		ReadIMU(Acc, Gyro); // read acc,gyro sensors

		for (uint8_t j = 0; j < 3; j++) {// and sum sensor axes values
			gyroSum[j] = Gyro[j];
			accSum[j] = Acc[j];
		}
	}// end for

	for (uint8_t k = 0; k < 3; k++) {
		gyroOff[k] = (gyroSum[k] / SAMPLESIZE);
		accOff[k] = (accSum[k] / SAMPLESIZE);
	}//end for
}//end calibrateIMU()

void CalculateIMU() {// acc and gyro values after calibration
	ax = Acc[0] - accOff[0];
	ay = Acc[1] - accOff[1];
	az = Acc[2] - accOff[2]; // gravity compensated
	//az = Acc[2] - ACC_1G; // same effect on z axis
	gx = Gyro[0] - gyroOff[0];
	gy = Gyro[1] - gyroOff[1];
	gz = Gyro[2] - gyroOff[2];
}

//------------------- MAG FUNCTIONS -----------------

void MagCalibration() {
	magCalibrated = false;
	uint8_t i = 0;
	const uint8_t SAMPLECOUNT = 128;
	Serial.println("Wave device in a figure of eight until done!");
	delay(2000); // 2s delay. 

	for (i = 0; i < SAMPLECOUNT; i++) {
		Read_Mag();

		for (int j = 0; j < 3; j++) {
			if (i == 0) (magMax[j] = magTemp[j]) && (magMin[j] = magTemp[j]);
			else	if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			else  if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}//end for j

		delay(25);// wait 25ms
	}//end for i

	magBias[0] = (magMax[0] + magMin[0]) / 2; //get average mag bias
	magBias[1] = (magMax[1] + magMin[1]) / 2;
	magBias[2] = (magMax[2] + magMin[2]) / 2;
	delay(3);

	//calculate the magnetometer offset
	magOff[0] = myAHRS.readMagX() - magBias[0];
	magOff[1] = myAHRS.readMagY() - magBias[1];
	magOff[2] = myAHRS.readMagZ() - magBias[2];
	magCalibrated = true;

	delay(3000);
}// end MagCalibration()
 //-----------------------------------------------
void Read_Mag() {//read mag raw values
	magTemp[0] = myAHRS.readMagX();
	magTemp[1] = myAHRS.readMagY();
	magTemp[2] = myAHRS.readMagZ();
}

void CalculateMag() {//calculate values after calibration
	mx = myAHRS.readMagX() - magOff[0];
	my = myAHRS.readMagY() - magOff[1];
	mz = myAHRS.readMagZ() - magOff[2];
	
}

//-------------------------------- FUSION FILTERS ---------------------

/* Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
(see http://www.x-io.co.uk/category/open-source/ for examples and more details)
which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
device orientation -- which can be converted to yaw, pitch, and roll. 
The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
but is much less computationally intensive*/

void MadgwickAHRSUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];

	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q[0] = q0 * recipNorm;
	q[1] = q1 * recipNorm;
	q[2] = q2 * recipNorm;
	q[3] = q3 * recipNorm;
}

// Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
// measured ones.
void MahonyAHRSUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float recipNorm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalize accelerometer measurement validate it (avoid NaN )
	if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
	}

	// Normalise magnetometer measurement and avoid NaN
	if ((mx != 0.0f) && (my != 0.0f) && (mz != 0.0f)) {
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;
	}
	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (Ki > 0.0f)
	{
		eInt[0] += ex;      // accumulate integral error
		eInt[1] += ey;
		eInt[2] += ez;
	}
	else
	{
		eInt[0] = 0.0f;     // prevent integral wind up
		eInt[1] = 0.0f;
		eInt[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + Kp * ex + Ki * eInt[0];
	gy = gy + Kp * ey + Ki * eInt[1];
	gz = gz + Kp * ez + Ki * eInt[2];

	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

	// Normalise quaternion
	recipNorm = invSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q[0] = q1 * recipNorm;
	q[1] = q2 * recipNorm;
	q[2] = q3 * recipNorm;
	q[3] = q4 * recipNorm;
}

void QuatToTaitBryanHard(float *ypr) {//calculate Tait-Bryan angles from quaternion(x+ forward,z+ downward, y+ left)
	ypr[0] = -RAD_TO_DEG * asin(2 * q[1] * q[3] + 2 * q[0] * q[2]); // roll
	ypr[1] = RAD_TO_DEG * atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1); // pitch
	ypr[2] = -RAD_TO_DEG * atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1); // yaw
	ypr[2] -= 2.42f; //declination @Espinho (2017)
}

void QuatToTaitBryanSoft(float *ypr) {//calculate Tait_Bryan angles from quaternion(turn x axis 90 degrees on the BNO055)
	ypr[0] = RAD_TO_DEG * atan2(2.0f * q[2] * q[3] - 2.0f * q[0] * q[1], 2.0f * q[0] * q[0] + 2.0f * q[3] * q[3] - 1.0f); // roll
	ypr[1] = RAD_TO_DEG * asin(2.0f * q[1] * q[3] + 2.0f * q[0] * q[2]); // pitch
	ypr[2] = RAD_TO_DEG * atan2(2.0f * q[1] * q[2] - 2.0f * q[0] * q[3], 2.0f * q[0] * q[0] + 2.0f * q[1] * q[1] - 1.0f); // yaw
	ypr[2] += 90.f;//turn x axis 90 degrees
	ypr[2] -= ACC_1G;// gravity
	ypr[2] -= 2.42; //declination @Espinho (2017)
}

//-----------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}