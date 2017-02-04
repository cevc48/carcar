//******************************************************************//
//               ***   Carlos Carvalho - Feb 2017  ***              //
//                                                                  //
//	PWM(3)>-------------------.										                  //
//                            \                                     //
//                  pot 100k  /    100R ____                        //
//							  						\<----- |____|----.---> analog(0)     //
//                            /              __|__                  //
//                            \             _____ 1000 uF           //
//                            |               |					            //
//                         	///             ///                     //
//******************************************************************//
/*Low pass filter to test PID response
  PID controller in a ARDUINO UNO/MEGA 2650 with a low pass filter in a feedback circuit.
  The input is on the analog A/D converter 0 and the PWM output on the pin 3 */

#define INPIN 0
#define OUTPIN 3

const float epsilon = 0.01;//minimum error
const float deltat = 0.01;//integration interval
const float sp = 190.0f; //set point 
float PID;

typedef struct {
	const uint8_t MAX;// windup max value
	const uint8_t MIN;//windup min value
	const float KP; // proportional gain
	const float KD;// derivative gain
	const float KI;// integral gain

}PID_t;


PID_t myPID = { 255,0,2.0,0.01,0.5 };//create a PID_t variable
void ComputePID(const float &setPoint, float input, float &output);//calculate PID output



void setup()
{

	Serial.begin(9600);

}

void loop()
{

	float myInput = (analogRead(INPIN) / 4);// set a value within the available range
	ComputePID(sp, myInput, PID);//calculate PID
	delay(50); // print slowly
	//Serial.println();
	//Serial.println("SetPoint-Input-Output"); // print PID to the serial monitor
	//Serial.println("---------------------");
	Serial.print(sp, 1);//print set point
	Serial.print(",");
	Serial.print(myInput, 1);//print input value
	Serial.print(",");
	Serial.println(PID);// print output variable

}


void ComputePID(const float &setPoint, float input, float &output) {//function to compute PID 


	
	static float lastError = 0;
	static float integral = 0;
	float error;
	float derivative;
	

	//Calculate P,I,D

	
	error = setPoint - input;

	// in case the error is too small stop integration

	if (abs(error) > epsilon) {

		//integral += error * deltat; // rectangular integration
		integral += (error + lastError) * 0.5f * deltat; // trapezoidal rule

	}

	derivative = (error - lastError) / deltat;
	output = myPID.KP * error + myPID.KI * integral + myPID.KD * derivative;//output value 

	//windup guard
	if (output > myPID.MAX) {
		output = myPID.MAX;
	}
	else if(output < myPID.MIN){

		output = myPID.MIN;
	}

	// update error

	lastError = error;
	analogWrite(OUTPIN, output);

}
