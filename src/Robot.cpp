#include "WPILib.h"

RobotDrive *myRobot; // robot drive system
Talon *leftFront;
Talon *leftRear;
Talon *rightFront;
Talon *rightRear;
Compressor *roboCompressor;
Gyro *analogGyro;
AnalogInput *gyroInput;
Relay *spikeRelay;
Servo *ultrasonicServo;
Ultrasonic *sr04Ultrasonic;
DigitalOutput *digiOut;
DigitalInput *digiIn;
DoubleSolenoid *piston;
DigitalInput *limitSwitch;
float gyroAngle;
static const float Kp = 0.03;

class Robot : public IterativeRobot
{

	Joystick stick; // only joystick
	LiveWindow *lw;
	int autoLoopCounter;


public:
	Robot() :
		stick(0),		// as they are declared above.
		lw(NULL),
		autoLoopCounter(0)
	{
		leftFront = new Talon(0);
		leftRear = new Talon(1);
		rightFront = new Talon(2);
		rightRear = new Talon(3);
		roboCompressor = new Compressor(0);
		gyroInput = new AnalogInput(0);
		analogGyro = new Gyro(gyroInput);
		spikeRelay = new Relay(0);
		ultrasonicServo = new Servo(9);
		digiIn = new DigitalInput(8);
		digiOut = new DigitalOutput(9);
		sr04Ultrasonic = new Ultrasonic(digiOut, digiIn);
		piston = new DoubleSolenoid(1,0,7);
		limitSwitch = new DigitalInput(7);

		myRobot = new RobotDrive(leftFront, leftRear, rightFront, rightRear);	// these must be initialized in the same order
		myRobot->SetExpiration(2.0);
	}

private:
	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		piston->Set(DoubleSolenoid::kReverse);
		roboCompressor->SetClosedLoopControl(true);
		analogGyro->Reset();
		ultrasonicServo->SetAngle(90);
	}

	void AutonomousInit()
	{
		autoLoopCounter = 0;
	}

	void AutonomousPeriodic()
	{
		if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			gyroAngle = analogGyro->GetAngle();
			// Do something w/ the Angle
			myRobot->Drive(-0.5, -gyroAngle * Kp); 	// drive forwards half speed

			autoLoopCounter++;
		}
		else
		{
			myRobot->Drive(0.0, 0.0); // stop robot
		}
	}

	void TeleopInit()
	{
		roboCompressor->SetClosedLoopControl(true);
	}

	void TeleopPeriodic()
	{
		int offsetVal = 1;
		int servoAngle = 80;
		int passCount = 0;
		const int numLoopsPerScan = 250;
		float rightY;
		float leftY;

		ultrasonicServo->SetAngle(90);
		myRobot->SetSafetyEnabled(false);

		while (IsOperatorControl()) {

			rightY = stick.GetRawAxis(5);
			leftY =	stick.GetRawAxis(1) * -1.0;

			if ((rightY < 0.1) && (rightY > -0.1)) {
				rightY = 0;
			}

			if ((leftY < 0.1) && (leftY > -0.1)) {
				leftY = 0;
			}

			leftFront->Set(leftY);
			leftRear->Set(leftY);
			rightFront->Set(rightY);
			rightRear->Set(rightY);

			passCount++;
			if ((passCount % numLoopsPerScan) == 0) {
				offsetVal = 1;
			}
			else if (servoAngle >= 120) {
				offsetVal = -1;
			}
			else { // not sure if this was intended to be in an else, but it sorta looked like it
  			    servoAngle += offsetVal;
				ultrasonicServo->SetAngle(servoAngle);
				passCount = 0;
			}

			if (!(limitSwitch->Get())) {
				// It's closed now
			}//-r-
		}
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot);
