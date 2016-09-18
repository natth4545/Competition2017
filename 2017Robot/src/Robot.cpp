#include "WPILib.h"
#include "AHRS.h"
#include <math.h>
#include <memory>

#define PI 3.14159265

using std::shared_ptr;

class Robot: public IterativeRobot, public PIDOutput
{

private:
	LiveWindow *lw = nullptr;
	SendableChooser *chooser;
	const std::string autoNameDefault = "Default";
	const std::string boxAuto = "Box Auto";

	CANTalon *moveWheel1 = new CANTalon(0);
	CANTalon *moveWheel2 = new CANTalon(1);
	CANTalon *moveWheel3 = new CANTalon(2);
	CANTalon *moveWheel4 = new CANTalon(3);
	CANTalon *turnWheel1 = new CANTalon(4);
	CANTalon *turnWheel2 = new CANTalon(5);
	CANTalon *turnWheel3 = new CANTalon(6);
	CANTalon *turnWheel4 = new CANTalon(7);
	CANTalon *camMotor = new CANTalon(8);
	/* CANJaguar
	CANJaguar *turnWheel1 = new CANJaguar(0);
	CANJaguar *turnWheel2 = new CANJaguar(1);
	CANJaguar *turnWheel3 = new CANJaguar(2);
	CANJaguar *turnWheel4 = new CANJaguar(3);
  */

	Joystick *leftJoystick = new Joystick(0);
	Joystick *rightJoystick = new Joystick(1);
	Joystick *controlJoystick = new Joystick(2);

  PowerDistributionPanel *pdp = new PowerDistributionPanel(0);

	AHRS *ahrs;
	PIDController *turnController;

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
		chooser = new SendableChooser();
		chooser->AddDefault(autoNameDefault, (void*)&autoNameDefault);
		chooser->AddObject(boxAuto, (void*)&boxAuto);
		SmartDashboard::PutData("Auto Modes", chooser);

		//RobotDrive *drive = new RobotDrive(moveWheel1,moveWheel2,moveWheel3,moveWheel4);
		//drive->SetExpiration(20000);
		//drive->SetSafetyEnabled(false);

    //The encoders and values are placeholders for later
		moveWheel1->SetSafetyEnabled(false);
		moveWheel2->SetSafetyEnabled(false);
		moveWheel3->SetSafetyEnabled(false);
		moveWheel4->SetSafetyEnabled(false);
		turnWheel1->SetSafetyEnabled(false);
		turnWheel2->SetSafetyEnabled(false);
		turnWheel4->SetSafetyEnabled(false);
		turnWheel1->SetSafetyEnabled(false);
		camMotor->SetSafetyEnabled(false);

		moveWheel1->SetExpiration(200000);
		moveWheel2->SetExpiration(200000);
		moveWheel3->SetExpiration(200000);
		moveWheel4->SetExpiration(200000);
		turnWheel1->SetExpiration(200000);
		turnWheel2->SetExpiration(200000);
		turnWheel4->SetExpiration(200000);
		turnWheel1->SetExpiration(200000);
		camMotor->SetExpiration(200000);

		moveWheel1->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		moveWheel2->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		moveWheel3->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		moveWheel4->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		turnWheel1->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel2->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel3->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel4->SetControlMode(CANTalon::ControlMode::kPosition);
		camMotor->SetControlMode(CANTalon::ControlMode::kPosition);

		/*
		moveWheel1->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel2->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel3->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel4->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		*/
		turnWheel1->SetFeedbackDevice(CANTalon::FeedbackDevice::AnalogEncoder);
		turnWheel2->SetFeedbackDevice(CANTalon::FeedbackDevice::AnalogEncoder);
		turnWheel3->SetFeedbackDevice(CANTalon::FeedbackDevice::AnalogEncoder);
		turnWheel4->SetFeedbackDevice(CANTalon::FeedbackDevice::AnalogEncoder);
		camMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);

		moveWheel1->EnableControl();
		moveWheel2->EnableControl();
		moveWheel3->EnableControl();
		moveWheel4->EnableControl();
    turnWheel1->EnableControl();
    turnWheel2->EnableControl();
    turnWheel3->EnableControl();
    turnWheel4->EnableControl();
		camMotor->EnableControl();

		//Gyroscope stuff
		try {
			/* Communicate w/navX-MXP via the MXP SPI Bus.                                       */
			/* Alternatively:  I2C::Port::kMXP, SerialPort::Port::kMXP or SerialPort::Port::kUSB */
			/* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details.   */
			ahrs = new AHRS(SPI::Port::kMXP);
		} catch (std::exception ex) {
			std::string err_string = "Error instantiating navX-MXP:  ";
			err_string += ex.what();
			//DriverStation::ReportError(err_string.c_str());
		}

		if (ahrs) {
			LiveWindow::GetInstance()->AddSensor("IMU", "Gyro", ahrs);
			ahrs->ZeroYaw();

			// Kp	  Ki	 Kd		Kf    PIDSource PIDoutput
			/*
			turnController = new PIDController(0.015f, 0.003f, 0.100f, 0.00f,
					ahrs, this);
			turnController->SetInputRange(-180.0f, 180.0f);
			turnController->SetOutputRange(-1.0, 1.0);
			turnController->SetAbsoluteTolerance(2); //tolerance in degrees
			turnController->SetContinuous(true);
			*/
		}
	}


	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	double rotateRate = 0;
	void PIDWrite(float output) { // Implement PIDOutput
		rotateRate = output;
	}

    int currentState = 1;
    int autoSelected = 0;
	Timer *timer = new Timer();
	void AutonomousInit()
	{
		autoSelected = *((int*) chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;
	}

	void AutonomousPeriodic()
	{
    UpdateDashboard();
		switch(autoSelected)
    {
      default:
      case 0:
        //do jack
        break;
      case 1:
        AutonomousBox();
        break;
    }
	}

  void AutonomousBox()
  {
    // Do a marching band box to the left

    switch(currentState)
    {
      case 1:
        timer->Reset();
        timer->Start();
        //State: Stopped
        //Transition: Driving State, forward 8 feet
        currentState = 2;
        break;
      case 2:
        //State: Driving
        //Currently: 8 feet Forward
        //Transition: Driving, left 8 feet

    }
  }

	void TeleopInit()
	{

	}

	bool dMode = true; // true is true swerve, false is robot oriented swerve
	bool r1 = false,r2 = false,r3 = false,r4 = false; // this is if we need to turn the wheel a lot
	//for example, if you need to do a 180 it's easier to reverse
	//int p1=0,p2=0,p3=0,p4=0; // position of wheel angle motors to track their position
	float currentAngle = 0, angleDerivative; // manual math

	void TeleopPeriodic() override
	{
			float temp, theta, xRate, yRate, rotateRate; // values of forward movement and rotary movement
			float A,B,C,D; //math variables
			float ws1,ws2,ws3,ws4; // wheel speeds
			float wa1,wa2,wa3,wa4; // wheel angles
			float camAng; // camera angle
			float cwa1,cwa2,cwa3,cwa4; // current wheel angles
			float max; // some more wheel balancing math
			/*
			1 - 4 are quadrants 1 - 4 respectively
			*/

			xRate = rightJoystick->GetX(); // Get values of movement
			yRate = rightJoystick->GetY() * -1;
			rotateRate = leftJoystick->GetX();

			float multiplier; // TURBO modo
			if(rightJoystick->GetRawButton(1))
			{
				multiplier = 1;
			} else {
				multiplier = .65;
			}

			// temp
			// FWD = yRate STR = xRate
			theta = ahrs->GetYaw();
			theta = theta * PI / 180.0; // degrees to radians
			/*
			TODO: manual math to calculate current angle of robot to do checking aginst gyro
			*/



			temp = yRate * cos(theta) + xRate * sin(theta);
			xRate = -1 * yRate * sin(theta) + xRate * cos(theta);
			yRate = temp;

			A = xRate - rotateRate;
			B = xRate + rotateRate;
			C = yRate - rotateRate;
			D = yRate + rotateRate;

			ws1 = sqrt(pow(B,2.0) + pow(C,2.0));
			ws2 = sqrt(pow(B,2.0) + pow(D,2.0));
			ws3 = sqrt(pow(A,2.0) + pow(D,2.0));
			ws4 = sqrt(pow(A,2.0) + pow(C,2.0));

			wa1 = atan2(B, C) * (180/PI) % 360;
			wa2 = atan2(B, D) * (180/PI) % 360;
			wa3 = atan2(A, D) * (180/PI) % 360;
			wa4 = atan2(A, C) * (180/PI) % 360;

			max = ws1;
			if(ws2 > max)
			{
				max = ws2;
			}
			if(ws3 > max)
			{
				max = ws3;
			}
			if(ws4 > max)
			{
				max = ws4;
			}

			if(max > 1)
			{
				ws1 /= max;
				ws2 /= max;
				ws3 /= max;
				ws4 /= max;
			}

			//math is done, now we set our motors

			//ANALOG ENCODERS NEED TO HAVE THEIR POSITION TRACKED BECAUSE THEY ARE CONTINUOUS
			// maybe % 360 will work
			cwa1 = turnWheel1->Get() % 360;
			cwa2 = turnWheel2->Get() % 360;
			cwa3 = turnWheel3->Get() % 360;
			cwa4 = turnWheel4->Get() % 360;

			// need logic to make turning wheels more efficient
			// here we turn the angle of the wheel around and reverse the move motors
			// TODO: more logic is needed to help reverse wheel angle motors maybe

			if(abs(wa1-cwa1) > 90 && abs(wa1-cwa1) < 270)
			{
				wa1 = wa1 + 180 % 360; // reverse the angle
				r1 = !r1;
				moveWheel1->SetInverted(r1);
			}
			if(abs(wa2-cwa2) > 90 && abs(wa2-cwa2) < 270)
			{
				wa2 = wa2 + 180 % 360; // reverse the angle
				r2 = !r2;
				moveWheel1->SetInverted(r2);
			}
			if(abs(wa3-cwa3) > 90 && abs(wa3-cwa3) < 270)
			{
				wa3 = wa3 + 180 % 360; // reverse the angle
				r3 = !r3;
				moveWheel1->SetInverted(r3);
			}
			if(abs(wa4-cwa4) > 90 && abs(wa4-cwa4) < 270)
			{
				wa4 = wa4 + 180 % 360; // reverse the angle
				r4 = !r4;
				moveWheel1->SetInverted(r4);
			}

			// convert wheel angles in degrees to encoder values
			// 0-359 degrees -> 0-1023 encoder ticks

			wa1 = wa1/360 * 1023;
			wa2 = wa2/360 * 1023;
			wa3 = wa3/360 * 1023;
			wa4 = wa4/360 * 1023;

			turnWheel1->Set(wa1); // setting our turn wheel motors
			turnWheel2->Set(wa2);
			turnWheel3->Set(wa3);
			turnWheel4->Set(wa4);

			moveWheel1->Set(ws1 * multiplier); // setting our move motors
			moveWheel2->Set(ws2 * multiplier);
			moveWheel3->Set(ws3 * multiplier);
			moveWheel4->Set(ws4 * multiplier);

			motorSpeed();

			camMotor->Set(-1 * theta); // currently in mode field centric, I'll do more logic later
	}

	void DisabledPeriodic() override
	{
			UpdateDashboard();
	}

	void UpdateDashboard()
	{

	}

	void motorSpeed()
	{
		float voltage = moveWheel1->GetBusVoltage();
		double speed = moveWheel1->GetSpeed();

		printf("%d / %f", speed, voltage);
	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
