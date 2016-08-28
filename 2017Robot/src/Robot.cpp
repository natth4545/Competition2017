#include "WPILib.h"
#include <memory>

using std::shared_ptr;

class Robot: public IterativeRobot
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

		moveWheel1->SetControlMode(CANTalon::ControlMode::kSpeed);
		moveWheel2->SetControlMode(CANTalon::ControlMode::kSpeed);
		moveWheel3->SetControlMode(CANTalon::ControlMode::kSpeed);
		moveWheel4->SetControlMode(CANTalon::ControlMode::kSpeed);
		turnWheel1->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel2->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel3->SetControlMode(CANTalon::ControlMode::kPosition);
		turnWheel4->SetControlMode(CANTalon::ControlMode::kPosition);
		camMotor->SetControlMode(CANTalon::ControlMode::kPosition);

		moveWheel1->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel2->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel3->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		moveWheel4->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		turnWheel1->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		turnWheel2->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		turnWheel3->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
		turnWheel4->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
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

	void TeleopPeriodic()
	{

	}

	void DisabledPeriodic() override
	{

	}

  void calibrate()
  {

  }
	void UpdateDashboard()
	{

	}
	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
