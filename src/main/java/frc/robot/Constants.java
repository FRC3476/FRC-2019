package frc.robot;

public final class Constants {

	// Other
	public static final int SensorTicksPerMotorRotation = 4096;

	// CAN IDs
	public static final int LeftMasterDriveId = 16;
	public static final int LeftSlaveDriveId = 15;
	public static final int LeftSlave2DriveId = 14;
	public static final int RightMasterDriveId = 11;
	public static final int RightSlaveDriveId = 12;
	public static final int RightSlave2DriveId = 13;

	public static final int ElevatorMotorId = 24;
	public static final int ElevatorSlaveMotorId = 25;
	public static final int ArmId = 30;

	public static final int Climber1TalonId = 21;
  	public static final int Climber2TalonId = 26;
  
  // PCM IDs
  public static final int DriveShifterId = 0;

  // Controller
	public static final double MinControllerInput = 0.15;
	public static final double MaxControllerInput = 1;
	public static final double MinControllerOutput = 0;
	public static final double MaxControllerOutput = 1;
  public static final double MaxAcceleration = 1000;
  
  // General
	public static final double EncoderTicksPerRotation = 4096;
	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;
	public static final int xAxisJoystick = 4;
	public static final int yAxisJoystick = 1;

	// Autonomous Driving
	public static final double TrackRadius = 12;
	public static final double WheelDiameter = 6;
	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120;
	public static final double MinLookAheadDistance = 14;
 	public static final double MaxLookAheadDistance = 30;
  
  // Subsystems
  public static final int TimeoutMs = 10;

  // Arm
	public static final int ArmFeedbackSensorPidIdx = 0;
	public static final double ArmConfigKP = 6;
	public static final double ArmConfigKI = 0.0;
	public static final double ArmConfigKD = 2;
	public static final double ExpectedArmCurrent = 0;
	public static final double ExpectedArmRPM = 0;
	public static final double ExpectedArmPosition = 0;
	public static final double ArmDownDegrees = -38;
	public static final int PracticeBotArmTicksOffset = 4007;//4000;

  // Drive
	public static final double HighDriveSpeed = 185;
  	public static final double LowDriveSpeed = 95;
  
	public static final double kRightHighP = 0.02;
	public static final double kRightHighD = 0;
	public static final double kRightHighF = 0.035;
	public static final double kRightHighFIntercept = 0;
	public static final double kRightHighA = 0;
	public static final double kRightLowP = 0.1;
	public static final double kRightLowD = 0.1;
	public static final double kRightLowF = 0.05763730970902943999708309631717;
	public static final double kRightLowFIntercept = 0;
	public static final double kRightLowA = 0;

	public static final double kLeftHighP = 0.0;
	public static final double kLeftHighD = 0;
	public static final double kLeftHighF = 0.035;
	public static final double kLeftHighFIntercept = 0;
	public static final double kLeftHighA = 0;
	public static final double kLeftLowP = 0.1;
	public static final double kLeftLowD = 0;
	public static final double kLeftLowF = 0.05763730970902943999708309631717;
	public static final double kLeftLowFIntercept = 0;
	public static final double kLeftLowA = 0;

	public static final double kRightAutoP = 0.12;
	public static final double kRightAutoD = 0.7;
	public static final double kRightAutoF = 0.035;
	public static final double kLeftAutoP = 0.12;
	public static final double kLeftAutoD = 0.7;
	public static final double kLeftAutoF = 0.035;
	public static final double TeleopAccLimit = 120;
	public static final double TeleopJerkLimit = 2000;
	public static final double ExpectedDriveCurrent = 1.5;
	public static final double ExpectedDriveRPM = 0;
	public static final double ExpectedDrivePosition = 0;

	// Ground Intake	
	public static final double IntakeMotorPercentOutputIntake = -1;
	public static final double IntakeMotorPercentOutputOuttake = 0.275;
	public static final int IntakeSolenoidId = 10;//Random number for now
	public static final double IntakeMediumRPM = 700;//Random number for now
	public static final double IntakeFastRPM = 700;//Random number for now

	//Intake
	public static final int Intake1Id = 22;
	public static final int Intake2Id = 23;
	public static final int IntakeSolenoid30PsiId = 4;
	public static final int IntakeSolenoid60PsiId = 1;
	public static final double IntakeSpeed = 0.75; 

	//Elevator										
	public static final double HighElevatorHomeSpeed = 100;
	public static final double MidElevatorHomeSpeed = 50;
	public static final double LowElevatorHomeSpeed = 10;
	public static final double ElevatorInchesPerMotorRotation = 8;
	public static final double ElevatorTicksPerInch = 512;
	public static final int ElevatorSensorPidIdx = 0;

	public static final double LowElevatorAmps = 0;
	public static final double MaxElevatorAmps = 25;

	public static final int ELevatorIntegralZone = 1000;
	public static final double kElevatorP = 0.125;
	public static final double kElevatorI = 0.0;
	public static final double kElevatorD = 0.0;

	public static final double HatchPanelHeight = 2 + (1 / 6); // The height of each hatch panel
	public static final double ElevatorPositionDefault = 1 + (7 / 12);
	public static final double ElevatorPositionMiddle = ElevatorPositionDefault + HatchPanelHeight;
	public static final double ElevatorPositionHigh = ElevatorPositionDefault + (2 * HatchPanelHeight);

	private Constants() {
	}
}