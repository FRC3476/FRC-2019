// Copyright 2019 FRC Team 3476 Code Orange

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

	public static final int ManipulatorMotor1Id = 22;
	public static final int ManipulatorMotor2Id = 23;
	
	public static final int Climber1TalonId = 21;
	public static final int Climber2TalonId = 26;
	
	// PCM IDs
	public static final int DriveShifterId = 0;

	public static final int ManipulatorSolenoidId = 4;
	
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
	public static final double RocketBaseHeight = 27.5;
	public static final double RocketMiddleHeight = 55.5;
	public static final double RocketTopHeight = 83.5;
	
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
	public static final double ArmLength = 18;//in Just a random number for now
	public static final int ArmFeedbackSensorPidIdx = 0;
	public static final double ArmConfigKP = 6;
	public static final double ArmConfigKI = 0.0;
	public static final double ArmConfigKD = 2;
	public static final double ExpectedArmCurrent = 0;
	public static final double ExpectedArmRPM = 0;
	public static final double ExpectedArmPosition = 0;
	public static final double ArmAngleLimit = 180;
	public static final double LowArmAmps = 0;
	public static final double HighArmAmps = 25;
	public static final double ArmHomingSpeed = 0.5;
	public static final double ArmRotationsPerMotorRotation = 1 / 1.5;
	public static final double ArmIntakingAngle = 45;//For now	
	public static final double ExpectedArmAngle = 180;
	
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
	public static final double IntakeMediumRPM = 700;//Random number for now
	public static final double IntakeFastRPM = 700;//Random number for now
	
	// Manipulator
	public static final double NormalManipulatorSpeed = 0.75;
	public static final double LowManipulatorSpeed = 0.50;
	
	//Elevator										
	public static final double ElevatorHomeSpeed = -0.2;
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
	
	public static final double MaxElevatorHeight = 70;//in number for now
	public static final double ElevatorIntakeHeight = 10;//For now
	
	//Climber
	public static final int ClimberSpark1 = 3;//Replace with ID
	public static final int ClimberSpark2 = 5;//Replace with ID
	public static final double MaxClimbingTicks = 4000;//Just a random number for now
	
	//Ball Intake
	public static final int BallIntakeTalonId = 6;//Random number for now.
	public static final double BallIntakePercentSpeed = 0.75;//Change when testing or if better idea
	
	// Turret
	public static final int TurretMotorId = 2; // Replace with port ID
	public static final double AngleConversionRate = 
	360 * (1d / SensorTicksPerMotorRotation);
	public static final double AngleConversionRate2 = 
	(1d / 360) * SensorTicksPerMotorRotation; // Figure out the difference between the two!
	private Constants() {
	}
}