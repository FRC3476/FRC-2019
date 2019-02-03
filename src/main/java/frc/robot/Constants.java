// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

public final class Constants {

	// CAN IDs
	public static final int DriveLeftMasterId = 16;
	public static final int DriveLeftSlave1Id = 15;
	public static final int DriveLeftSlave2Id = 14;
	public static final int DriveRightMasterId = 11;
	public static final int DriveRightSlave1Id = 12;
	public static final int DriveRightSlave2Id = 13;

	public static final int TurretMotorId = 2;
	
	public static final int ElevatorMasterId = 24;
	public static final int ElevatorSlaveId = 25;

	public static final int ManipulatorMotor1Id = 22;
	public static final int ManipulatorMotor2Id = 23;

	public static final int BallIntakeMasterId = 20;
	
	public static final int ClimberMasterId = 21;
	public static final int ClimberSlaveId = 26;

	public static final int HatchIntakeMotorId = 5;//Just a random number for now
	public static final int HatchHandoffId = 7;//Just a random number for now
	
	// PCM IDs
	public static final int DriveShifterSolenoidId = 0;
	public static final int ManipulatorSolenoidId = 4;
	
	// Controller
	public static final double MinControllerInput = 0.15;
	public static final double MaxControllerInput = 1;
	public static final double MinControllerOutput = 0;
	public static final double MaxControllerOutput = 1;
	public static final double MaxAcceleration = 1000;
	
	// General
	public static final double EncoderTicksPerRotation = 4096;
	public static final double DegreesPerEncoderTick = 360 * (1d / EncoderTicksPerRotation);
	public static final double EncoderTicksPerDegree = (1d / 360) * EncoderTicksPerRotation;

	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;

	// Game
	public static final double RocketBaseHeight = 27.5;
	public static final double RocketMiddleHeight = 55.5;
	public static final double RocketTopHeight = 83.5;

	public static final double HatchPanelHeight = 2 + (1 / 6); // The height of each hatch panel
	
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
	
	// Drive
	public static final double DriveHighSpeed = 185;
	public static final double DriveLowSpeed = 95;
	
	public static final double kDriveRightHighP = 0.02;
	public static final double kDriveRightHighD = 0;
	public static final double kDriveRightHighF = 0.035;
	public static final double kDriveRightHighFIntercept = 0;
	public static final double kDriveRightHighA = 0;
	public static final double kDriveRightLowP = 0.1;
	public static final double kDriveRightLowD = 0.1;
	public static final double kDriveRightLowF = 0.05763730970902943999708309631717;
	public static final double kDriveRightLowFIntercept = 0;
	public static final double kDriveRightLowA = 0;
	
	public static final double kDriveLeftHighP = 0.0;
	public static final double kDriveLeftHighD = 0;
	public static final double kDriveLeftHighF = 0.035;
	public static final double kDriveLeftHighFIntercept = 0;
	public static final double kDriveLeftHighA = 0;
	public static final double kDriveLeftLowP = 0.1;
	public static final double kDriveLeftLowD = 0;
	public static final double kDriveLeftLowF = 0.05763730970902943999708309631717;
	public static final double kDriveLeftLowFIntercept = 0;
	public static final double kDriveLeftLowA = 0;
	
	public static final double kDriveRightAutoP = 0.12;
	public static final double kDriveRightAutoD = 0.7;
	public static final double kDriveRightAutoF = 0.035;
	public static final double kDriveLeftAutoP = 0.12;
	public static final double kDriveLeftAutoD = 0.7;
	public static final double kDriveLeftAutoF = 0.035;
	public static final double DriveTeleopAccLimit = 120;
	public static final double DriveTeleopJerkLimit = 2000;
	public static final double DriveExpectedCurrent = 1.5;
	public static final double DriveExpectedRPM = 0;
	public static final double DriveExpectedPosition = 0;

	// Superstructure

	// Turret
	public static final int turretLimitId = 8;//Random channel for now
	public static final int maxTurretHomingAngle = 45;//Random degrees for now
	public static final double turretHomingSpeed = 0.2;//Random percent for now
	public static final double kTurretP = 0.125;
	public static final double kTurretI = 0.0;
	public static final double kTurretD = 0.0;

	//Arm
	public static final int ArmPuncherId = 4;//Random id

	// Elevator
	public static final double ElevatorHomeSpeed = -0.2;
	public static final double ElevatorInchesPerMotorRotation = 8;
	public static final double ElevatorTicksPerInch = 512;
	public static final int ElevatorSensorPidIdx = 0;
	
	public static final double ElevatorLowAmps = 0;
	public static final double ElevatorHighAmps = 25;
	public static final double ElevatorStallAmps = 3;
	
	public static final int ELevatorIntegralZone = 1000;
	public static final double kElevatorP = 0.125;
	public static final double kElevatorI = 0.0;
	public static final double kElevatorD = 0.0;
	
	public static final double ElevatorPositionDefault = 1 + (7 / 12);
	public static final double ElevatorPositionMiddle = ElevatorPositionDefault + HatchPanelHeight;
	public static final double ElevatorPositionHigh = ElevatorPositionDefault + (2 * HatchPanelHeight);
	
	public static final double ElevatorMaxHeight = 70;//in number for now
	public static final double ElevatorIntakeHeight = 10;//For now

	// Manipulator
	public static final double ManipulatorNormalSpeed = 0.75;
	public static final double ManipulatorLowSpeed = 0.50;

	// Ground Ball Intake	
	public static final double IntakeMotorPercentOutputIntake = -1;
	public static final double IntakeMotorPercentOutputEject = 0.275;
	public static final double IntakeMediumRPM = 700; // Random number for now
	public static final double IntakeFastRPM = 700; // Random number for now
	
	// Climber
	public static final double ClimberMaxAngle = 90;//Just a random number for now
	

	//HatchIntake
	public static final double HatchIntakeSpeed = 0.75;//Just a random percent for now
	public static final double HatchHandoffAngle = 90;
	public static final double HatchStowAngle = 0;
	public static final double HatchIntakeAngle = 180;

	private Constants() {
	}
}