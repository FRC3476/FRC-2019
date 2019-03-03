// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

public final class Constants {
	// Networking
	public static final int TelemetryPort = 5801;
	public static final String DriverStationIPv4 = "10.34.76.5";
	public static final int JetsonPort = 8000;
	public static final String JetsonIPv4 = "10.34.76.72";

	// CAN IDs
	public static final int DriveLeftMasterId = 16;
	public static final int DriveLeftSlave1Id = 15;
	public static final int DriveLeftSlave2Id = 14;
	public static final int DriveRightMasterId = 11;
	public static final int DriveRightSlave1Id = 12;
	public static final int DriveRightSlave2Id = 13;

	public static final int BallIntakeMasterId = 20;

	public static final int HatchIntakeMotorId = 5;
	public static final int HatchIntakeDeployMotorId = 5;

	public static final int ElevatorMasterId = 9;
	public static final int ElevatorSlaveId = 8;

	public static final int ManipulatorMotor1Id = 10;
	public static final int ManipulatorMotor2Id = 11;
	
	public static final int ClimberMasterId = 21;
	public static final int ClimberSlaveId = 26;
	
	// PCM IDs
	public static final int DriveShifterSolenoidId = 4;
	public static final int BallIntakeSolenoidId = 5;
	public static final int ArmSolenoidId = 6;
	public static final int ManipulatorSolenoidId = 7;

	// IO IDs
	public static final int TurretLimitId = 0;
	
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
	public static final double TrackRadius = -12;
	public static final double WheelDiameter = 6;
	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120;
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;
	
	// Subsystems
	public static final int TimeoutMs = 10;
	
	// Drive
	public static final double maxTurnError = 2;
	public static final double maxPIDStopSpeed = 8;
	public static final double DriveHighSpeed = 215;
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
	
	public static final double kDriveRightAutoP = 0.00065; //0.15
	public static final double kDriveRightAutoD = 0.0001; //0.7
	public static final double kDriveRightAutoF = 0.0000; //0.055
	public static final double kDriveLeftAutoP = 0.00065;
	public static final double kDriveLeftAutoD = 0.0001;
	public static final double kDriveLeftAutoF = 0.0000; //0.0005 too high
	public static final double DriveTeleopAccLimit = 120;
	public static final double DriveTeleopJerkLimit = 2000;
	public static final double DriveExpectedCurrent = 1.5;
	public static final double DriveExpectedRPM = 0;
	public static final double DriveExpectedPosition = 0;

	// Superstructure

	// Ground Ball Intake	
	public static final double IntakeMotorPowerIntake = 1;
	public static final double IntakeMotorPowerEject = 0.275;
	public static final double IntakeMediumRPM = 700; // Random number for now
	public static final double IntakeFastRPM = 700; // Random number for now
	public static final long IntakeDeployTime = 0;

	// Hatch Intake
	public static final double HatchIntakeMotorPower = 0.75;//Just a random percent for now
	public static final double HatchHandoffAngle = 90;
	public static final double HatchStowAngle = 0;
	public static final double HatchIntakeAngle = 180;

	// Turret
	public static final int TurretCollisionRange = 0;
	public static final double maxTurretOverTravel = 15;
	public static final int turretLimitId = 8;//Random channel for now
	public static final int maxTurretHomingAngle = 45;//Random degrees for now
	public static final double turretHomingSpeed = 0.2;//Random percent for now
	public static final int TurretMotorId = 7;
//	public static final double kTurretP = 0.25;

	public static final int TurretMaxHomingAngle = 45;//Random degrees for now
	public static final double TurretHomingPower = 0.4;//Random percent for now
	public static final double kTurretP = 0.4;


	public static final double kTurretI = 0.00;
	public static final double kTurretD = 0.0;

	// Elevator
	public static final double ElevatorHomeSpeed = -0.1;
	public static final double ElevatorInchesPerMotorRotation = 8;
	public static final double ElevatorTicksPerInch = 4096.0/(1.5*3.141592);
	public static final int ElevatorSensorPidIdx = 0;
	public static final double ElevatorTargetError = 10;
	
	public static final double ElevatorLowAmps = 0;
	public static final double ElevatorHighAmps = 25;
	public static final double ElevatorStallAmps = 3;
	
	public static final int ELevatorIntegralZone = 1000;
	public static final double kElevatorP = 0.20;
	public static final double kElevatorI = 0.0;
	public static final double kElevatorD = 0.0;
	
	public static final double ElevatorIntakeSafe = 0;
	public static final double ElevatorDeployingSafe = 0;
	public static final double ElevatorSafetyError = 0;
	public static final double ElevatorPositionDefault = 1 + (7 / 12);
	public static final double ElevatorPositionMiddle = ElevatorPositionDefault + HatchPanelHeight;
	public static final double ElevatorPositionHigh = ElevatorPositionDefault + (2 * HatchPanelHeight);
	public static final double ElevatorPositionLow = 0;
	
	public static final double ElevatorMaxHeight = 70;//in number for now
	public static final double ElevatorIntakeHeight = 10;//For now

	// Manipulator
	public static final double ManipulatorNormalPower = 0.75;
	public static final double ManipulatorLowPower = 0.50;
	
	// Climber
	public static final double ClimberMaxAngle = 90;//Just a random number for now
	
	private Constants() {
	}
}