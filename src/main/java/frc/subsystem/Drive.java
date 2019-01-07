// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path;
import frc.utility.control.motion.PurePursuitController;
import frc.utility.math.Rotation2D;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class Drive extends Threaded {

	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, DONE
	}

	public static class DriveSignal {
		/*
		 * Inches per second for speed
		 */
		public double leftVelocity;
		public double rightVelocity;
		public double leftAcc;
		public double rightAcc;

		public DriveSignal(double left, double right) {
			this(left, 0, right, 0);
		}

		public DriveSignal(double left, double leftAcc, double right, double rightAcc) {
			leftVelocity = left;
			this.leftAcc = leftAcc;
			rightVelocity = right;
			this.rightAcc = rightAcc;
		}
	}

	public static class AutoDriveSignal {
		public DriveSignal command;
		public boolean isDone;

		public AutoDriveSignal(DriveSignal command, boolean isDone) {
			this.command = command;
			this.isDone = isDone;
		}
	}

	private static final Drive instance = new Drive();

	public static Drive getInstance() {
		return instance;
	}

	private double quickStopAccumulator;

	private boolean drivePercentVbus;

	private ADXRS450_Gyro gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Solenoid shifter;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;

	private Drive() {
		shifter = new Solenoid(Constants.DriveShifterId);
		leftTalon = new LazyTalonSRX(Constants.LeftMasterDriveId);
		rightTalon = new LazyTalonSRX(Constants.RightMasterDriveId);

		leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		leftSlaveTalon = new LazyTalonSRX(Constants.LeftSlaveDriveId);
		leftSlave2Talon = new LazyTalonSRX(Constants.LeftSlave2DriveId);
		rightSlaveTalon = new LazyTalonSRX(Constants.RightSlaveDriveId);
		rightSlave2Talon = new LazyTalonSRX(Constants.RightSlave2DriveId);
		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(1.0, 0, 1.2, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.HighDriveSpeed, -Constants.HighDriveSpeed);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.TeleopAccLimit);
		turnProfiler = new RateLimiter(100);

		configHigh();
	}

	private void configAuto() {
		rightTalon.config_kP(0, Constants.kRightAutoP, 10);
		rightTalon.config_kD(0, Constants.kRightAutoD, 10);
		rightTalon.config_kF(0, Constants.kRightAutoF, 10);
		leftTalon.config_kP(0, Constants.kLeftAutoP, 10);
		leftTalon.config_kD(0, Constants.kRightAutoD, 10);
		leftTalon.config_kF(0, Constants.kLeftAutoF, 10);
		driveMultiplier = Constants.HighDriveSpeed;
		rightTalon.configClosedloopRamp(12d / 200d, 10);
		leftTalon.configClosedloopRamp(12d / 200d, 10);
		
		//System.out.println(rightTalon.)
	}

	private void configHigh() {
		rightTalon.config_kP(0, Constants.kRightHighP, 10);
		rightTalon.config_kD(0, Constants.kRightHighD, 10);
		rightTalon.config_kF(0, Constants.kRightHighF, 10);
		rightTalon.configClosedloopRamp(12d / 200d, 10);
		leftTalon.config_kP(0, Constants.kLeftHighP, 10);
		leftTalon.config_kD(0, Constants.kRightHighD, 10);
		leftTalon.config_kF(0, Constants.kLeftHighF, 10);
		leftTalon.configClosedloopRamp(12d / 200d, 10);

		driveMultiplier = Constants.HighDriveSpeed;
	}

	private void configLow() {
		rightTalon.config_kP(0, Constants.kRightLowP, 10);
		rightTalon.config_kF(0, Constants.kRightLowF, 10);
		leftTalon.config_kP(0, Constants.kLeftLowP, 10);
		leftTalon.config_kF(0, Constants.kLeftLowF, 10);
		driveMultiplier = Constants.LowDriveSpeed;
	}

	public void arcadeDrive(double moveValue, double rotateValue) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}
		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= Constants.HighDriveSpeed;
			rightMotorSpeed *= Constants.HighDriveSpeed;
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	public void cheesyDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);

		double leftMotorSpeed;
		double rightMotorSpeed;
		double angularPower = 1;

		double overPower;

		if (isQuickTurn) {
			overPower = 1;
			if (moveValue < 0.2) {
				quickStopAccumulator = 0.9 * quickStopAccumulator + 0.1 * rotateValue * 2;
			}
			angularPower = rotateValue * 0.2;
		} else {
			overPower = 0;
			angularPower = Math.abs(moveValue) * rotateValue - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		// moveValue = moveProfiler.update(moveValue * driveMultiplier) /
		// driveMultiplier;
		leftMotorSpeed = moveValue + angularPower;
		rightMotorSpeed = moveValue - angularPower;

		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= overPower * (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= overPower * (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += overPower * (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += overPower * (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void orangeDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);
		// 50 is min turn radius
		double radius = (1 / rotateValue) + Math.copySign(24, rotateValue);
		double deltaSpeed = (Constants.TrackRadius * ((moveValue * driveMultiplier) / radius));
		deltaSpeed /= driveMultiplier;
		if (isQuickTurn) {
			deltaSpeed = rotateValue;
		}
		double leftMotorSpeed = moveValue + deltaSpeed;
		double rightMotorSpeed = moveValue - deltaSpeed;
		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	private void configMotors() {
		leftSlaveTalon.set(ControlMode.Follower, Constants.LeftMasterDriveId);
		leftSlave2Talon.set(ControlMode.Follower, Constants.LeftMasterDriveId);
		rightSlaveTalon.set(ControlMode.Follower, Constants.RightMasterDriveId);
		rightSlave2Talon.set(ControlMode.Follower, Constants.RightMasterDriveId);
		setBrakeState(NeutralMode.Brake);

		leftTalon.setInverted(true);
		leftSlaveTalon.setInverted(true);
		leftSlave2Talon.setInverted(true);

		rightTalon.setInverted(false);
		rightSlaveTalon.setInverted(false);
		rightSlave2Talon.setInverted(false);

		leftTalon.setSensorPhase(false);
		rightTalon.setSensorPhase(false);

		rightTalon.setNeutralMode(NeutralMode.Brake);
		leftTalon.setNeutralMode(NeutralMode.Brake);
		rightSlaveTalon.setNeutralMode(NeutralMode.Brake);
		leftSlaveTalon.setNeutralMode(NeutralMode.Brake);
		rightSlave2Talon.setNeutralMode(NeutralMode.Brake);
		leftSlave2Talon.setNeutralMode(NeutralMode.Brake);
	}

	public void resetMotionProfile() {
		moveProfiler.reset();
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(gyroSensor.getAngle());
	}

	public double getLeftDistance() {
		return leftTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.WheelDiameter
				* Math.PI * 22d / 62d / 3d;
	}

	public double getRightDistance() {
		return rightTalon.getSelectedSensorPosition(0) / Constants.SensorTicksPerMotorRotation * Constants.WheelDiameter
				* Math.PI * 22d / 62d / 3d;
	}

	public double getSpeed() {
		return ((leftTalon.getSelectedSensorVelocity(0) + rightTalon.getSelectedSensorVelocity(0))
				/ Constants.SensorTicksPerMotorRotation) / 10 / 2 * Constants.WheelDiameter * Math.PI;
	}

	public double getLeftSpeed() {
		return leftTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10
				* Constants.WheelDiameter * Math.PI * 22d / 62d / 3d;
	}

	public double getRightSpeed() {
		return rightTalon.getSelectedSensorVelocity(0) / Constants.SensorTicksPerMotorRotation * 10
				* Constants.WheelDiameter * Math.PI * 22d / 62d / 3d;
	}

	public double scaleJoystickValues(double rawValue) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinimumControllerInput,
				Constants.MaximumControllerInput, Constants.MinimumControllerOutput, Constants.MaximumControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		configAuto();
		updatePurePursuit();
	}

	public void setBrakeState(NeutralMode mode) {
		leftTalon.setNeutralMode(mode);
		rightTalon.setNeutralMode(mode);
		leftSlaveTalon.setNeutralMode(mode);
		rightSlaveTalon.setNeutralMode(mode);
		leftSlave2Talon.setNeutralMode(mode);
		rightSlave2Talon.setNeutralMode(mode);
	}

	public double getVoltage() {
		return (leftTalon.getMotorOutputVoltage() + rightTalon.getMotorOutputVoltage()
				+ leftSlaveTalon.getMotorOutputVoltage() + rightSlaveTalon.getMotorOutputVoltage()
				+ rightSlave2Talon.getMotorOutputVoltage() + leftSlave2Talon.getMotorOutputVoltage()) / 6;
	}

	private void setWheelPower(DriveSignal setVelocity) {
		leftTalon.set(ControlMode.PercentOutput, setVelocity.rightVelocity);
		rightTalon.set(ControlMode.PercentOutput, setVelocity.leftVelocity);
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > Constants.HighDriveSpeed
				|| Math.abs(setVelocity.leftVelocity) > Constants.HighDriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.HighDriveSpeed + " !", false);
			return;
		}
		// System.out.println("Left: " + setVelocity.leftWheelSpeed + " Speed:"
		// + getLeftSpeed());
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.rightVelocity) * 4096 / (Constants.WheelDiameter * Math.PI * 10)
				* (62d / 22d) * 3d;
		double rightSetpoint = (setVelocity.leftVelocity) * 4096 / (Constants.WheelDiameter * Math.PI * 10) * (62 / 22d)
				* 3d;
		leftTalon.set(ControlMode.Velocity, leftSetpoint);
		rightTalon.set(ControlMode.Velocity, rightSetpoint);
	}

	public synchronized void setSimpleDrive(boolean setting) {
		drivePercentVbus = setting;
	}

	@Override
	public void update() {
		DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
				break;
			case PUREPURSUIT:
				updatePurePursuit();
				break;
			case TURN:
				updateTurn();
				break;
		}
	}

	public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
		configHigh();
	}

	private void updateTurn() {
		double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().rotationMat.inverse()).getDegrees();
		double deltaSpeed;
		System.out.println(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees());
		System.out.println("error: " + error);
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
				OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.HighDriveSpeed), deltaSpeed);
		if (Math.abs(error) < 2) {
			setWheelVelocity(new DriveSignal(0, 0));
			synchronized (this) {
				driveState = DriveState.DONE;
			} 
		} else {
			setWheelVelocity(new DriveSignal(-deltaSpeed, deltaSpeed));
		}
	}

	public void setShiftState(boolean state) {
		shifter.set(state);
		if (state) {
			configLow();
		} else {
			configHigh();
		}
	}

	private void updatePurePursuit() {
		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if (signal.isDone) {
			synchronized (this) {
				driveState = DriveState.DONE;
			}
			configHigh();
		}
		setWheelVelocity(signal.command);
	}

	public void resetGyro() {
		gyroSensor.reset();
	}

	public boolean checkSubsystem() {

		// TODO: Get accurate thresholds
		// TODO: Use PDP to get current
		// boolean success =
		boolean success = leftTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0;
		success = rightTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0 && success;
		success = OrangeUtility.checkMotors(.25, Constants.ExpectedDriveCurrent, Constants.ExpectedDriveRPM,
				Constants.ExpectedDrivePosition, rightTalon, rightTalon, rightSlaveTalon, rightSlave2Talon);
		success = OrangeUtility.checkMotors(.25, Constants.ExpectedDriveCurrent, Constants.ExpectedDriveRPM,
				Constants.ExpectedDrivePosition, leftTalon, leftTalon, leftSlaveTalon, leftSlave2Talon) && success;
		configMotors();
		return success;
	}

	public void stopMovement() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
		driveState = DriveState.TELEOP;
	}

	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE;
	}

	public void clearStickyFaults() {
		leftTalon.clearStickyFaults(10);
		leftSlaveTalon.clearStickyFaults(10);
		leftSlave2Talon.clearStickyFaults(10);
		rightTalon.clearStickyFaults(10);
		rightSlaveTalon.clearStickyFaults(10);
		rightSlave2Talon.clearStickyFaults(10);
	}
}
