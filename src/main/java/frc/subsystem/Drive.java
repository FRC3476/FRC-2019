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

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.utility.LazyCANSparkMax;

public class Drive extends Threaded {

	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE
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

	private ADXRS450_Gyro gyroSensor;// = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
	//private LazyTalonSRX leftTalon, rightTalon, leftSlaveTalon, leftSlave2Talon, rightSlaveTalon, rightSlave2Talon;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Solenoid shifter;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;

	double prevPositionL = 0;
	double prevPositionR = 0;

	public LazyCANSparkMax leftSpark, rightSpark, leftSparkSlave, rightSparkSlave, leftSparkSlave2, rightSparkSlave2;
  	private CANPIDController leftSparkPID, rightSparkPID;
	  private CANEncoder leftSparkEncoder, rightSparkEncoder;
	  

	private Drive() {

		gyroSensor = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

		leftSpark = new LazyCANSparkMax(Constants.DriveLeftMasterId, MotorType.kBrushless);
		leftSparkSlave = new LazyCANSparkMax(Constants.DriveLeftSlave1Id, MotorType.kBrushless);
		rightSpark = new LazyCANSparkMax(Constants.DriveRightMasterId, MotorType.kBrushless);
		rightSparkSlave = new LazyCANSparkMax(Constants.DriveRightSlave1Id, MotorType.kBrushless);
		//rightSparkSlave2 = new CANSparkMax(0, MotorType.kBrushless);
		//leftSparkSlave2 = new CANSparkMax(0, MotorType.kBrushless);
		leftSpark.setInverted(true);
		rightSpark.setInverted(false);
		leftSparkSlave.setInverted(true);
		rightSparkSlave.setInverted(false);


		

		leftSparkPID = leftSpark.getPIDController();
		rightSparkPID = rightSpark.getPIDController();
		leftSparkEncoder = leftSpark.getEncoder();
		rightSparkEncoder = rightSpark.getEncoder();

	
		//leftSparkPID.
		
		//rightSparkSlave.follow(rightSpark);
		//leftSparkSlave.follow(leftSpark);
		//leftSparkSlave2.follow(leftSpark);
		//rightSparkSlave2.follow(rightSpark);


		shifter = new Solenoid(Constants.DriveShifterSolenoidId);
		//leftTalon = new LazyTalonSRX(Constants.DriveLeftMasterId);
		//rightTalon = new LazyTalonSRX(Constants.DriveRightMasterId);

		//leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		//rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		//leftSlaveTalon = new LazyTalonSRX(Constants.DriveLeftSlave1Id);
		//leftSlave2Talon = new LazyTalonSRX(Constants.DriveLeftSlave2Id);
		//rightSlaveTalon = new LazyTalonSRX(Constants.DriveRightSlave1Id);
		//rightSlave2Talon = new LazyTalonSRX(Constants.DriveRightSlave2Id);
		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(1.0, 0, 1.2, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveHighSpeed, -Constants.DriveHighSpeed);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.DriveTeleopAccLimit);
		turnProfiler = new RateLimiter(100);

		//configHigh();
		configAuto();
	}

	public void debug() {
		System.out.println("L enc: " + getLeftDistance()+ " velo " + getLeftSpeed()); 
		System.out.println("R enc: " + getRightDistance() + " velo " + getRightSpeed()); 
		System.out.println("Gyro: " + getAngle()/*getGyroAngle().getDegrees()*/);
	}

	public void debugSpeed() {
		System.out.println("L speed " +  " actual " + getLeftSpeed());
		System.out.println("R speed " +   " actual " + getRightSpeed());

	}

	public void setRight() {
		setWheelVelocity(new DriveSignal(40, 0));
	}

	private void configAuto() {
		/*
		rightTalon.config_kP(0, Constants.kDriveRightAutoP, 10);
		rightTalon.config_kD(0, Constants.kDriveRightAutoD, 10);
		rightTalon.config_kF(0, Constants.kDriveRightAutoF, 10);
		leftTalon.config_kP(0, Constants.kDriveLeftAutoP, 10);
		leftTalon.config_kD(0, Constants.kDriveRightAutoD, 10);
		leftTalon.config_kF(0, Constants.kDriveLeftAutoF, 10);
		driveMultiplier = Constants.DriveHighSpeed;
		rightTalon.configClosedloopRamp(12d / 200d, 10);
		leftTalon.configClosedloopRamp(12d / 200d, 10);
		*/
		//System.out.println(rightTalon.)
		rightSparkPID.setP(Constants.kDriveRightAutoP, 0);
		rightSparkPID.setD(Constants.kDriveRightAutoD, 0);
		rightSparkPID.setFF(Constants.kDriveRightAutoF,0);
		rightSparkPID.setOutputRange(-1, 1);


		leftSparkPID.setP(Constants.kDriveLeftAutoP, 0);
		leftSparkPID.setD(Constants.kDriveLeftAutoD, 0);
		leftSparkPID.setFF(Constants.kDriveLeftAutoF,0);
		leftSparkPID.setOutputRange(-1, 1);
		

		
	}

	private void configHigh() {
		//rightTalon.config_kP(0, Constants.kDriveRightHighP, 10);
		//rightTalon.config_kD(0, Constants.kDriveRightHighD, 10);
		//rightTalon.config_kF(0, Constants.kDriveRightHighF, 10);
		//rightTalon.configClosedloopRamp(12d / 200d, 10);
		//leftTalon.config_kP(0, Constants.kDriveLeftHighP, 10);
		//leftTalon.config_kD(0, Constants.kDriveRightHighD, 10);
		//leftTalon.config_kF(0, Constants.kDriveLeftHighF, 10);
		//leftTalon.configClosedloopRamp(12d / 200d, 10);

		driveMultiplier = Constants.DriveHighSpeed;
	}

	private void configLow() {
		//rightTalon.config_kP(0, Constants.kDriveRightLowP, 10);
		//rightTalon.config_kF(0, Constants.kDriveRightLowF, 10);
		//leftTalon.config_kP(0, Constants.kDriveLeftLowP, 10);
		//leftTalon.config_kF(0, Constants.kDriveLeftLowF, 10);
		driveMultiplier = Constants.DriveLowSpeed;
	}
	boolean teleopstart =true;

	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}

	public void arcadeDrive(double moveValue, double rotateValue) {
		//String toPrint="";
		//double time = Timer.getFPGATimestamp();
		synchronized(this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue);
		rotateValue = scaleJoystickValues(rotateValue);
		//double t = Timer.getFPGATimestamp() - time;
		//if(teleopstart) toPrint += (t) + " 12\n";

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}
		//if(teleopstart) toPrint += (Timer.getFPGATimestamp() - time) + " 12\n";

		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {
			//System.out.println("left " + getLeftSpeed() + " power: " + leftMotorSpeed);
			//System.out.println("right " + getRightSpeed() + " power: " + rightMotorSpeed);

			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed = moveValue + rotateValue*0.5;
			rightMotorSpeed = moveValue - rotateValue*0.5;
			leftMotorSpeed *= Constants.DriveHighSpeed;
			rightMotorSpeed *= Constants.DriveHighSpeed;

		//	System.out.println("left " + (leftMotorSpeed - getLeftSpeed() ));
			//System.out.println("right " + (rightMotorSpeed - getRightSpeed() ));

			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
		//System.out.println("left motor speed " + leftMotorSpeed + " right motor speed " + rightMotorSpeed);
		//if(teleopstart) toPrint += (Timer.getFPGATimestamp() - time) + " 12\n";
		//System.out.print(toPrint);
		teleopstart = false;
	}

	public void calibrateGyro() {
		gyroSensor.calibrate();
	}

	public void printCurrent() {
		System.out.println(leftSpark);
	}

	public void startHold() {
		prevPositionL = getLeftDistance();
		prevPositionR = getRightDistance();
		driveState = DriveState.HOLD;
		setShiftState(true);
	}

	public void endHold() {
		driveState = DriveState.TELEOP;
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

	public void hold() {
		//leftSparkPID.setReference(leftSparkEncoder.getPosition(), ControlType.kPosition);
		//rightSparkPID.setReference(leftSparkEncoder.getPosition(), ControlType.kPosition);
		//driveState = DriveState.HOLD;
		double errorL = prevPositionL - getLeftDistance();
		double errorR = prevPositionR - getRightDistance();
		setWheelVelocity(new DriveSignal(errorL * Constants.kHoldP , errorR* Constants.kHoldP));


		
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
		leftSparkSlave.follow(leftSpark);
		rightSparkSlave.follow(rightSpark);
		
		leftSpark.setIdleMode(IdleMode.kCoast);
		rightSpark.setIdleMode(IdleMode.kCoast);
		leftSparkSlave.setIdleMode(IdleMode.kCoast);
		rightSparkSlave.setIdleMode(IdleMode.kCoast); 
		/*
		leftSlaveTalon.set(ControlMode.Follower, Constants.DriveLeftMasterId);
		leftSlave2Talon.set(ControlMode.Follower, Constants.DriveLeftMasterId);
		rightSlaveTalon.set(ControlMode.Follower, Constants.DriveRightMasterId);
		rightSlave2Talon.set(ControlMode.Follower, Constants.DriveRightMasterId);
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
		*/
		
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
		/*
		return leftTalon.getSelectedSensorPosition(0) / Constants.EncoderTicksPerRotation * Constants.WheelDiameter
				* Math.PI * 22d / 62d / 3d;
		*/
		return leftSparkEncoder.getPosition() * Constants.WheelDiameter
		* Math.PI * 22d / 62d / 3d;
	}

	public double getRightDistance() {
		//return rightTalon.getSelectedSensorPosition(0) / Constants.EncoderTicksPerRotation * Constants.WheelDiameter
		//		* Math.PI * 22d / 62d / 3d;
		return rightSparkEncoder.getPosition() * Constants.WheelDiameter
		* Math.PI * 22d / 62d / 3d;
	}

	public double getSpeed() {
		/*
		return (-leftSparkEncoder.getVelocity() + rightSparkEncoder.getVelocity())
		 / 10 / 2 * Constants.WheelDiameter * Math.PI;  */
		return (getLeftSpeed()+getRightSpeed())/2;
	}

	public double getLeftSpeed() {
		return leftSparkEncoder.getVelocity()  * 2 * Math.PI/60d * Constants.WheelDiameter/2d
		* 22d / 62d / 3d;
	}

	public double getRightSpeed() {
		return rightSparkEncoder.getVelocity()  * 2 * Math.PI/60d * Constants.WheelDiameter/2d
		* 22d / 62d / 3d;
	}

	public double scaleJoystickValues(double rawValue) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinControllerInput,
				Constants.MaxControllerInput, Constants.MinControllerOutput, Constants.MaxControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		configAuto();
		//System.out.println("even more bad");
		updatePurePursuit();
	}

	public void setBrakeState(NeutralMode mode) {
		//leftTalon.setNeutralMode(mode);
		//rightTalon.setNeutralMode(mode);
		//leftSlaveTalon.setNeutralMode(mode);
		//rightSlaveTalon.setNeutralMode(mode);
		//leftSlave2Talon.setNeutralMode(mode);
		//rightSlave2Talon.setNeutralMode(mode);
	}

	public double getVoltage() {
		return 0;
		//return (leftTalon.getMotorOutputVoltage() + rightTalon.getMotorOutputVoltage()
		//		+ leftSlaveTalon.getMotorOutputVoltage() + rightSlaveTalon.getMotorOutputVoltage()
		//		+ rightSlave2Talon.getMotorOutputVoltage() + leftSlave2Talon.getMotorOutputVoltage()) / 6;
	}

	private void setWheelPower(DriveSignal setVelocity) {
		//leftTalon.set(ControlMode.PercentOutput, setVelocity.rightVelocity);
		//rightTalon.set(ControlMode.PercentOutput, setVelocity.leftVelocity);
		
		leftSpark.set(setVelocity.leftVelocity);
		//leftSparkSlave.set(setVelocity.leftVelocity);
		//rightSparkSlave.set(setVelocity.rightVelocity);
		rightSpark.set(setVelocity.rightVelocity);
	/*	System.out.println(
			"Left Spark: " + leftSpark.getOutputCurrent() + "\n" +
			"Left Slave: " + leftSparkSlave.getOutputCurrent() + "\n" +
			"Right Spark: " + rightSpark.getOutputCurrent() + "\n" +
			"Right Slave: " + rightSparkSlave.getOutputCurrent() + "\n"
		); */
		//System.out.println("velo: " + setVelocity.leftVelocity);

		//leftSparkPID.setReference(setVelocity.leftVelocity, ControlType.kDutyCycle);
		//rightSparkPID.setReference(setVelocity.rightVelocity, ControlType.kDutyCycle);
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		if (Math.abs(setVelocity.rightVelocity) > Constants.DriveHighSpeed
				|| Math.abs(setVelocity.leftVelocity) > Constants.DriveHighSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DriveHighSpeed + " !", false);
			return;
		}
		 //System.out.println("Left: " + setVelocity.leftVelocity);
		 //+ getLeftSpeed());
		// inches per sec to rotations per min
		double leftSetpoint = (setVelocity.leftVelocity)/(2 * Math.PI * Constants.WheelDiameter/2d) * 60d *
				 (62d / 22d) * 3d;
		double rightSetpoint = (setVelocity.rightVelocity)/(2 * Math.PI * Constants.WheelDiameter/2d) * 60d *
		(62d / 22d) * 3d;
		//leftTalon.set(ControlMode.Velocity, leftSetpoint);
		//rightTalon.set(ControlMode.Velocity, rightSetpoint);
		leftSparkPID.setReference(leftSetpoint, ControlType.kVelocity);
		rightSparkPID.setReference(rightSetpoint, ControlType.kVelocity);
		//System.out.println("desired left rpm: " +setVelocity.leftVelocity + "desired right rpm: " + setVelocity.rightVelocity);
		//System.out.println("actual left rpm: " + getLeftSpeed() + " actual right rpm: " + getRightSpeed());
	}

	public synchronized void setSimpleDrive(boolean setting) {
		drivePercentVbus = setting;
	}

	@Override
	public void update() {
	//	System.out.println("L speed " + getLeftSpeed() + " position x " + RobotTracker.getInstance().getOdometry().translationMat.getX());
	//	System.out.println("R speed " + getRightSpeed() + " position y " + RobotTracker.getInstance().getOdometry().translationMat.getY());
	//System.out.println(driveState);	
	DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
				break;
			case PUREPURSUIT:
				//System.out.println("bad!");
				updatePurePursuit();
				break;
			case TURN:
				updateTurn();
				break;
			case HOLD:
				hold();
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
		//System.out.println(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees());
		//System.out.println("error: " + error);
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
				OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.DriveHighSpeed), deltaSpeed);
		if (Math.abs(error) < Constants.maxTurnError && deltaSpeed < Constants.maxPIDStopSpeed) {
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
	//	System.out.println("updating pure presuit");
		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if (signal.isDone) {
			synchronized (this) {
				driveState = DriveState.DONE;
			}
			configHigh();
		}
		//System.out.println("signal l:" + signal.command.leftVelocity + " signal R " + signal.command.rightVelocity);
		setWheelVelocity(signal.command);
	}

	public void resetGyro() {
		gyroSensor.reset();
	}

	public boolean checkSubsystem() {

		// TODO: Get accurate thresholds
		// TODO: Use PDP to get current
		// boolean success =
		//boolean success = leftTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0;
		//success = rightTalon.getSensorCollection().getPulseWidthRiseToRiseUs() == 0 && success;
		//success = OrangeUtility.checkMotors(.25, Constants.DriveExpectedCurrent, Constants.DriveExpectedRPM,
		//		Constants.DriveExpectedPosition, rightTalon, rightTalon, rightSlaveTalon, rightSlave2Talon);
		//success = OrangeUtility.checkMotors(.25, Constants.DriveExpectedCurrent, Constants.DriveExpectedRPM,
		//		Constants.DriveExpectedPosition, leftTalon, leftTalon, leftSlaveTalon, leftSlave2Talon) && success;
		configMotors();
		return true;
		//return success;
	}

	synchronized public void stopMovement() {
		//leftTalon.set(ControlMode.PercentOutput, 0);
		//rightTalon.set(ControlMode.PercentOutput, 0);
		leftSpark.set(0);
		//leftSpark.set(ControlMode.Current, 3);
		rightSpark.set(0);
		leftSparkPID.setReference(0, ControlType.kDutyCycle);
		rightSparkPID.setReference(0, ControlType.kDutyCycle);
		setWheelVelocity(new DriveSignal(0,0));

		driveState = DriveState.TELEOP;
		resetMotionProfile();
	}

	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
	}

	public void clearStickyFaults() {
		//leftTalon.clearStickyFaults(10);
		//leftSlaveTalon.clearStickyFaults(10);
		//leftSlave2Talon.clearStickyFaults(10);
		//rightTalon.clearStickyFaults(10);
		//rightSlaveTalon.clearStickyFaults(10);
		//rightSlave2Talon.clearStickyFaults(10);
	}
}
