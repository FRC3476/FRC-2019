// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.subsystem.BallIntake.DeployState;
import frc.utility.control.RateLimiter;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

public class Elevator extends Threaded {

	public enum ElevatorHeight {
		BASE, MIDDLE, TOP
	}

	public enum ElevatorState{
		HOMING, SETPOINT
	}
	
	private static final Elevator instance = new Elevator();
	
	public static Elevator getInstance() {
		return instance;
	}

	BallIntake ballIntake = BallIntake.getInstance();
	Turret turret = Turret.getInstance();
	
	private LazyTalonSRX elevMaster = new LazyTalonSRX(Constants.ElevatorMasterId);
	private LazyTalonSRX elevSlave = new LazyTalonSRX(Constants.ElevatorSlaveId);
	private RateLimiter elevatorLimiter;

	private ElevatorState elevState;
	public double requested;
	private boolean safetyEngage = false;
	private double safeHeight = Constants.ElevatorDeployingSafe;
	private double startTime;
	private boolean isFinished;
	
	private Elevator() {
		elevSlave.follow(elevMaster);
		elevMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
			Constants.ElevatorSensorPidIdx, Constants.TimeoutMs);
		elevMaster.setInverted(false);
		elevSlave.setInverted(false);
		elevMaster.setSensorPhase(false);
		elevMaster.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
		elevMaster.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
		elevMaster.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
		elevMaster.config_IntegralZone(0, Constants.ELevatorIntegralZone, Constants.TimeoutMs);
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);

		elevMaster.configContinuousCurrentLimit(15, 1500);
		elevMaster.configPeakCurrentLimit(20, 50);

		elevatorLimiter = new RateLimiter(Constants.ElevatorVelocityLimit, Constants.ElevatorAccelerationLimit);
	}

	public void resetRateLimits() {
		elevatorLimiter.setAccelLimit(Constants.ElevatorVelocityLimit);
		elevatorLimiter.setJerkLimit(Constants.ElevatorAccelerationLimit);
	}

	public void manualControl(double input) {
		elevMaster.set(ControlMode.PercentOutput, input);
	}
	
	// Gets current height of the elevator
	public double getHeight() {
		return elevMaster.getSelectedSensorPosition()/Constants.ElevatorTicksPerInch;
	}

	public void zero() {
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, Constants.TimeoutMs);
	}
	
	// Sets the height of the elevator
	public void setHeight(double position) {
		if (position < Constants.ElevatorIntakeSafe &&
		 ballIntake.getDeployState() != DeployState.DEPLOY && 
		 Math.abs(turret.getAngle()) < Constants.TurretCollisionRange) {
			requested = position;
			position = safeHeight;
			safetyEngage = true;
			return;
		} else safetyEngage = false;
		//requested = position;
	}

	public void setSafetyHeight(double height) {
		safeHeight = height;
	}

	public boolean isFinished() {
		return isFinished;
	}

	public boolean isSafe() {
		if (Math.abs(safeHeight - getHeight()) < Constants.ElevatorSafetyError) return true;
		else return false;
	}
	
	public void home() {
		startTime = Timer.getFPGATimestamp();
		elevState = ElevatorState.HOMING;
	}
	
	@Override
	public void update() {
		if (Math.abs(requested - getHeight()) < Constants.ElevatorTargetError) isFinished = true;
		switch(elevState){
			// If is in homing mode
			case HOMING:
				if ((Timer.getFPGATimestamp() - startTime) <= 5) {
					System.out.println(elevMaster.getOutputCurrent());
					if (elevMaster.getOutputCurrent() < Constants.ElevatorStallAmps) {
						elevMaster.set(ControlMode.PercentOutput,Constants.ElevatorHomeSpeed);
					} else {
						// Zero
						elevMaster.set(ControlMode.PercentOutput, 0);
						elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
						Constants.TimeoutMs);
						elevState = ElevatorState.SETPOINT;
						System.out.println("Homing succeeded");
					}
				} else{
					// Homing failed
					elevMaster.set(ControlMode.PercentOutput, 0);
					elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);
					elevState = ElevatorState.SETPOINT;
					System.out.println("Homing failed");
				}
			break;
			
			// If is in setpoint mode
			case SETPOINT:
				if (safetyEngage) {
					double setpoint = elevatorLimiter.update(requested);
					elevMaster.set(ControlMode.Position, setpoint * Constants.ElevatorTicksPerInch);
				}
				//if(elevMaster.getOutputCurrent())
				//System.out.println("moving elevator");
				//elevator on triggers
				//if(Robot.j.getRawAxis(2) > 0.1) setHeight(getHeight() - 10*Robot.j.getRawAxis(2));
				//else  setHeight(getHeight() + 10* Robot.j.getRawAxis(3));

				break;
		}
	}
}
