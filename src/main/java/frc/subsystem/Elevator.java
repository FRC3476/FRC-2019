// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.subsystem.BallIntake.DeployState;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.RateLimiter;

import java.time.Duration;

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
	
	private LazyTalonSRX elevMaster = new LazyTalonSRX(Constants.ElevatorMasterId);
	private LazyTalonSRX elevSlave = new LazyTalonSRX(Constants.ElevatorSlaveId);
	BallIntake ballIntake = BallIntake.getInstance();
	Turret turret = Turret.getInstance();
	public double requested = Constants.HatchElevLow;
	private boolean safetyEngage = false;
	private double safeHeight = Constants.ElevatorDeployingSafe;
	private double startTime;
	private ElevatorState elevState;
	private boolean isFinished;
	private RateLimiter rateLimiter = new RateLimiter(60, 160);
	private double prevHeight = 0; //temp
	private double prevTime = 0;
	private double maxRate = 0;
	// Elevator constructor to setup the elevator (zero it in the future with current measurement)
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
		//elevHome();
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);

		elevMaster.configContinuousCurrentLimit(15,1500);
		elevMaster.configPeakCurrentLimit(20, 50);
		elevState = ElevatorState.SETPOINT;

		setPeriod(Duration.ofMillis(20));
	}

	public void manualControl(double input) {
		elevMaster.set(ControlMode.PercentOutput, input);
	}
	
	// Gets current height of the elevator
	public double getHeight() {
		return elevMaster.getSelectedSensorPosition()/Constants.ElevatorTicksPerInch;
	}

	public void zero() {
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);
	}
	
	
	// Sets the height of the elevator
	synchronized public void setHeight(double position) {
		/*
		if (position < Constants.ElevatorIntakeSafe &&
		 ballIntake.getDeployState() != DeployState.DEPLOY && 
		 Math.abs(turret.getAngle()) < Constants.TurretCollisionRange) {
			requested = position;
			position = safeHeight;
			safetyEngage = true;
			return;
		} else safetyEngage = false;
		*/
		requested = position;
	}
	
	synchronized public double getPulledCurrent() {
		return elevMaster.getOutputCurrent();
	}

	synchronized public void setSafetyHeight(double height) {
		safeHeight = height;
	}

	synchronized public boolean isFinished() {
		if(Math.abs(getHeight() - requested) < Constants.ElevatorTargetError) return true;
		else return false;
	}

	synchronized public boolean isFinishedOrHigher() {
		return (getHeight() > requested) || isFinished();
	}

	synchronized public void resetDT() {
		rateLimiter.resetTime();
	}

	synchronized public boolean isSafe() {
		if(Math.abs(safeHeight - getHeight()) < Constants.ElevatorSafetyError) return true;
		else return false;
	}

	synchronized public double getRequested() {
		return requested;
	}
	
	synchronized public void elevHome() {
		startTime = Timer.getFPGATimestamp();
		elevState = ElevatorState.HOMING;
	}
	
	public void setHeightState(ElevatorHeight level) {
		switch (level) {
			case BASE:
			setHeight(Constants.ElevatorPositionLow);
			break;
			case MIDDLE:
			setHeight(Constants.ElevatorPositionMiddle);
			break;
			case TOP:
			setHeight(Constants.ElevatorPositionHigh);
			break;
		}
	}
	boolean started = false;
	@Override
	synchronized public void update() {
		if(started != true) {
			startTime = Timer.getFPGATimestamp();
			started = true;
		}

		//if(Math.abs(requested - getHeight()) < Constants.ElevatorTargetError) isFinished = true;
		switch(elevState){
			//If is in homing mode
			case HOMING:
				if((Timer.getFPGATimestamp()-startTime)<=3) {
					//System.out.println(getPulledCurrent());
					if(getPulledCurrent() < Constants.ElevatorStallAmps) {
						elevMaster.set(ControlMode.PercentOutput,Constants.ElevatorHomeSpeed);
					} else {
					//Zero
					elevMaster.set(ControlMode.PercentOutput, 0);
					elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);
					elevState = ElevatorState.SETPOINT;
					setHeight(Constants.HatchElevLow);
					//System.out.println("Elevator homing succeeded");
					}
				} else{
					//Homing failed
					elevMaster.set(ControlMode.PercentOutput, 0);
					elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
					Constants.TimeoutMs);
					elevState = ElevatorState.SETPOINT;
					//System.out.println("Elevator homing failed");
				}
			break;
			
			//If is in setpoint mode
			case SETPOINT:
				double setpoint = rateLimiter.update(requested);
				//System.out.println("elev " + setpoint);
				elevMaster.set(ControlMode.Position, setpoint * Constants.ElevatorTicksPerInch);
				double rate = (getHeight() - prevHeight)/(Timer.getFPGATimestamp() - prevTime);
				prevTime = Timer.getFPGATimestamp();
				prevHeight = getHeight();
				if(rate >= maxRate) maxRate = rate;
				//System.out.println("elev max rate: " + maxRate); 
				//System.out.println(elevMaster.getSelectedSensorPosition());
				//if(safetyEngage) setHeight(requested);
				//System.out.println("Elevator current: " + getPulledCurrent());
				//if(elevMaster.getOutputCurrent())
				//System.out.println("moving elevator");
				//elevator on triggers
				//if(Robot.j.getRawAxis(2) > 0.1) setHeight(getHeight() - 10*Robot.j.getRawAxis(2));
				//else  setHeight(getHeight() + 10* Robot.j.getRawAxis(3));
				
				break;
		}
	}
}
