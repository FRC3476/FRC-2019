// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.subsystem.BallIntake.DeployState;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;

public class Elevator extends Threaded {

	public enum ElevatorHeight {
		LOWER, MIDDLE, TOP
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
	
	
	// Sets the height of the elevator in inches
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
		elevMaster.set(ControlMode.Position, position * Constants.ElevatorTicksPerInch);
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
			case LOWER:
			setHeight(Constants.HatchElevLow);
			break;
			case MIDDLE:
			setHeight(Constants.HatchElevMid);
			break;
			case TOP:
			setHeight(Constants.HatchElevHigh);
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

		if(Math.abs(requested - getHeight()) < Constants.ElevatorTargetError) isFinished = true;
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
				if(safetyEngage) setHeight(requested);
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
