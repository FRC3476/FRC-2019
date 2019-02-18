// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.subsystem.BallIntake.DeployState;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.*;

public class Elevator extends Threaded {

	public enum ElevatorHeight {
		BASE, MIDDLE, TOP
	}
	
	private static final Elevator instance = new Elevator();
	
	public static Elevator getInstance() {
		return instance;
	}
	
	private LazyTalonSRX elevMaster = new LazyTalonSRX(Constants.ElevatorMasterId);
	private LazyTalonSRX elevSlave = new LazyTalonSRX(Constants.ElevatorSlaveId);
	BallIntake ballIntake = BallIntake.getInstance();
	Turret turret = Turret.getInstance();
	public double requested;
	private boolean safetyEngage = false;
	private double safeHeight = Constants.ElevatorDeployingSafe;

	// Elevator constructor to setup the elevator (zero it in the future with current measurement)
	private Elevator() {
		elevSlave.follow(elevMaster);
		elevMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
		Constants.ElevatorSensorPidIdx, Constants.TimeoutMs);
		elevMaster.setInverted(true);
		elevSlave.setInverted(true);
		elevMaster.setSensorPhase(true);
		elevMaster.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
		elevMaster.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
		elevMaster.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
		elevMaster.config_IntegralZone(0, Constants.ELevatorIntegralZone, Constants.TimeoutMs);
		//elevHome();
	}
	
	// Gets current height of the elevator
	public double getHeight() {
		return elevMaster.getSelectedSensorPosition(Constants.ElevatorSensorPidIdx) 
			* Constants.ElevatorTicksPerInch;
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

		requested = position;
		elevMaster.set(ControlMode.Position, position * Constants.ElevatorTicksPerInch);
	}
	
	public double getPulledCurrent() {
		return elevMaster.getOutputCurrent();
	}

	public void setSafetyHeight(double height) {
		safeHeight = height;
	}

	public boolean isSafe() {
		if(Math.abs(safeHeight - getHeight()) < Constants.ElevatorSafetyError) return true;
		else return false;
	}
	
	public void elevHome() {
		while (getPulledCurrent() < Constants.ElevatorStallAmps) {
			elevMaster.set(ControlMode.PercentOutput,Constants.ElevatorHomeSpeed);
		}
		elevMaster.set(ControlMode.PercentOutput, 0);
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
		Constants.TimeoutMs);
	}
	
	public void setHeightState(ElevatorHeight level) {
		switch (level) {
			case BASE:
			elevHome();
			break;
			case MIDDLE:
			setHeight(Constants.ElevatorPositionMiddle);
			break;
			case TOP:
			setHeight(Constants.ElevatorPositionHigh);
			break;
		}
	}
	
	@Override
	public void update() {
		if(safetyEngage) setHeight(requested);

		//elevator on triggers
		if(Robot.j.getRawAxis(2) > 0.1) setHeight(getHeight() - 10*Robot.j.getRawAxis(2));
		else  setHeight(getHeight() + 10* Robot.j.getRawAxis(3));
		
	}
}
