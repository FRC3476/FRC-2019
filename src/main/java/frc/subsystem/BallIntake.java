// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallIntake extends Threaded {

	public enum DeployState {
		DEPLOY, STOW
	}
	
	public enum IntakeState {
		OFF, INTAKE, EJECT
	}
	
	private static final BallIntake instance = new BallIntake();
	
	public static BallIntake getInstance() {
		return instance;
	}
	
	private Solenoid deploySolenoid;
	private LazyTalonSRX intakeMotor;
	private DeployState deployState = DeployState.STOW;
	private IntakeState intakeState = IntakeState.OFF;
	
	private BallIntake() {
		deploySolenoid = new Solenoid(Constants.BallIntakeSolenoidId);
		intakeMotor = new LazyTalonSRX(Constants.BallIntakeMasterId);
	}

	public DeployState getDeployState() {
		return deployState;
	}

	public IntakeState getIntakeState() {
		return intakeState;
	}
	
	// Set the deployment state of the intake
	public void setDeployState(DeployState deployState) {
		synchronized (this) {
			this.deployState = deployState;
		}
	}
	
	// Set the state of the intake
	public void setIntakeState(IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}
	}
	
	// Gets the current draw
	public double getCurrent() {
		return intakeMotor.getOutputCurrent();
	}
	
	public boolean isFinished() {
		return true;
	}
	
	@Override
	public void update() {
		IntakeState intake;
		DeployState deploy;
		synchronized (this) {
			intake = intakeState;
			deploy = deployState;
		}
		
		switch (intake) {
			case INTAKE:
				intakeMotor.set(ControlMode.PercentOutput, Constants.IntakeMotorPowerIntake);
				break;
			case EJECT:
				intakeMotor.set(ControlMode.PercentOutput, -Constants.IntakeMotorPowerEject);
				break;
			case OFF:
				intakeMotor.set(ControlMode.PercentOutput, 0);
				break;
		}

		switch (deploy) {
			case DEPLOY:
				deploySolenoid.set(true);
				break;
			case STOW:
				deploySolenoid.set(false);
				break;
		}
	}
}