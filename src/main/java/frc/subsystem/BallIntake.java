// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallIntake extends Threaded {
	
	public enum IntakeState {
		INTAKE, EJECT, NEUTRAL
	}
	public enum DeployState {
		DEPLOY, DEPLOYING, RETRACTED
	}
	
	private static final BallIntake instance = new BallIntake();
	
	public static final BallIntake getInstance () {
		return instance;
	}
	
	private LazyTalonSRX intakeMotor;
	private IntakeState intakeState;
	private DeployState deployState;
	private Solenoid deploySolenoid;

	
	private BallIntake () {
		intakeMotor = new LazyTalonSRX(Constants.BallIntakeMasterId);
		intakeState = IntakeState.NEUTRAL;
	}

	public DeployState getDeployState() {
		return deployState;
	}
	
	//Set the state of the intake
	public void setIntake (IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}
	}
	
	//Gets the pulled current
	public double getCurrent () {
		return intakeMotor.getOutputCurrent ();
	}
	
	public boolean isFinished () {
		return true;
	}
	
	@Override
	public void update () {
		IntakeState snapIntake;
		DeployState deploy;
		synchronized (this) {
			snapIntake = intakeState;
			deploy = deployState;
		}
		
		switch (snapIntake) {
			case INTAKE:
			intakeMotor.set (ControlMode.PercentOutput, 
			Constants.IntakeMotorPercentOutputIntake);
			break;
			case EJECT:
			intakeMotor.set (ControlMode.PercentOutput, 
			Constants.IntakeMotorPercentOutputEject);
			break;
			case NEUTRAL:
			intakeMotor.set (ControlMode.PercentOutput, 0);
			break;
		}

		switch(deploy) {
			case DEPLOY:
				deploySolenoid.set(true);
				break;
			case RETRACTED:
				deploySolenoid.set(false);
				break;
		}


	}
}