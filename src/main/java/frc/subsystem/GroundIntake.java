// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class GroundIntake extends Threaded {

	private static final GroundIntake instance = new GroundIntake ();

	private LazyTalonSRX intakeMotor;
	private IntakeState intakeState;

	private GroundIntake () {
		intakeMotor = new LazyTalonSRX (Constants.Intake1Id);
		intakeState = IntakeState.NEUTRAL;
	}
	
	public static final GroundIntake getInstance () {
		return instance;
	}

	public enum IntakeState {
		INTAKE, OUTTAKE, NEUTRAL
	}

	public void setIntake (IntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}
	}

	public double getCurrent () {
		return intakeMotor.getOutputCurrent ();
	}

	public boolean isFinished () {
		return true;
	}

	@Override
	public void update () {
		IntakeState snapIntake;
		synchronized (this) {
			snapIntake = intakeState;
		}

		switch (snapIntake) {
			case INTAKE:
				intakeMotor.set (ControlMode.PercentOutput, 
					Constants.IntakeMotorPercentOutputIntake);
				break;
			case OUTTAKE:
				intakeMotor.set (ControlMode.PercentOutput, 
					Constants.IntakeMotorPercentOutputOuttake);
				break;
			case NEUTRAL:
				intakeMotor.set (ControlMode.PercentOutput, 0);
				break;
		}
	}
}