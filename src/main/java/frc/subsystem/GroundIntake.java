// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

public class GroundIntake extends Threaded {

	private static final GroundIntake instance = new GroundIntake();

	private Solenoid intakeSolenoid30Psi;
	private Solenoid intakeSolenoid60Psi;
	private LazyTalonSRX intakeMotor1;
	private LazyTalonSRX intakeMotor2;
	private DigitalInput cubeSensor = new DigitalInput(1);
	private IntakeState intakeState;
	private SolenoidState solenoidState;
	private BiasState biasState;
	private double autoGripTimeout;

	private GroundIntake() {
		intakeMotor1 = new LazyTalonSRX(Constants.Intake1Id);
		intakeMotor2 = new LazyTalonSRX(Constants.Intake2Id);
		intakeSolenoid30Psi = new Solenoid(Constants.IntakeSolenoid30PsiId);
		intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoid60PsiId);
		intakeState = IntakeState.NEUTRAL;
		solenoidState = SolenoidState.CLAMP;
		biasState = BiasState.NORMAL;
	}
	
	private static final GroundIntake getInstance() {
		return instance;
	}

	public enum SolenoidState {
		OPEN, CLAMP, INTAKING, AUTO
	}

	public enum IntakeState {
		INTAKE, OUTTAKE, NEUTRAL
	}

	private enum BiasState {
		REVERSE, NORMAL
	}

	public void setIntake(IntakeState intakeState, SolenoidState solenoidState) {
		synchronized (this) {
			this.intakeState = intakeState;
			this.solenoidState = solenoidState;
		}
	}

	private void setIntakeSolenoid(SolenoidState state) {
		switch (state) {
			case AUTO:
				// Nothing to see here?
			case OPEN:
				intakeSolenoid30Psi.set(true);
				intakeSolenoid60Psi.set(true);
				break;
			case CLAMP:
				intakeSolenoid30Psi.set(false);
				intakeSolenoid60Psi.set(false);
				break;
			case INTAKING:
				intakeSolenoid30Psi.set(true);
				intakeSolenoid60Psi.set(false);
				break;
		}
	}

	public double getCurrent() {
		return (intakeMotor1.getOutputCurrent() + intakeMotor2.getOutputCurrent()) / 2d;
	}

	public boolean isFinished() {
		return true;
	}

	@Override
	public void update() {
		IntakeState snapIntake;
		SolenoidState snapSolenoid;
		synchronized (this) {
			snapIntake = intakeState;
			snapSolenoid = solenoidState;
		}

		switch (snapIntake) {
			case INTAKE:
				if(Constants.OldIntake) {

					double currentRight =  intakeMotor1.getOutputCurrent();
					double currentLeft = intakeMotor2.getOutputCurrent();
					double powerLeft = OrangeUtility.coercedNormalize(currentLeft, 1.5, 20, 0.2, 1);
					double powerRight = OrangeUtility.coercedNormalize(currentRight, 1.5, 20, 0.2, 1);
					double bias = 0;
					if (biasState == BiasState.NORMAL) {
						intakeMotor1.set(ControlMode.PercentOutput, -powerRight);
						intakeMotor2.set(ControlMode.PercentOutput, -powerLeft);
					} else {
						intakeMotor1.set(ControlMode.PercentOutput, -powerRight);
						intakeMotor2.set(ControlMode.PercentOutput, -(-powerLeft));
					}
				} else {
					intakeMotor1.set(ControlMode.PercentOutput, -1);
					intakeMotor2.set(ControlMode.PercentOutput, -1);
				}
				break;
			case OUTTAKE:
				intakeMotor1.set(ControlMode.PercentOutput, .275);
				intakeMotor2.set(ControlMode.PercentOutput, .275);
				break;
			case NEUTRAL:
				intakeMotor1.set(ControlMode.PercentOutput, 0);
				intakeMotor2.set(ControlMode.PercentOutput, 0);
				break;
		}

		if (snapSolenoid == SolenoidState.AUTO) {
			if (!cubeSensor.get()) {
				autoGripTimeout = Timer.getFPGATimestamp();
				setIntakeSolenoid(SolenoidState.INTAKING);
			} else {
				if (Timer.getFPGATimestamp() - autoGripTimeout > 0.5) {
					setIntakeSolenoid(SolenoidState.OPEN);
				}
			}
		} else {
			setIntakeSolenoid(snapSolenoid);
		}
	}
}