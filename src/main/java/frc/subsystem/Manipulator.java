// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;

public class Manipulator extends Threaded {

	public enum ManipulatorState {
		HATCH, BALL
	}

	public enum ManipulatorIntakeState {
		OFF, INTAKE, EJECT
	}

	private static final Manipulator instance = new Manipulator();
	
	public static Manipulator getInstance() {
		return instance;
	}

	private LazyTalonSRX leftTalon;
	private LazyTalonSRX rightTalon;
	private Solenoid manipulatorSolenoid;
	private ManipulatorState state;
	private ManipulatorIntakeState intakeState;

	private Manipulator() {
		leftTalon = new LazyTalonSRX(Constants.ManipulatorMotor1Id);
		rightTalon = new LazyTalonSRX(Constants.ManipulatorMotor2Id);
		manipulatorSolenoid = new Solenoid(Constants.ManipulatorSolenoidId);
	}

	public ManipulatorState getManipulatorState() {
		return state;
	}

	public ManipulatorIntakeState getManipulatorIntakeState() {
		return intakeState;
	}
	
	// Set the deployment state of the intake
	public void setManipulatorState(ManipulatorState state) {
		synchronized (this) {
			this.state = state;
		}
	}
	
	// Set the state of the intake
	public void setManipulatorIntakeState(ManipulatorIntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
		}
	}
	
	@Override
	public void update() {
		ManipulatorState manipulator;
		ManipulatorIntakeState intake;
		synchronized (this) {
			manipulator = state;
			intake = intakeState;
		}

		if (intake == ManipulatorIntakeState.OFF) {
			leftTalon.set(ControlMode.PercentOutput, 0);
			rightTalon.set(ControlMode.PercentOutput, 0);
		} else {
			double basePower = ((manipulator == ManipulatorState.HATCH) ? 1D : -1D)
			                 * ((intake == ManipulatorIntakeState.INTAKE) ? 1D : -1D);

			leftTalon.set(ControlMode.PercentOutput, -1D * basePower * Constants.ManipulatorNormalPower);
			rightTalon.set(ControlMode.PercentOutput, basePower * Constants.ManipulatorNormalPower);
			
			if (manipulator == ManipulatorState.HATCH) manipulatorSolenoid.set(false);
			else manipulatorSolenoid.set(true);       
		}
	}
}
