// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;

public class Manipulator extends Threaded {

	public enum ManipulatorState {
		HATCH, BALL
	}

	public enum ManipulatorIntakeState {
		OFF, INTAKE, EJECT, HATCH_HOLD, BALL_HOLD
	}

	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
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

		leftTalon.configPeakCurrentLimit(0);
		leftTalon.configPeakCurrentDuration(0);
		leftTalon.configContinuousCurrentLimit(20);
		leftTalon.enableCurrentLimit(true);

		rightTalon.configPeakCurrentLimit(0);
		rightTalon.configPeakCurrentDuration(0);
		rightTalon.configContinuousCurrentLimit(20);
		rightTalon.enableCurrentLimit(true);
		

		manipulatorSolenoid = new Solenoid(Constants.ManipulatorSolenoidId);

		setPeriod(Duration.ofMillis(50));
	}

	public ManipulatorState getManipulatorState() {
		return state;
	}


	public ManipulatorIntakeState getManipulatorIntakeState() {
		return intakeState;
	}
	
	public double getCurrent() {
		return leftTalon.getOutputCurrent() + rightTalon.getOutputCurrent();
	}

	// Set the deployment state of the intake
	 public void setManipulatorState(ManipulatorState state) {
		synchronized(this) {
			this.state = state;
			changeManipulator();
		}
		//System.out.println("change state to: " + state);
		switch (state) {
			case HATCH:
				//manipulatorSolenoid.set(false);
				//telemetryServer.sendString("sMnM", "hatch");
				break;
			case BALL:
				//manipulatorSolenoid.set(true);
				//telemetryServer.sendString("sMnM", "ball");
				break;
		} 
	
	}

	
	
	// Set the state of the intake
	public void setManipulatorIntakeState(ManipulatorIntakeState intakeState) {
		synchronized (this) {
			this.intakeState = intakeState;
			changeManipulator();

		}
		
		
		switch (intakeState) {
			case INTAKE:
				//telemetryServer.sendString("sMnI", "intake");
				break;
			case EJECT:
				//telemetryServer.sendString("sMnI", "eject");
				break;
			case OFF:
				//telemetryServer.sendString("sMnI", "off");
				break;
			
		}

		
	}

	private void changeManipulator() {
		ManipulatorState manipulator;
		ManipulatorIntakeState intake;
	
		synchronized (this) {
			manipulator = state;
			intake = intakeState;
		}

		switch (manipulator) {
			case HATCH:
				manipulatorSolenoid.set(false);
				//telemetryServer.sendString("sMnM", "hatch");
				break;
			case BALL:
				manipulatorSolenoid.set(true);
				//telemetryServer.sendString("sMnM", "ball");
				break;
		} 


		if (intake == ManipulatorIntakeState.OFF) {
			leftTalon.set(ControlMode.PercentOutput, 0);
			rightTalon.set(ControlMode.PercentOutput, 0);
		} else if(intake == ManipulatorIntakeState.INTAKE || intake == ManipulatorIntakeState.EJECT){
			double basePower = ((manipulator == ManipulatorState.HATCH) ? 1D : -1D)
			                 * ((intake == ManipulatorIntakeState.INTAKE) ? 1D : -1D);
			
			leftTalon.set(ControlMode.PercentOutput,  basePower * Constants.ManipulatorNormalPower);
			rightTalon.set(ControlMode.PercentOutput, -1D *basePower * Constants.ManipulatorNormalPower);
		} else {
			if(intake == ManipulatorIntakeState.HATCH_HOLD) {
				leftTalon.set(ControlMode.PercentOutput, 0.15);
				rightTalon.set(ControlMode.PercentOutput, -0.15);
				//leftTalon.set(ControlMode.Current, 1);
				//leftTalon.set(ControlMode.Current, -1);

			} else {
				leftTalon.set(ControlMode.PercentOutput, -0.15);
				rightTalon.set(ControlMode.PercentOutput, 0.15);
				//leftTalon.set(ControlMode.Current, -1);
				//leftTalon.set(ControlMode.Current, 1);
			}
		}
	}
	
	@Override
	public void update() {
		
		
	}
}
