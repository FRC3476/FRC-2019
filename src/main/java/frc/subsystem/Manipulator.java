// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

public class Manipulator {

	public enum ManipulatorState {
		HATCH_INTAKE, BALL_INTAKE, HATCH_EJECT, BALL_EJECT
	}
	
	public enum EjectDirection {
		LEFT, RIGHT
	}

	private static final Manipulator instance = new Manipulator();
	
	public static Manipulator getInstance() {
		return instance;
	}

	private static LazyTalonSRX leftTalon;
	private static LazyTalonSRX rightTalon;
	private static Solenoid manipulatorSolenoid;

	private Manipulator() {
		leftTalon = new LazyTalonSRX(Constants.ManipulatorMotor1Id);
		rightTalon = new LazyTalonSRX(Constants.ManipulatorMotor2Id);
		manipulatorSolenoid = new Solenoid(Constants.ManipulatorSolenoidId);
	}
	
	// Stop wheels from spinning
	public static void stop() {
		leftTalon.set(ControlMode.PercentOutput, 0);
		rightTalon.set(ControlMode.PercentOutput, 0);
	}
	
	// Eject to the left or to the right
	public static void setSideEject(EjectDirection dir) {
		if(dir == EjectDirection.LEFT){
			rightTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed);
			leftTalon.set(ControlMode.PercentOutput, Constants.LowManipulatorSpeed);
		}
		else if(dir == EjectDirection.RIGHT){
			rightTalon.set(ControlMode.PercentOutput, Constants.LowManipulatorSpeed);
			leftTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed);
		}
	}
	
	// Eject the ball or hatch straight
	public static void setStraightEject(ManipulatorState hatchBall) {
		if (hatchBall == ManipulatorState.HATCH_EJECT) {
			leftTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed);
			rightTalon.set(ControlMode.PercentOutput, -1 * Constants.NormalManipulatorSpeed);
		} else if (hatchBall == ManipulatorState.BALL_EJECT) {
			leftTalon.set(ControlMode.PercentOutput, -1 * Constants.NormalManipulatorSpeed);
			rightTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed);          
		}
	}
	
	// Intakes Hatch or Ball depending on the inputed enum
	public static void setIntake(ManipulatorState hatchBall) {
		if (hatchBall == ManipulatorState.HATCH_INTAKE) {
			// Close manipulator and spin wheels inward 
			manipulatorSolenoid.set(false);
			leftTalon.set(ControlMode.PercentOutput, -1 * Constants.NormalManipulatorSpeed);
			rightTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed); 
		} else if (hatchBall == ManipulatorState.BALL_INTAKE) { 
			// Open manipulator and spin wheels outward
			manipulatorSolenoid.set(true);
			leftTalon.set(ControlMode.PercentOutput, Constants.NormalManipulatorSpeed);
			rightTalon.set(ControlMode.PercentOutput, -1 * Constants.NormalManipulatorSpeed);
		}
	}
}
