// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import edu.wpi.first.wpilibj.Solenoid;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Arm {
	
	public enum ArmState {
		EXTEND, RETRACT
	}
	
	private static Arm instance = new Arm();
	
	public static Arm getInstance() {
		return instance;
	}
	
	private Solenoid armSolenoid;
	
	private Arm() {
		armSolenoid = new Solenoid(Constants.ArmSolenoidId);
	}
	
	public void setState(ArmState state) {
		if (state == ArmState.RETRACT) armSolenoid.set(false);
		else if (state == ArmState.EXTEND) armSolenoid.set(true);
	}
}