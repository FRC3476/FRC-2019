// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.telemetry.TelemetryServer;

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
	
	private TelemetryServer telemetryServer = TelemetryServer.getInstance(); //
	private Solenoid armSolenoid;
	private ArmState lastState = ArmState.RETRACT;
	
	private Arm() {
		armSolenoid = new Solenoid(Constants.ArmSolenoidId);
	}
	
	public void setState(ArmState state) {
		if (state != lastState) {
			switch (state) {
				case RETRACT:
					armSolenoid.set(false);
					telemetryServer.sendString("sArm", "retract");
					break;
				case EXTEND:
					armSolenoid.set(true);
					telemetryServer.sendString("sArm", "extend");
					break;
			}
		}
		lastState = state;
	}
}