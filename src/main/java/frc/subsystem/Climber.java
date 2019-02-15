// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Climber extends Threaded {

	public enum ClimberState {
		OFF, CLIMB, DONE
	}

	private static Climber instance = new Climber();

	public static Climber getInstance() {
		return instance;
	}

	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
	private CANSparkMax climberMaster;
	private CANSparkMax climberSlave;
	private CANPIDController climberPID;
	private CANEncoder climberEncoder;
	private ClimberState state = ClimberState.OFF;
	
	private Climber() {
		climberMaster = new CANSparkMax(Constants.ClimberMasterId, MotorType.kBrushless);
		climberSlave = new CANSparkMax(Constants.ClimberSlaveId, MotorType.kBrushless);
		climberPID = climberMaster.getPIDController();
		climberEncoder = climberMaster.getEncoder();
		climberSlave.follow(climberMaster, true);
	}
	
	public void beginClimb() {
		state = ClimberState.CLIMB;
		telemetryServer.sendString("sClm", "climb");
		climberPID.setReference(Constants.ClimberMaxAngle, ControlType.kPosition);
	}

	@Override
	public void update() {
		// TODO: Set state and turn off motor when done
	}
}
