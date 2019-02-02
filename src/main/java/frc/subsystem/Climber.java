// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Climber {

	private static Climber instance = new Climber();

	public static Climber getInstance() {
		return instance;
	}

	private static CANSparkMax climberMaster;
	private static CANSparkMax climberSlave;
	private static CANPIDController climberPID;
	private static CANEncoder climberEncoder;
	
	private Climber() {
		climberMaster = new CANSparkMax(Constants.ClimberMasterId, MotorType.kBrushless);
		climberSlave = new CANSparkMax(Constants.ClimberSlaveId, MotorType.kBrushless);
		climberPID = climberMaster.getPIDController();
		climberEncoder = climberMaster.getEncoder();
	}
	
	public void climb(double value) {
		climberPID.setReference(value, ControlType.kPosition);
	}
	
	public void configMotors() {
		climberSlave.follow(climberMaster, true);
	}
}
