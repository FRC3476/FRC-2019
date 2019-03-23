// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.Threaded;
import frc.utility.telemetry.TelemetryServer;

import java.time.Duration;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Solenoid;

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
	private Solenoid deploySolenoid;
	private ClimberState state = ClimberState.OFF;
	private AnalogPotentiometer pot = new AnalogPotentiometer(0, 3600, -1750);
	//private boolean 

	private Climber() {
		climberMaster = new CANSparkMax(Constants.ClimberMasterId, MotorType.kBrushless);
		climberSlave = new CANSparkMax(Constants.ClimberSlaveId, MotorType.kBrushless);
		deploySolenoid = new Solenoid(Constants.ClimberSolenoidID);

		climberPID = climberMaster.getPIDController();
		climberEncoder = climberMaster.getEncoder();
		climberMaster.setInverted(true);
		climberSlave.setInverted(false);

		climberSlave.follow(climberMaster, true);
		setPeriod(Duration.ofMillis(50));
	}
	
	public void beginClimb() {
		state = ClimberState.CLIMB;
		//telemetryServer.sendString("sClm", "climb");
		climberPID.setReference(Constants.ClimberMaxAngle, ControlType.kPosition);
	}

	public void setPower(double p) {
		if(deploySolenoid.get() != true) {
			climberMaster.set(0);
			return; 
		}
		if(p != 0) {
			//System.out.println("Current: " + climberMaster.getOutputCurrent());
			System.out.println("Pot: " + pot.get());

		}

		if(pot.get() >= Constants.ClimberMaxAngle && p > 0) climberMaster.set(0);
		else if(pot.get() <= Constants.ClimberMinAngle && p < 0) climberMaster.set(0);
		else if(pot.get() >= Constants.ClimberMaxAngle-10 && climberMaster.getOutputCurrent()>20) climberMaster.set(0);
		else climberMaster.set(p);
	}

	public void setDeploySolenoid(boolean state) {
		deploySolenoid.set(state);
	}

	@Override
	public void update() {
		//System.out.println("Potentiometer: " + pot.get());

		// TODO: Set state and turn off motor when done
	}
}
