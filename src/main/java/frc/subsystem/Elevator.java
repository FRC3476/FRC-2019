// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Elevator extends Threaded {

	//Master Talon
	private LazyTalonSRX elevMaster = new LazyTalonSRX(Constants.ElevatorMotorId);
	private static final Elevator instance = new Elevator();

	//Return an instance of Elevator
	public static Elevator getInstance() {
		return instance;
	}

	//Set PID constants
	private void configMotors(){
		elevMaster.config_kP(0, Constants.kElevatorP, 10);
		elevMaster.config_kI(0, Constants.kElevatorI, 10);
		elevMaster.config_kD(0, Constants.kElevatorD, 10);
    	elevMaster.config_IntegralZone(0, Constants.ELevatorIntegralZone, 10);
	}

	//Elevator constructor to setup the elevator (zero it in the future with current measurement)
	private Elevator() {
		elevMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		elevMaster.setInverted(true);
		elevMaster.setSensorPhase(true);
		elevHome();
		configMotors();
	}
	
	//Gets current position of the elevator
	public double getHeight(){
		return elevMaster.getSelectedSensorPosition(0) * Constants.ElevatorTicksPerInch;
	}

	//Sets the height of the elevator
	public void setHeight(double position){
		elevMaster.set(ControlMode.Position,
		position * Constants.ElevatorTicksPerInch);
	}

	public double getPulledCurrent(){
		return elevMaster.getOutputCurrent();
	}

	public void elevHome(){

		while(getPulledCurrent()<Constants.MaxElevatorAmps){
		elevMaster.set(ControlMode.Velocity, Constants.HighElevatorHomeSpeed*Constants.ElevatorTicksPerInch);
		}

		setHeight(10);
		OrangeUtility.sleep(200);

		/*while(getPulledCurrent()<Constants.MaxElevatorAmps){
			elevMaster.set(ControlMode.Velocity, Constants.MidElevatorHomeSpeed*Constants.ElevatorTicksPerInch);
		}

		setHeight(5);
		OrangeUtility.sleep(200);

		while(getPulledCurrent()<Constants.MaxElevatorAmps){
			elevMaster.set(ControlMode.Velocity, Constants.LowElevatorHomeSpeed*Constants.ElevatorTicksPerInch);
		}*/
		
		elevMaster.setSelectedSensorPosition(0,0,10);//Zero out the encoder
	}

	@Override
	public void update() {
  }
}
