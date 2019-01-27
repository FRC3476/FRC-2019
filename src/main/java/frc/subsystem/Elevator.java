// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

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
		elevMaster.config_kP(0, Constants.kElevatorP, Constants.TimeoutMs);
		elevMaster.config_kI(0, Constants.kElevatorI, Constants.TimeoutMs);
		elevMaster.config_kD(0, Constants.kElevatorD, Constants.TimeoutMs);
    	elevMaster.config_IntegralZone(0, Constants.ELevatorIntegralZone, Constants.TimeoutMs);
	}

	//Elevator constructor to setup the elevator (zero it in the future with current measurement)
	private Elevator() {
		elevMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
			Constants.ElevatorSensorPidIdx, Constants.TimeoutMs);
		elevMaster.setInverted(true);
		elevMaster.setSensorPhase(true);
		elevHome();
		configMotors();
	}
	
	//Gets current position of the elevator
	public double getHeight(){
		return elevMaster.getSelectedSensorPosition(Constants.ElevatorSensorPidIdx) 
			* Constants.ElevatorTicksPerInch;
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
		elevMaster.set(ControlMode.Velocity, Constants.HighElevatorHomeSpeed 
			* Constants.ElevatorTicksPerInch);
		}
		elevMaster.set(ControlMode.PercentOutput, 0);
		OrangeUtility.sleep(50);
	
		elevMaster.setSelectedSensorPosition(0, Constants.ElevatorSensorPidIdx, 
			Constants.TimeoutMs);//Zero out the encoder
	}

	@Override
	public void update() {
  }
}
