// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.utility.OrangeUtility;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class Arm extends Threaded {

	private LazyTalonSRX armTalon;

	private static final Arm instance = new Arm ();

	public static Arm getInstance () {
		return instance;
	}

	public void setPercentOutput (double output) {
		armTalon.set (ControlMode.PercentOutput, output);
	}

	protected void setAngle (double angle) {
		if(angle>Constants.ArmUpperDegreeLimit!=true || angle<Constants.ArmDownDegreeLimit!=true){

			armTalon.set (ControlMode.Position, angle * Constants.AngleConversionRate2);
		}
	}

	public void setSpeed (double speed) {
		armTalon.set (ControlMode.Velocity, speed * Constants.AngleConversionRate2);
	}

	public double getSpeed () {
		return armTalon.getSelectedSensorVelocity (0) * Constants.AngleConversionRate;
	}

	public double getAngle () {
		return armTalon.getSelectedSensorPosition (0) * Constants.AngleConversionRate;
	}

	public double getTargetAngle () {
		return armTalon.getSetpoint () * Constants.AngleConversionRate;
	}

	public double getOutputCurrent () {
		return armTalon.getOutputCurrent ();
	}

	//Sets the angle which makes it go at a certain height
	public void setHeight(double height){
		double angle = Math.toDegrees(Math.acos(height/Constants.ArmLength));
		setAngle(angle);
	}

	//Gets height according to the angle and the length of the arm
	public double getHeight(){
		double height = Math.cos(Math.toRadians(getAngle()))*Constants.ArmLength;
		return height;
	}

	public void armHome () {
		while (getOutputCurrent () < Constants.HighArmAmps) {
			armTalon.set(ControlMode.PercentOutput, Constants.ArmHomingSpeed);
		}

		armTalon.set(ControlMode.PercentOutput, 0);//Stop arm
		OrangeUtility.sleep(50);//Sleep for 50 ms
		armTalon.setSelectedSensorPosition(0,0,10);//Zero encoder
	}

	private Arm() {
		armTalon = new LazyTalonSRX(Constants.ArmId);
		armTalon.setSensorPhase(false);
		armTalon.setInverted(false);
  	}

	@Override 
	public void update () {}
}