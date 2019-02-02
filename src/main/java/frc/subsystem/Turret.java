// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Turret extends Threaded {
	
	private static Turret instance = new Turret();
	private LazyTalonSRX turretTalon;
	private static DigitalInput turretHalleffect;


	public static Turret getInstance() {
		return instance;
	}
		
	private Turret() {
		turretTalon = new LazyTalonSRX(Constants.TurretMotorId);
		turretTalon.setSensorPhase(false);
		turretTalon.setInverted(false);
		//turretTalon.setSelectedSensorPosition(0,0,10);
		
		turretHalleffect = new DigitalInput(Constants.turretLimitId);
	}
	
	public void stop(){
		setPercentOutput(0);
	}

	public void setPercentOutput(double output) {
		turretTalon.set(ControlMode.PercentOutput, output);
	}
	
	public void setAngle(double angle) {
		turretTalon.set(ControlMode.Position, angle * Constants.EncoderTicksPerDegree);
	}
	
	public void setSpeed(double speed) {
		turretTalon.set(ControlMode.Velocity, speed * Constants.EncoderTicksPerDegree);
	}
	
	public double getSpeed() {
		return turretTalon.getSelectedSensorVelocity(Constants.TurretMotorId) * Constants.DegreesPerEncoderTick;
	}
	
	public double getAngle () {
		return turretTalon.getSelectedSensorPosition(Constants.TurretMotorId) * Constants.DegreesPerEncoderTick;
	}
	
	public double getTargetAngle() {
		return turretTalon.getSetpoint() * Constants.DegreesPerEncoderTick;
	}
	
	//Return current
	public double getOutputCurrent() {
		return turretTalon.getOutputCurrent();
	}

	public void homeTurret(){
		turretTalon.setSelectedSensorPosition(0,0,10);//Zero encoder
		double dir = 1;//left

		boolean isTriggered = false;
		while(!isTriggered){
			setPercentOutput(Constants.turretHomingSpeed*dir);
			
			if(getAngle()>=Constants.maxTurretHomingAngle){
				dir*=-1;//Switch direction
			}

			if(turretHalleffect.isAnalogTrigger()){
				stop();
				turretTalon.setSelectedSensorPosition(0,0,10);//Zero the encoder
			}
		}
	}
	
	@Override
	public void update() {}
}
