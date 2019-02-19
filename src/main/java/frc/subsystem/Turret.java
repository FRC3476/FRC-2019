// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.JetsonUDP;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.utility.telemetry.TelemetryServer;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;

public class Turret extends Threaded {
	
	private static final Turret instance = new Turret();

	public static Turret getInstance() {
		return instance;
	}

	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
	private LazyTalonSRX turretMotor;
	private static DigitalInput turretHallEffect;
	private double angle = 0;
	private double prevSign = 1;

	private double dir = 1;
	private boolean isTriggered = false;
	private TurretState turretState;

	JetsonUDP visionData = JetsonUDP.getInstance();

	private Turret() {
		turretMotor = new LazyTalonSRX(Constants.TurretMotorId);
		turretMotor.setSensorPhase(false);
		turretMotor.setInverted(false);
		turretMotor.config_kP(0, Constants.kTurretP, Constants.TimeoutMs);
		turretMotor.config_kI(0, Constants.kTurretI, Constants.TimeoutMs);
		turretMotor.config_kD(0, Constants.kTurretD, Constants.TimeoutMs);
		turretHallEffect = new DigitalInput(Constants.TurretLimitId);
		homeTurret();
	}

	public static enum TurretState{
		HOMING, SETPOINT
	}
	
	public void setAngle(double angle) {
		if(turretState==TurretState.HOMING) return;

		//normalize requested angle on [-180,180]
		angle -= 360.0*Math.round(angle/360.0);

		double setpoint = angle;
		double current = this.angle;

		double dCW;	 //calculate the distance to spin CCW
		double dCCW; //calculate the distance to spin CW

		//pick shortest rotate direction, given that it doesn't twist the cable beyond [-190, 190]
		if (setpoint > current) {	//setpoint is ahead of current
			dCW = Math.abs(setpoint - current);
			dCCW = Math.abs((360 - setpoint) + current);
			if(dCCW < dCW && dCCW <= Constants.maxTurretOverTravel) { //twist further case
				setpoint = setpoint - 360;
			}
		} else {						//setpoint is behind current
			dCCW = Math.abs(setpoint - current);
			dCW = Math.abs((360 + setpoint) - current);
			if(dCW < dCCW && dCW <= Constants.maxTurretOverTravel) {	//twist further case
				setpoint = 360 + setpoint;
			}
		}

		//set talon SRX setpoint between [-180, 180]
		turretMotor.set(ControlMode.Position, setpoint * Constants.EncoderTicksPerDegree);
	}
	
	public void setSpeed(double speed) {
		turretMotor.set(ControlMode.Velocity, speed * Constants.EncoderTicksPerDegree);
	}
	
	public double getSpeed() {
		return turretMotor.getSelectedSensorVelocity() * Constants.DegreesPerEncoderTick;
	}
	
	public double getAngle() {
		return turretMotor.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick;
	}
	
	public double getTargetAngle() {
		return turretMotor.getSetpoint() * Constants.DegreesPerEncoderTick;
	}
	
	public double getOutputCurrent() {
		return turretMotor.getOutputCurrent();
	}

	public void homeTurret() {
		turretState = TurretState.HOMING;
		turretMotor.setSelectedSensorPosition(0, 0, 10); // Zero encoder
		dir = 1; // left
		turretMotor.set(ControlMode.PercentOutput, 0);
	}
	
	@Override
	public void update() {
		VisionTarget[] target = visionData.getTargets();
		//printf(target[0].x);
		//System.out.println(turretMotor.getSelectedSensorPosition());
		switch(turretState){
			//If it is in homing mode
			case HOMING:
				if(!turretHallEffect.get()){
					turretMotor.set(ControlMode.PercentOutput, Constants.TurretHomingPower * dir);
				
					if (getAngle() >= Constants.TurretMaxHomingAngle) dir *= -1; // Switch direction

					//Failed
					if(getAngle() <= -Constants.TurretMaxHomingAngle){
						turretState = TurretState.SETPOINT;
						turretMotor.setSelectedSensorPosition(0,0,10);
						System.out.println("Homing failed");
					}

				} else {
						//Success
						turretMotor.set(ControlMode.PercentOutput, 0);
						turretMotor.setSelectedSensorPosition(0, 0, 10); // Zero encoder
						turretState = TurretState.SETPOINT;
						System.out.println("Homing succeeded");
				}
			break;

			//if it is setpoint mode
			case SETPOINT:
				if(target.length > 0 && target[0] != null) {
					System.out.println("target x " + target[0].x + " angle " + getAngle() + " desired " + angle);
					double error = target[0].x/640.0 - 0.5;
					angle = getAngle() + error * 30.0;	
					prevSign = Math.abs(error)/error;
					setAngle(angle);
				}

			break;
		}

		/*telemetryServer.sendData(
			"trtL", 
			getTargetAngle(), 
			turretMotor.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick
		);*/
		
		//System.out.println(angle);
	//	turretMotor.set(ControlMode.PercentOutput, 0.3);
	}
}
