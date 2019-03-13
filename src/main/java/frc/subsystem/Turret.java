// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.JetsonUDP;
import frc.utility.LazyTalonSRX;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.utility.telemetry.TelemetryServer;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Robot;

public class Turret extends Threaded {
	
	private static final Turret instance = new Turret();

	public static Turret getInstance() {
		return instance;
	}

	private TelemetryServer telemetryServer = TelemetryServer.getInstance();
	private LazyTalonSRX turretMotor;
	public static DigitalInput turretHallEffect;
	private double angle = 0;
	private double prevSign = 1;

	private double dir = 1;
	private boolean isTriggered = false;
	private TurretState turretState;
	private boolean switchFlag = false;
	private double lastTargetGyro = 0;
	private double lastDeltaX = 0;
	private double lastX = 0 ;
	private boolean reacquire = false;

	private double desired;
	private boolean fieldRelative;

	private double requested;

	JetsonUDP jetsonUDP = JetsonUDP.getInstance();
	Drive drive = Drive.getInstance();

	private Turret() {
		turretMotor = new LazyTalonSRX(Constants.TurretMotorId);
		turretMotor.setSensorPhase(false);
		turretMotor.setInverted(false);
		turretMotor.config_kP(0, Constants.kTurretP, Constants.TimeoutMs);
		turretMotor.config_kI(0, Constants.kTurretI, Constants.TimeoutMs);
		turretMotor.config_kD(0, Constants.kTurretD, Constants.TimeoutMs);
		turretHallEffect = new DigitalInput(Constants.TurretLimitId);
		turretMotor.setSelectedSensorPosition(0, 0, 10);
		//homeTurret();
		turretState = TurretState.SETPOINT;
		this.fieldRelative = true;

		setPeriod(Duration.ofMillis(20));
	}

	public static enum TurretState{
		HOMING, SETPOINT, VISION
	}
	
	public void setAngle(double angle) {
		if(turretState==TurretState.HOMING) return;

		//normalize requested angle on [-180,180]
		angle -= 360.0*Math.round(angle/360.0);

		double setpoint = angle;
		double current = getAngle();

		double dCW;	 //calculate the distance to spin CCW
		double dCCW; //calculate the distance to spin CW
		
		//pick shortest rotate direction, given that it doesn't twist the cable beyond [-190, 190]
		if (setpoint > current) {	//setpoint is ahead of current
			dCCW = Math.abs(setpoint - current);
			dCW = Math.abs((360 - setpoint) + current);
			if(dCW < dCCW && Math.abs(Math.abs(setpoint)-180) <= Constants.maxTurretOverTravel) { //twist further case
				setpoint = setpoint - 360;
			}
		} else {						//setpoint is behind current
			dCW = Math.abs(setpoint - current);
			dCCW = Math.abs((360 + setpoint) - current);
			if(dCCW < dCW && Math.abs(Math.abs(setpoint)-180) <= Constants.maxTurretOverTravel) {	//twist further case
				setpoint = 360 + setpoint;
			}
		}
		//System.out.println("setpoint translated: " +setpoint + " speed " + turretMotor.getSelectedSensorVelocity());
		//set talon SRX setpoint between [-180, 180]
		if(setpoint > 180 + Constants.maxTurretOverTravel || setpoint < -180-Constants.maxTurretOverTravel) {
			//System.out.println("setpoint error");
			setpoint = 0;
		}
		synchronized(this) {
			requested = setpoint;
		}
		turretMotor.set(ControlMode.Position, -setpoint * Constants.EncoderTicksPerDegree*10.6);
	}
	
	public void setSpeed(double speed) {
		turretMotor.set(ControlMode.Velocity, speed * Constants.EncoderTicksPerDegree);
	}
	
	public double getSpeed() {
		return turretMotor.getSelectedSensorVelocity() * Constants.DegreesPerEncoderTick;
	}
	
	public double getAngle() {
		return -turretMotor.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick/10.6;
	}
	
	public double getTargetAngle() {
		return turretMotor.getSetpoint() * Constants.DegreesPerEncoderTick;
	}
	
	public double getOutputCurrent() {
		return turretMotor.getOutputCurrent();
	}

	synchronized public void homeTurret() {
		turretState = TurretState.HOMING;
		turretMotor.setSelectedSensorPosition(0, 0, 10); // Zero encoder
		dir = 1; // left
		switchFlag = false;
		turretMotor.set(ControlMode.PercentOutput, 0);
		//System.out.println("starting turret homing");
	}

	synchronized public void setDesired(double desired, boolean fieldRelative) {
		this.desired = desired;
		this.fieldRelative = fieldRelative;

	}

	synchronized public void addDesired(double delta) {
		this.desired += delta;
	}

	synchronized public void setState(TurretState state) {
		if(this.turretState != TurretState.HOMING)
		{
			this.turretState = state;
		}
	}

	synchronized public void restoreSetpoint() {
		if(fieldRelative) this.desired = getAngle() - drive.getAngle();
		else this.desired = getAngle();
	}
	
	synchronized public boolean isFinished() {//NOT YET IMPLEMENTED
		if(Math.abs(Math.abs(getAngle()) - Math.abs(requested)) < Constants.ElevatorTargetError) return true;
		else return false;
	}


	@Override
	synchronized public void update() {
		//System.out.println("turret hall effect: ");
		
		//System.out.println("turret hall effect: " + turretHallEffect.get());
		//VisionTarget[] target = visionData.getTargets();
		//System.out.println(target[0].x);
		//System.out.println(turretMotor.getSelectedSensorPosition());
		// synchronized(this) {
		// 	snapDesired 
		// }

		switch(turretState){
	
			//If it is in homing mode
			case HOMING:
				//System.out.println("homing now");
				//System.out.println(switchFlag + " " + dir);
				//System.out.println(Math.abs(getAngle()) >= Constants.TurretMaxHomingAngle);
				//System.out.println("he: " + turretHallEffect.get());
				if(turretHallEffect.get()){
					turretMotor.set(ControlMode.PercentOutput, Constants.TurretHomingPower * dir);
					
					if (Math.abs(getAngle()) >= Constants.TurretMaxHomingAngle && switchFlag == false) {
						dir *= -1; // Switch direction
						switchFlag = true;
					}
					//Failed
					/*
					if(Math.abs(getAngle()) <= Constants.TurretMaxHomingAngle){
						turretState = TurretState.SETPOINT;
						turretMotor.setSelectedSensorPosition(0,0,10);
						System.out.println("Homing failed");
					}*/

				} else {
						//Success
						turretMotor.set(ControlMode.PercentOutput, 0);
						turretMotor.setSelectedSensorPosition(0, 0, 10); // Zero encoder
						turretState = TurretState.SETPOINT;
						//System.out.println("Turret homing succeeded");
				}
			break;

			//if it is setpoint mode
			case SETPOINT:
				if(this.fieldRelative) setAngle(desired + drive.getAngle());
				else setAngle(desired);
			break;

			case VISION:
				double desiredAngle = 0;
				restoreSetpoint();
				//System.out.println("in vision mode ");
				VisionTarget[] targets = jetsonUDP.getTargets();
			
				if(targets.length == 0 || targets == null) {
					if(reacquire) {
						//turretState = turretState.SETPOINT;
						//restoreSetpoint();
					} else {
						//reacquire = true;
						//if(lastDeltaX < 0) setAngle(getAngle() -20);
						//else setAngle(getAngle()+20);
					}
				}
				else {
					//lastTargetGyro = drive.getAngle();
					reacquire = false;
					lastDeltaX = lastX - targets[0].x; 
			  		double d = targets[0].distance;
			  		double f = Math.toRadians((targets[0].x/640.0 - 0.5) * (59.7/2));
			  		double corrected = Math.atan2(Math.cos(f) * d + Constants.cameraYOffset, Math.sin(f) * d +  Constants.cameraXOffset);
					corrected = 90 - Math.toDegrees(corrected);  
					//double corrected = Math.toDegrees(f); 
					desiredAngle = getAngle() - corrected;
			 		// desiredAngle = turret.getAngle() - f;
					//System.out.println("theta start: " + Math.toDegrees(f) + " d: " + d + " correction: " + corrected);
					setAngle(desiredAngle);          
					lastX = targets[0].x;           
				}
			
			break;
			
		}

		/*telemetryServer.sendData(
			"trtL", 
			getTargetAngle(), 
			turretMotor.getSelectedSensorPosition() * Constants.DegreesPerEncoderTick
		);
		
		//System.out.println(angle);
	//	turretMotor.set(ControlMode.PercentOutput, 0.3);
		*/
	}
}
