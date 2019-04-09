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
import frc.utility.control.RateLimiter;

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
	private double lastDistance = Constants.AutoScoreDistance+10;

	private double desired;
	private boolean fieldRelative;
	private double targetDistance;

	private double requested = 0;
	public int twistDir = 1;

	private double prevAngle = 0;
	private double prevTime = 0;
	private double velocity = 0;

	private TurretState prevState = TurretState.SETPOINT;

	JetsonUDP jetsonUDP = JetsonUDP.getInstance();
	Drive drive = Drive.getInstance();

	private RateLimiter limiter;
	private boolean visionLimit = false;

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
		limiter = new RateLimiter(5000, 800);
		resetDistance();
		setPeriod(Duration.ofMillis(20));
	}

	public static enum TurretState{
		HOMING, SETPOINT, VISION, VISION_LIMITED
	}
	
	private void setAngle(double angle) {
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
		
		synchronized(this) {
			requested = setpoint;
		}
		
		//double before_limiter = setpoint;
		if(setpoint > 180 + Constants.maxTurretOverTravel || setpoint < -180-Constants.maxTurretOverTravel) {
			System.out.println("before limiter setpoint error, setpoint = " + setpoint);
		}
		setpoint = limiter.update(setpoint);
		
		if(setpoint > 180 + Constants.maxTurretOverTravel || setpoint < -180-Constants.maxTurretOverTravel) {
			System.out.println("setpoint error, setpoint = " + setpoint);
			if(setpoint > 180 + Constants.maxTurretOverTravel) setpoint = 180;// + Constants.maxTurretOverTravel;
			else setpoint = -180;// -Constants.maxTurretOverTravel;
			limiter.reset();//out of range, panic
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

	public void setVisionLimited(boolean on) {
		visionLimit = on;
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
	
	synchronized public boolean isFinished() {
		if(turretState != prevState) return false;
		if(turretState == TurretState.HOMING) return false;
		if(Math.abs(getAngle() - requested) < Constants.TurretTargetError) return true;
		else return false;
	}

	synchronized public boolean isInRange() {
		return lastDistance < Constants.AutoScoreDistance;
	}

	synchronized public int isInBallRange() {
		if(targetDistance < Constants.AutoScoreDistanceBallClose) return 0;
		else if(targetDistance < Constants.AutoScoreDistanceBallFar) return 1;
		else return 2;
	}

	synchronized public void resetDT() {
		limiter.resetTime();
	}


	synchronized public void resetDistance() {
		lastDistance = Constants.AutoScoreDistanceBallFar + 10;
		targetDistance = Constants.AutoScoreDistanceBallFar + 10;
	}

	private static VisionTarget getNearestTarget(VisionTarget[] t) {
		
		int nearIndex = 0;
		double minValue = Double.POSITIVE_INFINITY;
		for(int i = 0; i < t.length; i++) {
			double f = Math.toRadians((t[i].x/640.0 - 0.5) * 136/2);  
			double y = Math.cos(f) * t[i].distance + Constants.cameraYOffset;
			double x = Math.sin(f) * t[i].distance + Constants.cameraXOffset;
			t[i].setLoc(x, y);
			t[i].setTurretRelativeDistance( Math.sqrt(x * x + y * y) );

			if(t[i].getTurretDistance() < minValue) {
				minValue = t[i].getTurretDistance();
				nearIndex = i;
			}

		}
		return t[nearIndex];
	}

	public double getVelocity() {
		return this.velocity;
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
		velocity = (getAngle() - prevAngle)/(Timer.getFPGATimestamp()- prevTime);
		prevAngle = getAngle();
		prevTime = Timer.getFPGATimestamp();

		if(getAngle() < 0) twistDir = -1;
		else twistDir = 1;

		//System.out.println(turretState);

		switch(turretState){
			//If it is in homing mode
			case HOMING:
				//System.out.println("homing now");
				//System.out.println(switchFlag + " " + dir);
				//System.out.println(Math.abs(getAngle()) >= Constants.TurretMaxHomingAngle);
				//System.out.println("he: " + turretHallEffect.get());
				if(turretHallEffect.get()){
					turretMotor.set(ControlMode.PercentOutput, Constants.TurretHomingPower * dir);
					
					if (getAngle() <= -Constants.TurretMaxHomingAngle && switchFlag == false) {
						dir *= -1; // Switch direction
						switchFlag = true;
					}
					//Failed
					
					else if(getAngle() >= Constants.TurretMaxHomingAngle){
						turretState = TurretState.SETPOINT;
						//turretMotor.setSelectedSensorPosition(0,0,10);
						System.out.println("Homing failed");
					}

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

				if(this.fieldRelative) {
					//double setpoint = limiter.update(desired + drive.getAngle());    
					setAngle(desired + drive.getAngle());
				}
				else {
					//double setpoint = limiter.update(desired);
					setAngle(desired);      
				}
			break;

			case VISION:
				double desiredAngle = 0;
				restoreSetpoint();
				//System.out.println("in vision mode ");
				VisionTarget[] targets = jetsonUDP.getTargets();

				//System.out.println("amunt of targets" + targets.length);
				if(targets == null || targets.length <= 0) {
					//System.out.println("null");
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
					//System.out.println("turret");
					VisionTarget selected = getNearestTarget(targets);
					reacquire = false;
					lastDeltaX = lastX - selected.x; 
					double d = targets[0].distance;

					synchronized(this) {
						lastDistance = d;
					}
					//double beta = Math.toDegrees(Math.atan(Math.cos(Math.atan(3/4))*Math.tan(170/2)));
					//double focallength = 640 / (2*Math.tan(170/2));
					//double angtotarget = Math.atan2((selected.x - 640/2), focallength);
					//136
					//double f = Math.toRadians((selected.x/640.0 - 0.5) * 136/2);  //(148.16/2));
					//System.out.println("angtotarget: " + angtotarget + "f: " + f);
					//double y = Math.cos(f) * d + Constants.cameraYOffset;
					//double x = Math.sin(f) * d + Constants.cameraXOffset;
			  		double corrected = Math.atan2(selected.loc_y, selected.loc_x);
					corrected = 90 - Math.toDegrees(corrected);  
					//double corrected = Math.toDegrees(f); 
					desiredAngle = getAngle() - corrected;
					synchronized(this) {
						targetDistance = selected.getTurretDistance();
					}
					//System.out.println("memez y: " + y + " memez x: " + x);
					//System.out.println("distance " + Math.sqrt(x*x + y*y));
			 		// desiredAngle = turret.getAngle() - f;
					//System.out.println("theta start: " + Math.toDegrees(f) + " d: " + d + " correction: " + corrected);
					//double setpoint = limiter.update(desiredAngle);
					if(visionLimit && desiredAngle - getAngle() >= Constants.MaxVisionScoreAngle) break;
					else setAngle(desiredAngle);          
					lastX = selected.x;           
				}
			
			break;
			
		}
		
		prevState = turretState;
		
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
