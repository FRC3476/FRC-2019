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
	private boolean skipSetAngle = false;
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

	private boolean gotFrame = false;

	JetsonUDP jetsonUDP = JetsonUDP.getInstance();
	Drive drive = Drive.getInstance();

	private RateLimiter limiter;
	private boolean visionLimit = false;

	VisionTarget selected;

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
		setPeriod(Duration.ofMillis(7));
	}

	public static enum TurretState{
		HOMING, SETPOINT, VISION, VISION_LIMITED
	}
	/**
	 * The setAngle method is called by the {@link #update Update} method.
	 * If the state is not at homing, then it calculates the angle, which always calculates to 0. This is a problem because
	 * it always sets the setpoint value at 0 which makes the setpoint always be less than the current angle. This may be the cause 
	 * of the turret always running
	 * past the limit switch, which is why the turret causes internal damage on the robot.
	 * @param angle
	 */
	private void setAngle(double angle) {
		if(turretState == TurretState.HOMING) return;

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
			//case where the turret can go past the limit
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
		
		if(state == TurretState.VISION) {
			JetsonUDP.getInstance().changeExp(false);
			if(prevState != TurretState.VISION) 
			{
				skipSetAngle = false;
				JetsonUDP.getInstance().popTargets();
				gotFrame = false;
			}
		}
		else JetsonUDP.getInstance().changeExp(true);
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
		if(turretState == TurretState.VISION && !gotFrame) return false;
		if(Math.abs(getAngle() - requested) < Constants.TurretTargetError) return true;
		else return false;
	}

	synchronized public boolean isInRange() {
		return targetDistance < Constants.AutoScoreDistance;
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

	private VisionTarget getNearestTarget(VisionTarget[] t) {
		
		int nearIndex = 0;
		double minValue = Double.POSITIVE_INFINITY;
		for(int i = 0; i < t.length; i++) { //106
			double delta_phi = Math.toRadians((2*t[i].x/640.0 - 1) * 92/2) * -1;
			double theta = Math.toRadians(getAngle());
			double phi = delta_phi + theta;
			
			double xOff = Constants.cameraXOffset*Math.cos(theta) - Constants.cameraYOffset*Math.sin(theta);
			double yOff = Constants.cameraXOffset*Math.sin(theta) + Constants.cameraYOffset*Math.cos(theta);

			double y = Math.sin(phi) * t[i].distance + yOff;
			double x = Math.cos(phi) * t[i].distance + xOff;
			//System.out.println("delta: " + delta_phi + " phi: " + phi + " tur delta " + (Math.toDegrees(Math.atan2(y, x))-getAngle()));
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

	public synchronized VisionTarget getSelected() {
		//System.out.println(selected.loc_x+", " + selected.loc_y);
		return selected;
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
			/*VisionTarget[] ftargets = jetsonUDP.getTargets();
				if(ftargets == null || ftargets.length <= 0);
				else {
					synchronized(this) {
					getNearestTarget(ftargets);
					}
				}*/
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
				double delta = getAngle() - requested;
				/**
				 * Whenever called, sets skipSetAngle to false as needed
				 */
				if(Math.abs(delta) <= 10d){
					skipSetAngle = false;
				}
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
					
					synchronized(this) {
						selected = new VisionTarget(getNearestTarget(targets));
						//System.out.println("target " + selected.distance);

					}

					// if(!gotFrame)
					// {
					// 	System.out.println("(" + selected.loc_x + ", " + selected.loc_y + ") vl: " + visionLimit);
					// }
					//System.out.println("target " + selected.loc_x + ", " + selected.loc_y);
					reacquire = false;
					lastDeltaX = lastX - selected.x; 
					double d = selected.distance;//targets[0].distance;

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
			  		double corrected = Math.atan2(selected.loc_y, selected.loc_x);//+ Math.toRadians(getAngle());
					//corrected = 90 - Math.toDegrees(corrected);  
					//double corrected = Math.toDegrees(f); 
					desiredAngle = Math.toDegrees(corrected);
					synchronized(this) {
						targetDistance = selected.getTurretDistance();
					}
					//System.out.println("memez y: " + y + " memez x: " + x);
					//System.out.println("distance " + Math.sqrt(x*x + y*y));
			 		// desiredAngle = turret.getAngle() - f;
					//System.out.println("theta start: " + Math.toDegrees(f) + " d: " + d + " correction: " + corrected);
					//double setpoint = limiter.update(desiredAngle);
					if(visionLimit && desiredAngle - getAngle() >= Constants.MaxVisionScoreAngle) {
						break;
					} else {
						/**
						 * if the skipSetAngle is false, we then actually setAngle,
						 * otherwise we start the longSpin
						 */
						if(!skipSetAngle) {
							setAngle(desiredAngle);
						}
						skipSetAngle = isLongSpin();
					}
					
					synchronized(this) {
						gotFrame = true;

					}
					lastX = selected.x;          
				}
			
			break;
			
		}
		synchronized(this) {
			prevState = turretState;
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

	/**
	* 
	* absolute value of the difference between getAngle and requested, check if the distance is greater than 180 then set the
	* desired angle, otherwise don't set the desired angle.
	*/
	private boolean isLongSpin() {
		return Math.abs(getAngle() - requested) >= 180d;
	}
}
