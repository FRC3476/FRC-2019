// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Arm extends Threaded {

	private LazyTalonSRX armTalon;
	protected long homeStartTime;

	private static final Arm instance = new Arm();

	public static Arm getInstance() {
		return instance;
	}

	private Arm() {
		armTalon = new LazyTalonSRX(Constants.ArmId);
		armTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 
			Constants.ArmFeedbackSensorPidIdx, Constants.ArmTimeoutMs);
		setEncoderFromPWM(); 
		armTalon.config_kP(Constants.ArmFeedbackSensorPidIdx, Constants.ArmConfigKP,
			Constants.ArmTimeoutMs);
		armTalon.config_kI(Constants.ArmFeedbackSensorPidIdx, Constants.ArmConfigKI, 
			Constants.ArmTimeoutMs);
		armTalon.config_kD(Constants.ArmFeedbackSensorPidIdx, Constants.ArmConfigKD, 
			Constants.ArmTimeoutMs);
		armTalon.setSensorPhase(false);
		armTalon.setInverted(false);
  }

	public void setPercentOutput(double output) {
		armTalon.set(ControlMode.PercentOutput, output);
	}

	protected void setEncoderPosition(int position) {
		armTalon.setSelectedSensorPosition(position, Constants.ArmFeedbackSensorPidIdx, 
			Constants.ArmTimeoutMs);
	}

	public int getEncoderPosition() {
		return armTalon.getSelectedSensorPosition(Constants.ArmFeedbackSensorPidIdx);
	}

	protected void setAngle(double angle) {
		armTalon.set(ControlMode.Position, angle * (1d / 360) *
			Constants.SensorTicksPerMotorRotation);
	}

	public void setSpeed(double speed) {
		armTalon.set(ControlMode.Velocity, speed * (1d / 360) *
			Constants.SensorTicksPerMotorRotation);
	}

	public double getSpeed() {
		return armTalon.getSelectedSensorVelocity(0) * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getAngle() {
		return armTalon.getSelectedSensorPosition(0) * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getTargetAngle() {
		return armTalon.getSetpoint() * 360 * 
			(1d / Constants.SensorTicksPerMotorRotation);
	}

	public double getOutputCurrent() {
		return armTalon.getOutputCurrent();
	}

	public boolean checkSubsytem() {
		return OrangeUtility.checkMotors(0.05, Constants.ExpectedArmCurrent, Constants.ExpectedArmRPM,
			Constants.ExpectedArmPosition, armTalon, armTalon);
	}

	public void setEncoderFromPWM() {
		// Value becomes negative when we set it for some reason
		armTalon.getSensorCollection().setQuadraturePosition(
			(getPWMPosition() + (int) (Constants.ArmDownDegrees * (1d / 360) * 
			Constants.SensorTicksPerMotorRotation)), Constants.ArmTimeoutMs);
	}

	public int getPWMPosition() {
		int pwmValue = armTalon.getSensorCollection().getPulseWidthPosition();
		pwmValue -= Constants.PracticeBotArmTicksOffset;
		pwmValue %= Constants.SensorTicksPerMotorRotation;
		pwmValue += (pwmValue < 0 ? Constants.SensorTicksPerMotorRotation : 0);
		return pwmValue;
	}

	@Override
	public void update() {
	  
	}
}