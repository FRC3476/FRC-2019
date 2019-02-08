package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

class HatchIntake {
	
	public enum HatchIntakeState{
		STOW, HANDOFF, INTAKE
	}
	
	private static final HatchIntake instance = new HatchIntake();
	
	public static HatchIntake getInstance(){
		return instance;    
	}
	
	private static LazyTalonSRX HatchHandoffMotor;
	private static LazyTalonSRX HatchIntakeMotor;
	
	private HatchIntake(){
		HatchIntakeMotor = new LazyTalonSRX(Constants.HatchIntakeMotorId);
		//HatchHandoffMotor = new LazyTalonSRX(Constants.HatchHandoffId);
		//HatchHandoffMotor.setSelectedSensorPosition(0,0,10);
	}
	
	// Stop intake wheels
	public void stop(){
		HatchIntakeMotor.set(ControlMode.PercentOutput, 0);
	}
	
	// Spin wheels to intake hatch
	public void IntakeHatch(){
		HatchIntakeMotor.set(ControlMode.PercentOutput, Constants.HatchIntakePower);
	}
	
	// Set intake hatch state
	public void setHatchIntake(HatchIntakeState state){
		switch(state){
			case STOW:
				setAngle(Constants.HatchStowAngle);
				break;
			case HANDOFF:
				setAngle(Constants.HatchHandoffAngle);
				break;
			case INTAKE:
				setAngle(Constants.HatchIntakeAngle);
				break;
		}
	}
	
	public void setAngle(double angle){
		HatchHandoffMotor.set(ControlMode.Position, angle*Constants.EncoderTicksPerDegree);
	}
}