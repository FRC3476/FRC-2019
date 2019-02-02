package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

class HatchIntake{
    private static HatchIntake instance = new HatchIntake();
    
    private static LazyTalonSRX HatchHandoffMotor;
    private static LazyTalonSRX HatchIntakeMotor;

    //Return instance of intake
    public static HatchIntake getInstance(){
        return instance;    
    }

    //Stop intake wheels
    public void stop(){
        HatchIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    //Spin wheels to intake hatch
    public void IntakeHatch(){
        HatchIntakeMotor.set(ControlMode.PercentOutput, Constants.HatchIntakeSpeed);
    }

    //Handoff the hatch to the Manipulator
    public void HandoffHatch(){
        HatchHandoffMotor.set(ControlMode.Position, Constants.HatchHandoffAngle*Constants.DegreesPerEncoderTick);
    }

    public void LowerHatchHandoff(){
        HatchHandoffMotor.set(ControlMode.Position, 0);
    }

    //Initialize variables
    private HatchIntake(){
        HatchIntakeMotor = new LazyTalonSRX(Constants.HatchIntakeMotorId);
        HatchHandoffMotor = new LazyTalonSRX(Constants.HatchHandoffId);
        HatchHandoffMotor.setSelectedSensorPosition(0,0,10);
    }
}