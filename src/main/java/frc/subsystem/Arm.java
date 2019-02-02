package frc.subsystem;

import frc.utility.OrangeUtility;
import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

public class Arm{
    private static Solenoid puncher;
    private static Arm instance = new Arm();

    public static Arm getInstance(){
        return instance;
    }

    public void punch(SolenoidState state){
        if(state == SolenoidState.IN){
            puncher.set(false);
        }

        else if(state == SolenoidState.OUT){
            puncher.set(true);
        }
    }


    private enum SolenoidState{
        IN,OUT;
    }
    private Arm(){
        puncher = new Solenoid(Constants.ArmPuncherId);
    }
}