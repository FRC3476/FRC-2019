package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;

public class Intake{
    private static LazyTalonSRX leftTalon;
    private static LazyTalonSRX rightTalon;
    private static Solenoid intakeLeftSolenoid;
    private static Solenoid intakeRightSolenoid;

    public static void setSolenoid(IntakeState state){
        switch(state){
            case HATCH:
                intakeLeftSolenoid.set(true);
                intakeRightSolenoid.set(true);
            break;

            case BALL:
                intakeLeftSolenoid.set(false); 
                intakeRightSolenoid.set(false);
            break;
        }
    }

    public static void setIntakeSpeed(IntakeSpeed speed){
        switch(speed){
            case HATCH_INTAKE_MEDIUM:
                
            break;

            case HATCH_INTAKE_FAST:
          
            break;

            case HATCH_OUTTAKE_MEDIUM:

            break;

            case HATCH_OUTTAKE_FAST:

            break;

            case BALL_INTAKE_MEDIUM:

            break;

            case BALL_INTAKE_FAST:

            break;

            case BALL_OUTTAKE_MEDIUM:

            break;

            case BALL_OUTTAKE_FAST:

            break;
        }
    }

    public enum IntakeState{
        HATCH,BALL
    }

    public enum IntakeSpeed{
        BALL_INTAKE_MEDIUM, BALL_INTAKE_FAST, HATCH_INTAKE_MEDIUM, HATCH_INTAKE_FAST,
        BALL_OUTTAKE_MEDIUM, BALL_OUTTAKE_FAST, HATCH_OUTTAKE_MEDIUM, HATCH_OUTTAKE_FAST
    }

    public Intake(){
        leftTalon = new LazyTalonSRX(Constants.Intake1Id);
        rightTalon = new LazyTalonSRX(Constants.Intake2Id);
        intakeLeftSolenoid = new Solenoid(Constants.IntakeSolenoidId);
        intakeRightSolenoid = new Solenoid(Constants.IntakeSolenoidId);
    }
}
