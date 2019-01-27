package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;

public class BallIntake{
    private static LazyTalonSRX leftTalon;
    private static LazyTalonSRX rightTalon;
    private static Solenoid intakeLeftSolenoid;
    private static Solenoid intakeRightSolenoid;

    public static void outtakeForewared(IntakeState state , IntakeState speed){
        if(state==IntakeState.BALL){
            
        }
    }

    public static double ballIntakeOuttake(IntakeState speed, IntakeState inOut){
        if(inOut==IntakeState.INTAKE){
            if(speed==IntakeState.MEDIUM){
                return -1d*Constants.IntakeMediumRPM;
            }

            else if(speed==IntakeState.FAST){
                return -1d*Constants.IntakeFastRPM;
            }
        }

        else if(inOut==IntakeState.OUTTAKE){
            if(speed==IntakeState.MEDIUM){
                return Constants.IntakeMediumRPM;
            }

            else if(speed==IntakeState.FAST){
                return Constants.IntakeFastRPM;
            }
        }
        return 0d;
    }

    public enum IntakeState{
        HATCH,BALL,INTAKE,OUTTAKE,MEDIUM,FAST
    }


    public BallIntake(){
        leftTalon = new LazyTalonSRX(Constants.Intake1Id);
        rightTalon = new LazyTalonSRX(Constants.Intake2Id);
        intakeLeftSolenoid = new Solenoid(Constants.IntakeSolenoidId);
        intakeRightSolenoid = new Solenoid(Constants.IntakeSolenoidId);
    }
}
