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
    private static Solenoid intakeSolenoid60Psi;

    //Stop wheels from spinning
    public static void stop(){
        leftTalon.set(ControlMode.PercentOutput, 0);
        rightTalon.set(ControlMode.PercentOutput, 0);
    }

    public static void setOuttake(IntakeState hatchBall){
        if(hatchBall==IntakeState.HATCH){

        }
        else if(hatchBall==IntakeState.BALL){

        }
    }

    //Intakes Hatch or Ball depending on the inputed enum
    public static void setIntake(IntakeState hatchBall){

        //Close intake and spin wheels inward 
        if(hatchBall==IntakeState.HATCH){
            intakeSolenoid60Psi.set(false);
            leftTalon.set(ControlMode.PercentOutput, Constants.IntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, -1*Constants.IntakeSpeed); 
        } 

        //Open intake and spin wheels outward
        else if(hatchBall==IntakeState.BALL){
            intakeSolenoid60Psi.set(true);
            leftTalon.set(ControlMode.PercentOutput, -1*Constants.IntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, Constants.IntakeSpeed);
        }
    }

    public enum IntakeState{
        HATCH,BALL,INTAKE,OUTTAKE
    }


    public Intake(){
        //Initialize variables
        leftTalon = new LazyTalonSRX(Constants.Intake1Id);
        rightTalon = new LazyTalonSRX(Constants.Intake2Id);
        intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoidId);
        intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoidId);
    }
}
