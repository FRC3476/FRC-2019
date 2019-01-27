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

    public static void setSideOuttake(OuttakeDirection dir){
        if(dir == OuttakeDirection.LEFT){
            rightTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
            leftTalon.set(ControlMode.PercentOutput, Constants.LowIntakeSpeed);
        }
        else if(dir == OuttakeDirection.RIGHT){
            rightTalon.set(ControlMode.PercentOutput, Constants.LowIntakeSpeed);
            leftTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
        }
    }

    //Outtake the ball or hatch straight
    public static void setStraightOuttake(IntakeState hatchBall){
        if(hatchBall==IntakeState.HATCH){
            leftTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, -1*Constants.NormalIntakeSpeed);
        }
        else if(hatchBall==IntakeState.BALL){
            leftTalon.set(ControlMode.PercentOutput, -1*Constants.NormalIntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed);          
        }
    }

    //Intakes Hatch or Ball depending on the inputed enum
    public static void setIntake(IntakeState hatchBall){

        //Close intake and spin wheels inward 
        if(hatchBall==IntakeState.HATCH){
            intakeSolenoid60Psi.set(false);
            leftTalon.set(ControlMode.PercentOutput, -1*Constants.NormalIntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed); 
        } 

        //Open intake and spin wheels outward
        else if(hatchBall==IntakeState.BALL){
            intakeSolenoid60Psi.set(true);
            leftTalon.set(ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
            rightTalon.set(ControlMode.PercentOutput, -1*Constants.NormalIntakeSpeed);
        }
    }

    public enum IntakeState{
        HATCH,BALL,INTAKE,OUTTAKE
    }

    public enum OuttakeDirection{
        LEFT,RIGHT
    }


    public Intake(){
        //Initialize variables
        leftTalon = new LazyTalonSRX(Constants.Intake1Id);
        rightTalon = new LazyTalonSRX(Constants.Intake2Id);
        intakeSolenoid60Psi = new Solenoid(Constants.IntakeSolenoidId);
    }
}
