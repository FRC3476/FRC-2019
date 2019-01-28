// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Solenoid;
import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

public class Intake{
    //Talons objects
    private static LazyTalonSRX leftTalon;
    private static LazyTalonSRX rightTalon;
    private static Solenoid intakeSolenoid;
	private static final Intake instance = new Intake();

    public static Intake getInstance(){
        return instance;
    }

    // Stop wheels from spinning
    public static void stop () {
        leftTalon.set (ControlMode.PercentOutput, 0);
        rightTalon.set (ControlMode.PercentOutput, 0);
    }

    //Outtake to the left or to the right
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
    public static void setStraightOuttake (IntakeState hatchBall) {
        if (hatchBall == IntakeState.HATCH_OUTTAKE) {
            leftTalon.set (ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
            rightTalon.set (ControlMode.PercentOutput, -1 * Constants.NormalIntakeSpeed);
        } else if (hatchBall == IntakeState.BALL_OUTTAKE) {
            leftTalon.set (ControlMode.PercentOutput, -1 * Constants.NormalIntakeSpeed);
            rightTalon.set (ControlMode.PercentOutput, Constants.NormalIntakeSpeed);          
        }
    }

    // Intakes Hatch or Ball depending on the inputed enum
    public static void setIntake (IntakeState hatchBall) {

        // Close intake and spin wheels inward 
        if (hatchBall == IntakeState.HATCH_INTAKE) {
            intakeSolenoid.set (false);
            leftTalon.set (ControlMode.PercentOutput, -1 * Constants.NormalIntakeSpeed);
            rightTalon.set (ControlMode.PercentOutput, Constants.NormalIntakeSpeed); 
        } // Open intake and spin wheels outward
        else if (hatchBall == IntakeState.BALL_INTAKE) { 
            intakeSolenoid.set (true);
            leftTalon.set (ControlMode.PercentOutput, Constants.NormalIntakeSpeed);
            rightTalon.set (ControlMode.PercentOutput, -1 * Constants.NormalIntakeSpeed);
        }
    }

    public enum IntakeState {
        HATCH_INTAKE, BALL_INTAKE, HATCH_OUTTAKE, BALL_OUTTAKE
    }

    public enum OuttakeDirection {
        LEFT, RIGHT
    }


    private Intake(){
        //Initialize variables
        leftTalon = new LazyTalonSRX(Constants.Intake1Id);
        rightTalon = new LazyTalonSRX(Constants.Intake2Id);
        intakeSolenoid = new Solenoid(Constants.IntakeSolenoidId);
    }
}
