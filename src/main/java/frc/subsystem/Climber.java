package frc.subsystem;

import frc.robot.Constants;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class Climber{
    private static Climber instance = new Climber();
    private static CANSparkMax climberMaster;
    private static CANSparkMax climberSlave;
    private static CANPIDController climberPID;
    private static CANEncoder climberEncoder;

    //Get instance of the Climber
    public static Climber getInstance(){
        return instance;
    }

    public static void climb(double value){
        climberPID.setReference(value, ControlType.kPosition);
    }

    public static void configMotors(){
        climberSlave.follow(climberMaster,true);
    }

    private Climber(){
        climberMaster = new CANSparkMax(Constants.ClimberSpark1, MotorType.kBrushless);
        climberSlave = new CANSparkMax(Constants.ClimberSpark2, MotorType.kBrushless);
        climberPID = climberMaster.getPIDController();
        climberEncoder = climberMaster.getEncoder();
    }

}
