package frc.subsystem;

import frc.utility.LazyTalonSRX;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class BallIntake{
    private static BallIntake instance = new BallIntake();
    private static LazyTalonSRX intakeTalon;

    public static BallIntake getInstance(){
        return instance;
    }

    public void intakeBall(){
        intakeTalon.set(ControlMode.PercentOutput,Constants.BallIntakePercentSpeed);
    }

    public void stop(){
        intakeTalon.set(ControlMode.PercentOutput,0);
    }

    public BallIntake(){
        intakeTalon = new LazyTalonSRX(Constants.BallIntakeTalonId);
    }
}