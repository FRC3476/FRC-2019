package frc.subsystem;

import frc.subsystem.Elevator;
import frc.robot.Constants;
import frc.subsystem.Arm;

public class ElevArm{
    private static Elevator elevator;
    private static Arm arm;
    private static ElevArm instance = new ElevArm();

    public static ElevArm getInstance(){
        return instance;
    }

    //Sets angle and height of the intake by moving the elevator and the arm
    public static void setAngleHeight(double height, double angle){
        arm.setAngle(angle);
        elevator.setHeight(height);
    }

    //
    public static void setIntake(IntakeHeight level){
        switch(level){
            case INTAKE:
                setAngleHeight(Constants.ElevatorIntakeHeight,Constants.ArmIntakingAngle);//Random Height and angle
            break;

            case BASE:
                setAngleHeight(Constants.RocketBaseHeight-arm.getHeightSetPoint(),Constants.ExpectedArmAngle);
            break;

            case MIDDLE:
                setAngleHeight(Constants.RocketMiddleHeight-arm.getHeightSetPoint(),Constants.ExpectedArmAngle);
            break;

            case TOP:
                setAngleHeight(Constants.RocketTopHeight-arm.getHeightSetPoint(),Constants.ExpectedArmAngle);
            break;
        }
    }

    public static double getIntakeHeight(){
        double height = arm.getHeight()+elevator.getHeight();
        return height;
    }

	public enum IntakeHeight {
		INTAKE, BASE, MIDDLE, TOP
	}

    public ElevArm(){
        elevator = Elevator.getInstance();
        arm = Arm.getInstance();
    }
}