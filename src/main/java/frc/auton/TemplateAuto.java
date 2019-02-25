package frc.auton;

import frc.subsystem.*;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;


public class TemplateAuto implements Runnable { 
    Drive drive = Drive.getInstance();
    Elevator elevator = Elevator.getInstance();
    Arm arm = Arm.getInstance();
    BallIntake ballIntake = BallIntake.getInstance();
    Climber climber = Climber.getInstance(); 
    CollisionManager collisionManager = CollisionManager.getInstance();
    RobotTracker robotTracker = RobotTracker.getInstance();
    HatchIntake hatchIntake = HatchIntake.getInstance();
    Manipulator manipulator = Manipulator.getInstance();
    Turret turret = Turret.getInstance();

    public TemplateAuto(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public Translation2D here() {
        return RobotTracker.getInstance().getOdometry().translationMat;
    }

    @Override
    public void run() {

    }

}