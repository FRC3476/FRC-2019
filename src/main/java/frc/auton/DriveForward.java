package frc.auton;

import frc.subsystem.*;
import frc.utility.Threaded;

public class DriveForward extends Threaded {
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

    public DriveForward() {
        
    }

    @Override
    public void update() {
        /*
    Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
    drivePath.addPoint(new Translation2D(10, 0), 40);
    drivePath.addPoint(new Translation2D(20, 0), 40);
    drivePath.addPoint(new Translation2D(30, 0), 40);
    drivePath.addPoint(new Translation2D(30, -15), 40);
    drive.setAutoPath(drivePath, false);
    */
    }

}