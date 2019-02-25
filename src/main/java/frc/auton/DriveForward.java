package frc.auton;

import frc.subsystem.*;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;

public class DriveForward implements Runnable {
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
    public void run() {
        Path p1 = new Path(RobotTracker.getInstance().getOdometry().translationMat);
        p1.addPoint(new Translation2D(10, 0), 40);
        p1.addPoint(new Translation2D(20, 0), 40);
        p1.addPoint(new Translation2D(30, 0), 40);
        
        drive.setAutoPath(p1, false);
        while(!drive.isFinished());

    }

    

}