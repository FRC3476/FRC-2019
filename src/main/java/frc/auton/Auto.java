package frc.auton;

import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.Threaded;
import frc.utility.control.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;


public class Auto implements Runnable { 
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

    public Auto(Translation2D start) {
        RobotTracker.getInstance().setInitialTranslation(start);
    }

    public Translation2D here() {
        return RobotTracker.getInstance().getOdometry().translationMat;
    }

    @Override
    public void run() {

        // Move forward to rocket while bringing elevator up to first position.
        // Rocket position is 12 feet left, 19 feet 2 inches forward.
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(-30, 30), 40);
        p1.addPoint(new Translation2D(-138, 234 - Constants.BumperWidth), 40);
        drive.setAutoPath(p1, false);
        while(!drive.isFinished()) {
            if(p1.getPercentage() > 0.5) elevator.setHeightState(Elevator.ElevatorHeight.BASE); 
        };

        // Rotate turret to face rocket and place hatch

        // Move back to grab another hatch
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(-138, Constants.BumperWidth), 40);
        drive.setAutoPath(p2, false);
        while(!drive.isFinished()) {
            if(p2.getPercentage() > 0.5) elevator.setHeightState(Elevator.ElevatorHeight.BASE); 
        };

        // Rotate turret to wall and grab hatch

        // Return to rocket
        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(-138, 234 - Constants.BumperWidth), 40);
        drive.setAutoPath(p3, false);
        while(!drive.isFinished()) {
            if(p3.getPercentage() > 0.5) elevator.setHeightState(Elevator.ElevatorHeight.MIDDLE); 
        };

        // Rotate turret to rocket and release hatch

        // Return to hatch dropoff and grab another
        Path p4 = new Path(here());
        p4.addPoint(new Translation2D(-138, Constants.BumperWidth), 40);
        drive.setAutoPath(p4, false);
        while(!drive.isFinished()) {
            if(p4.getPercentage() > 0.5) elevator.setHeightState(Elevator.ElevatorHeight.BASE); 
        };

        // Rotate turret and grab hatch

        // Return to rocket
        Path p5 = new Path(here());
        p5.addPoint(new Translation2D(-138, 234 - Constants.BumperWidth), 40);
        drive.setAutoPath(p5, false);
        while(!drive.isFinished()) {
            if(p5.getPercentage() > 0.5) elevator.setHeightState(Elevator.ElevatorHeight.TOP); 
        };

        // Rotate turret and release hatch
    }

}