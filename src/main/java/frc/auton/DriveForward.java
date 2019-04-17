package frc.auton;

import frc.subsystem.*;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;

public class DriveForward extends TemplateAuto implements Runnable {

    public DriveForward() { 
        //Start position
        super(new Translation2D(0, 0));
    }

    public void moveToRocket(boolean raiseElevator, int dir) {

    }

    @Override
    public void run() {
        
        //Drive forward, blocking
        //Path p1 = new Path(here());
        //p1.addPoint(new Translation2D(60, 0), 40);
        //p1.addPoint(new Translation2D(60, 60), 40);
        //drive.setAutoPath(p1, false);
        drive.setRotation(Rotation2D.fromDegrees(180));
        while(!drive.isFinished()) {
           // System.out.println(robotTracker.getOdometry().translationMat.getX() + " , " +robotTracker.getOdometry().translationMat.getY());
        }; 
        
        /*
        //Drive forward whilst moving elevator, non blocking
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(50, 0), 40);
        drive.setAutoPath(p2, false);
        while(!drive.isFinished()) {
            if(p2.getPercentage() > 0.5) elevator.setHeight(400); 
        }; 

        //Move elevator, blocking
        elevator.setHeight(200);
        while(!elevator.isFinished()); 
        */
    }

    

}