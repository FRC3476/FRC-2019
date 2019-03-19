package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;
import frc.subsystem.Turret.*;

public class Lvl2ShipFront extends TemplateAuto implements Runnable {

    public Lvl2ShipFront() { 
        //Start position
        super(new Translation2D(48+18, 46));
    }

    public void moveToRocket(boolean raiseElevator, int dir) {

    }

    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        
        //Drive forward, blocking
        
        manipulator.setManipulatorState(ManipulatorState.HATCH);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(8*12+18, 46), 60);
        p1.addPoint(new Translation2D(168+12*4, 3*12+18+2), 60);
        p1.addPoint(new Translation2D(212+12*4+12, 3*12+18+2), 60);
        drive.setAutoPath(p1, false);
        turret.setDesired(-100, true);
        elevator.setHeight(Constants.HatchElevLow);
        while(!drive.isFinished()) {
            switch(stage) {
                case 0:
                    if(turret.isFinished()) {
                        turretFinishTime = Timer.getFPGATimestamp();
                        stage++;
                    }
                    break;
                case 1:
                    if(Timer.getFPGATimestamp() - turretFinishTime > 0.5) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    break;
            }
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
           // System.out.println(robotTracker.getOdometry().translationMat.getX() + " , " +robotTracker.getOdometry().translationMat.getY());
        }; 
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished() || !turret.isInRange()) {
          
        }

        collisionManager.score();
        //while(collisionManager.isScoring());
        
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