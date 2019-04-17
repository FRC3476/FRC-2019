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
import frc.subsystem.Arm.*;

public class ShipF_1 extends TemplateAuto implements Runnable {

    public ShipF_1(int side, double startX) { 
        //Start position
        super(new Translation2D(startX, side*46), side);
        
    }

    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        
        //Drive forward, blocking
        
        manipulator.setManipulatorState(ManipulatorState.HATCH);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(8*12 + 128 -18 -4 -2*12, this.side*11.5), 60); //TEMPORARY
        p1.addPoint(new Translation2D(8*12 + 128 -18 -4, this.side*11.5), 120);
        drive.setAutoPath(p1, false);
        turret.setDesired(0, true);
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
        //turret.setState(TurretState.SETPOINT); temp
        //turret.restoreSetpoint();
        collisionManager.score();
        while(collisionManager.isScoring());
        turret.setState(TurretState.SETPOINT);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(13*8, this.side*135), 120-6);
        p2.addPoint(new Translation2D(25, this.side*136), 160-6); //X=27, Y=135
        drive.setAutoPath(p2, true);

        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, true);
        elevator.setHeight(Constants.HatchElevLow);
        while(!drive.isFinished());
        
        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished() || !turret.isInRange());
        
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        arm.setState(ArmState.EXTEND);
        elevator.setHeight(Constants.HatchHP);
        double intakeAttemptTime = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 0.75);
        //arm.setState(ArmState.RETRACT);
        collisionManager.retrieveHatch();
        //elevator.setHeight(Constants.HatchElevLow);
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 1.5);

        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(168+12*4, this.side*(3*12+18+0)), 120);
        p3.addPoint(new Translation2D(212+12*4+12-1.5+2 /*0*/, this.side*(3*12+18-2/*0 */-1)), 120);
        drive.setAutoPath(p3, false);

        turret.setState(TurretState.SETPOINT);
        turret.setDesired(this.side*-90, true);
        while(!turret.isFinished());
        //manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        //turret.setState(TurretState.SETPOINT);
        
        while(!drive.isFinished());
        turret.setState(TurretState.VISION);


        while(!turret.isFinished() || !turret.isInRange()) {
          
        }

        collisionManager.score();
        while(collisionManager.isScoring());

        Path p4 = new Path(here());
        p4.addPoint(new Translation2D(212+12*4+12+24+4+8 + 21.5, this.side*(3*12+18-2-1)), 120);
        p4.addPoint(new Translation2D(212+12*4+12+24+4+8 + 21.5, this.side*(3*12+18-2-1+24)), 120);
        drive.setAutoPath(p4, false);
        while(!drive.isFinished());
        collisionManager.extendBallIntake();
        while(!collisionManager.isBallIntakeOut());
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