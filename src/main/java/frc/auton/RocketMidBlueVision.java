package frc.auton; //blue

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.subsystem.Turret.*;
import frc.subsystem.Arm.*;
import frc.utility.VisionTarget;;

public class RocketMidBlueVision extends TemplateAuto implements Runnable {

    public RocketMidBlueVision(int side, double startX) { 
        //Start position
        super(new Translation2D(startX, side*46), side);
    }

    Path getVisionPath() {
        VisionTarget selected =turret.getSelected();
        double xT = selected.loc_x + robotTracker.getOdometry().translationMat.getX();
        double yT = selected.loc_y + robotTracker.getOdometry().translationMat.getY();
        

        Translation2D robotHeading = robotTracker.getOdometry().rotationMat.getUnitVector().scale(5);
        Translation2D robotpheading = robotHeading.translateBy(here()); //#1

        Translation2D target = new Translation2D(selected.loc_x, selected.loc_y).rotateBy(dir());
    
        Translation2D targetprobot = new Translation2D(xT, yT);

        Translation2D targetmheading = target.getUnitVector().translateBy(robotHeading.inverse());
    
        Translation2D mod = target.getUnitVector().inverse().scale(27) ;//.translateBy(targetprobot);
        Translation2D loc = mod.translateBy(targetprobot); //#2
        //Translation2D target.translateBy(robotTracker.getOdometry().translationMat.inverse());
        System.out.println("target no rotate " + new Translation2D(selected.loc_x, selected.loc_y) + "dir" + dir() + " target " + target +  " mod: " + mod + " loc: " + loc + " targetprobot " + targetprobot);
        //drive.setRotation(targetprobot.getAngle(loc));
        Path pV = new Path(here());
        //pV.addPoint(robotpheading, 20);
        pV.addPoint(loc, 20);
       // pV.addPoint(loc)
       return pV;
    }


    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        
        //Drive forward, blocking
        System.out.println("rocket mid blue");  
        manipulator.setManipulatorState(ManipulatorState.HATCH);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        
        Path p1 = new Path(here());
        /*
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60);
        p1.addPoint(new Translation2D(144+48*2+5+15, this.side*(94+2)), 160);
        p1.addPoint(new Translation2D(169+48*2+5-1 -5 +15, this.side*(113+26+2-2-8-15)), 160);//-2
        */
        /*
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60);
        p1.addPoint(new Translation2D(144+48*2+5+20+10, this.side*(94+2)), 160);
        p1.addPoint(new Translation2D(144+48*2+5+20+10+20, this.side*(94+2+15)), 160);
        */
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60);
        p1.addPoint(new Translation2D(144+48*2+5, this.side*(94+2)), 160);
        //rmeove -1 on x
        p1.addPoint(new Translation2D(169+48*2+5-1 -5 /*expiermental*/ -2 + 15, this.side*(113+26+2-2)), 160);//-2

        //rmeove -1 on x
        //p1.addPoint(new Translation2D(169+48*2+5-1 -5 /*expiermental*/ -2 + 20, this.side*(113+26+2-2)), 160);//-2

        
        /*
            if(startX < 50) {
                Path dropOff = new Path(here());
                dropOff.addPoint(new Translation2D(8*12+18, this.side*46), 60);
                drive.setAutoPath(dropOff);
                while(!drive.isFinished() && !killSwitch);

                double reverseTime = Timer.getFPGATimestamp();
                Path reverse = new Path(here));
                reverse.addPoint(new Translation2D(0, this.side*46), 60);
                drive.setAutoPath(reverse);
                while(Timer.getFPGATimestamp() - reverseTime < 0.5);

                Path rocket1 = new Path(here));
                rocket1.addPoint(new Translation2D(144+48*2+5, this.side*(94+2)), 160);
                rocket1.addPoint(new Translation2D(169+48*2+5-1 -5, this.side*(113+26+2-2)), 160);//-2
                drive.setAutoPath(rocket1);
                

            } 
            else drive.setAutoPath(p1, false);
        */

        drive.setAutoPath(p1, false);
        turret.setDesired(this.side*140, true);
        while(!drive.isFinished() && !killSwitch) {
            if(isDead()) return;

            if(p1.getPercentage() > 0.3) elevator.setHeight(Constants.HatchElevMid);
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
            if(isDead()) return;
            }
        

        collisionManager.score();
        while(collisionManager.isScoring()) if(isDead()) return;
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, true);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(144+48*2, this.side*(94+2)), 160);
        p2.addPoint(new Translation2D(17 -2/*20 + 5*/  -2 + 20/*expiermental*/, this.side*(136 + 2+ 5 + 2)), 160); //X=27, Y=135
                                                                //X18, Y = 136
                                                                //x15, 
        drive.setAutoPath(p2, true);
        elevator.setHeight(Constants.HatchElevLow);
        while(!drive.isFinished())  if(isDead()) return;
        
        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished())if(isDead()) return;
        drive.setAutoPath(getVisionPath(), true);
        while(!drive.isFinished()) if(isDead()) return;

        while(!turret.isFinished() || !turret.isInRange())if(isDead()) return;

        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        arm.setState(ArmState.EXTEND);
        elevator.setHeight(Constants.HatchHP); 
        double intakeAttemptTime = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 0.75)if(isDead()) return;
        //arm.setState(ArmState.RETRACT);
        collisionManager.retrieveHatch();
        //elevator.setHeight(Constants.HatchElevLow);
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 1.5)if(isDead()) return;
        //manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        //112-18 + 48*2, 161-18
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(20*this.side, true);
        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(112-18 + 48*2+10 - 2 - 10 + 9 - 50 /*expiermental*/, this.side*(161-20)), 160); //112-18 + 48*2+10
        drive.setAutoPath(p3, false);
        while(collisionManager.isRetrieving())if(isDead()) return;//test
        elevator.setHeight(Constants.HatchElevMid);
        while(!drive.isFinished()) {if(isDead()) return;};

        while(!turret.isFinished()) {
            if(isDead()) return;
        }
        //OrangeUtility.sleep(1000);
        turret.setState(TurretState.VISION);
        //OrangeUtility.sleep(1000);
             
        while(!turret.isFinished()) {
            if(isDead()) return;
        }

        drive.setAutoPath(getVisionPath(), false);
        while(!drive.isFinished()) {if(isDead()) return;};
        

        while(!turret.isFinished()  || !turret.isInRange()) {
            if(isDead()) return;
        }

        collisionManager.score();
        
    }
      
    
    

}