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

public class Lvl1Rocket extends TemplateAuto implements Runnable {

    public Lvl1Rocket(int side) { 
        //Start position
        super(new Translation2D(48+18, side*46), side);
    }


    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        
        //Drive forward, blocking
        
        manipulator.setManipulatorState(ManipulatorState.HATCH);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60);
        p1.addPoint(new Translation2D(144+48*2+5, this.side*(94+2)), 160);
        //rmeove -1 on x
        p1.addPoint(new Translation2D(169+48*2+5-1, this.side*(113+26+2-2)), 160);//-2
        drive.setAutoPath(p1, false);
        turret.setDesired(this.side*160, true);
        while(!drive.isFinished()) {
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
          
        }

        collisionManager.score();
        while(collisionManager.isScoring());
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, true);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(144+48*2, this.side*(94+2)), 160);
        p2.addPoint(new Translation2D(17 /*15 */, this.side*(136 + 2+ 5 + 2)), 160); //X=27, Y=135
                                                                //X18, Y = 136
                                                                //x15, 
        drive.setAutoPath(p2, true);
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
        //manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        //112-18 + 48*2, 161-18
        turret.setState(TurretState.SETPOINT);
        turret.setDesired(20*this.side, true);
        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(112-18 + 48*2+10 - 2, this.side*(161-20)), 160); //112-18 + 48*2+10
        drive.setAutoPath(p3, false);
        while(collisionManager.isRetrieving()); //test
        elevator.setHeight(Constants.HatchElevMid);
        while(!drive.isFinished()) {};

        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished() || !turret.isInRange()) {
          
        }

        collisionManager.score();
        
    }
      
    
    

}