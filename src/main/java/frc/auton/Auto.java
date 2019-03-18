package frc.auton;

import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.math.*;
import frc.utility.control.motion.Path;


public class Auto extends TemplateAuto implements Runnable { 

    public boolean targetLeftRocket = true;

    // Prevents autonomous code from repeating itself and allows it to progress linearly
    boolean rerunPrevention1 = false;
    boolean rerunPrevention2 = false;

    public Auto(Translation2D start) {
        super (start);
    }

    @Override
    public void run() {

        if (!rerunPrevention1 && !rerunPrevention2) {
            // Move forward to rocket while bringing elevator up to first position and rotatng turret.
            // Rocket position is 12 feet left, 19 feet 2 inches forward.
            Path toRocket = new Path(here());
            toRocket.addPoint(new Translation2D(120, 54 * (targetLeftRocket ? 1 : -1)), 40);
            toRocket.addPoint(new Translation2D(234 - Constants.BumperWidth, 138 * (targetLeftRocket ? 1 : -1)), 40);
            drive.setAutoPath(toRocket, false);
            if(toRocket.getPercentage() > 0.5) {
                elevator.setHeightState(Elevator.ElevatorHeight.TOP); 
                turret.setAngle(28.75 * (targetLeftRocket ? 1 : -1));
            }
            if (toRocket.getPercentage() > 0.999 && turret.isFinished() && elevator.isFinished()) {
                rerunPrevention1 = true;
            }
        } else if (rerunPrevention1 && !rerunPrevention2) {
            // Place hatch
            hatchIntake.setIntakeState(HatchIntake.IntakeState.EJECT);
            if (hatchIntake.isFinished()) {
                rerunPrevention1 = false;
                rerunPrevention2 = true;
            }
        } else if (!rerunPrevention1 && rerunPrevention2) {
            // Move back to grab another hatch, rotating turret and moving elevator in the process
            Path toHatch = new Path(here());
            toHatch.addPoint(new Translation2D(Constants.BumperWidth, 138 * (targetLeftRocket ? 1 : -1)), 40);
            drive.setAutoPath(toHatch, false);
            if (!drive.isFinished()) {
                elevator.setHeightState(Elevator.ElevatorHeight.LOWER); 
                turret.setAngle(180);
            } else if (turret.isFinished() && elevator.isFinished()) {
                rerunPrevention1 = true;
            }
        } else {
            // Grab hatch
            hatchIntake.setIntakeState(HatchIntake.IntakeState.INTAKE);
            if (hatchIntake.isFinished()) {
                // Done with autonomous run!
            }
        }
    }

}