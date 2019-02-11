package frc.subsystem;

import frc.subsystem.Elevator.ElevatorHeight;
import frc.subsystem.Arm;
import frc.subsystem.Elevator;
import frc.subsystem.HatchIntake;
import frc.subsystem.Manipulator;
import frc.subsystem.Arm.SolenoidState;
import frc.subsystem.HatchIntake.HatchIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.utility.OrangeUtility;

public class GroundHatchArm{
    private static GroundHatchArm instance = new GroundHatchArm();
    
    private static Arm arm;
    private static Elevator elevator;
    private static HatchIntake hatchIntake;
    private static Manipulator manipulator;

    public GroundHatchArm getInstance(){
        return instance;
    }

    public void handoffFromGround(){
        //Put manipulator on hatch intake position
        manipulator.setIntake(ManipulatorState.HATCH_INTAKE);

        //Punch the arm out
        arm.punch(SolenoidState.OUT);

        //Put ground hatch intake into handoff position
        hatchIntake.setHatchIntake(HatchIntakeState.HANDOFF);
        
        //Do nothing for 200 mls
        OrangeUtility.sleep(200);

        //Raise the elevator a bit
        elevator.setHeight(12);

        //Do nothing again
        OrangeUtility.sleep(200);

        //Put the ground hatch intake in stow positon
        hatchIntake.setHatchIntake(HatchIntakeState.STOW);

        //Lower the elevator back down
        elevator.setHeightState(ElevatorHeight.BASE);

        //Done, the hatch should be on the manipulator
    }


    private GroundHatchArm(){
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        hatchIntake = HatchIntake.getInstance();
        manipulator = Manipulator.getInstance();
    }
}