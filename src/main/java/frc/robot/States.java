package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class States {

    public static enum DriveStates {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates {
        aligned, unAligned, normal
    }

    public static enum ElevatorStates {
        idle, zero, l2, l3, l4, highAlgae, lowAlgae, barge
    }

    public static enum ArmStates {
        idle, azero, al1, al2, al3, al4, barge, aintake, apickup, cpickup, armAlgae, CoralAlgae, Processor
    }

    public static enum IntakeStates {
        idle, holdc, intakec, intakea, holda, eject, autoOuttake, intake;

        Command print(String string) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'print'");
        }
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;

    public static ElevatorStates elevatorState = ElevatorStates.idle;
    public static ArmStates armState = ArmStates.idle;
    public static IntakeStates intakeState = IntakeStates.idle;
    
}
