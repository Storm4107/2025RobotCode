package frc.robot;

public class States {

    public static enum DriveStates {
        standard, leftHold, rightHold, forwardHold, backwardHold, DynamicLock
    }

    public static enum AlignedStates {
        aligned, unAligned, normal
    }

    public static enum ElevatorStates {
        idle, zero, l1, l2, l3, l4, lowAlgae, highAlgae, barge, pickup
    }

    public static enum ArmStates {
        idle, azero, al1, al2, al3, al4, alowAlgae, ahighAlgae, abarge, aintake, apickup
    }

    public static enum IntakeStates {
        idle, holdc, intakec, intakea, holda, eject
    }

    public static enum AlgaeIntakeStates {
        idle, holda, intakea, eject
    }

    public static enum AlgaeArmStates {
        idle, zero, processor, pickup
    }

    public static DriveStates driveState = DriveStates.standard;
    public static AlignedStates alignedState = AlignedStates.normal;

    public static ElevatorStates elevatorState = ElevatorStates.idle;
    public static ArmStates armState = ArmStates.idle;
    public static IntakeStates intakeState = IntakeStates.idle;
    public static AlgaeIntakeStates algaeIntakeState = AlgaeIntakeStates.idle;
    public static AlgaeArmStates algaeArmState = AlgaeArmStates.idle;
    
}
