package frc.robot;

import static edu.wpi.first.units.Units.Newton;

import java.lang.Thread.State;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.States.ArmStates;
import frc.robot.States.ElevatorStates;
import frc.robot.States.IntakeStates;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorCommands.ElevatorStateCommand;
import frc.robot.commands.ElevatorCommands.ElevatorVoltageOverrideCommand;
import frc.robot.commands.ElevatorCommands.ZeroElevatorCommand;
import frc.robot.commands.ArmCommands.ArmStateCommand;
import frc.robot.commands.ArmCommands.ArmVoltageOverrideCommand;
import frc.robot.commands.ArmCommands.ZeroArmCommand;
import frc.robot.commands.IntakeCommands.AutoIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeStateCommand;
import frc.robot.commands.IntakeCommands.IntakeVoltageOverrideCommand;

import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

   /* Driver Controls */
	private final int translationAxis = 1;
	private final int strafeAxis = 0;
	private final int rotationAxis = 2;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, 17); //Joystick trigger
    private final JoystickButton robotCentric = new JoystickButton(driver, 16); //Joystick B16

    private final JoystickButton allZero = new JoystickButton(driver, 21);

    private final JoystickButton dampen = new JoystickButton(driver, 18);

    private final JoystickButton zeroElevator = new JoystickButton(driver, 7);
    private final JoystickButton l4 = new JoystickButton(operator, 5);
    private final JoystickButton barge = new JoystickButton(driver, 1);

    private final JoystickButton ResetElevator = new JoystickButton(driver, 8);

    private final POVButton elevatorUp = new POVButton(operator, 0);
    private final POVButton elevatorDown = new POVButton(operator, 180);
    private final POVButton armUp = new POVButton(operator, 270);
    private final POVButton armDown = new POVButton(operator, 90);

    private final JoystickButton l2 = new JoystickButton(operator, 4);
    private final JoystickButton l3 = new JoystickButton(operator, 6);
    private final JoystickButton al4 = new JoystickButton(operator, 5);

    private final JoystickButton intakec = new JoystickButton(operator, 2);
    private final Trigger outtake = new Trigger(() -> (operator.getRawAxis(3) > 0));
    private final Trigger zeroArm = new Trigger(() -> (operator.getRawAxis(2) > 0));

    private final JoystickButton apickup = new JoystickButton(operator, 8);

    private final JoystickButton zero = new JoystickButton(operator, 8);
    //private final JoystickButton processor = new JoystickButton(operator, 10);
    private final JoystickButton pickup = new JoystickButton(operator, 9);
    private final JoystickButton highAlgae = new JoystickButton(driver, 2);
    private final JoystickButton lowAlgae = new JoystickButton(driver, 3);
    private final JoystickButton CoralAlgae = new JoystickButton(operator, 7);

    private final JoystickButton Processor = new JoystickButton(operator, 1);

    private final JoystickButton al1 = new JoystickButton(operator, 3);


    //private final JoystickButton DynamicLock = new JoystickButton(driver, XboxController.Button.kX.value);

    //private final Trigger forwardHold = new Trigger(() -> (driver.getRawAxis(4) > 0.75));
    //private final Trigger backwardHold = new Trigger(() -> (driver.getRawAxis(4) < -0.75));

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final Elevator s_Elevator = new Elevator();
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake();

    //private final Vision s_Vision = new Vision(s_PoseEstimator);

    /* AutoChooser */
    private final SendableChooser<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new SwerveCommand(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> dampen.getAsBoolean(),
                () -> 0 // Dynamic heading placeholder
            )
        );

        s_Elevator.setDefaultCommand(
            new ElevatorStateCommand(s_Elevator)
        );

        s_Arm.setDefaultCommand(
            new ArmStateCommand(s_Arm)
        );

        s_Intake.setDefaultCommand(
            new IntakeStateCommand(s_Intake)
        );

        //new Event

        // Configure the button bindings
        configureButtonBindings();


        //Pathplanner commands - templates TODO: Make named commands

        //Elevator commands
        NamedCommands.registerCommand("ElevatorL2", new InstantCommand(() -> States.elevatorState = ElevatorStates.l2));
        NamedCommands.registerCommand("ElevatorL3", new InstantCommand(() -> States.elevatorState = ElevatorStates.l3));
        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(() -> States.elevatorState = ElevatorStates.l4));
        NamedCommands.registerCommand("ElevatorZero", new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));

        //Arm Commands
        NamedCommands.registerCommand("Processor", new InstantCommand(() -> States.armState = ArmStates.Processor));
        NamedCommands.registerCommand("CoralAlgae", new InstantCommand(() -> States.armState = ArmStates.CoralAlgae));
        NamedCommands.registerCommand("ArmL1", new InstantCommand(() -> States.armState = ArmStates.al1));
        NamedCommands.registerCommand("ArmL2", new InstantCommand(() -> States.armState = ArmStates.al2));
        NamedCommands.registerCommand("ArmL3", new InstantCommand(() -> States.armState = ArmStates.al3));
        NamedCommands.registerCommand("ArmL4", new InstantCommand(() -> States.armState = ArmStates.al4));
        //THIS BREAKS THE ROBOT
        //NamedCommands.registerCommand("ArmZero", new InstantCommand(() -> States.armState = ArmStates.azero));

        //Intake commands
        NamedCommands.registerCommand("Outtake", new IntakeVoltageOverrideCommand(s_Intake, () -> -6).withTimeout(1));
        NamedCommands.registerCommand("IntakeOverride", new IntakeVoltageOverrideCommand(s_Intake, () -> 9).withTimeout(1));
        NamedCommands.registerCommand("Intake", new AutoIntakeCommand(s_Intake).withTimeout(10));

        //NamedCommands.registerCommand("Intake", new InstantCommand(() -> States.intakeState = IntakeStates.intakec));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("new auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        allZero.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        allZero.onTrue(new InstantCommand(() -> States.armState = ArmStates.azero));

        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        zeroElevator.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        barge.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.barge));
        barge.onTrue(new SequentialCommandGroup(new InstantCommand(() -> States.elevatorState = ElevatorStates.barge), new WaitCommand(2), new InstantCommand(()->States.armState = ArmStates.barge)));
        l4.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l4));
        l3.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l3));
        l2.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l2));

        ResetElevator.whileTrue(new ZeroElevatorCommand(s_Elevator));

        elevatorUp.whileTrue(new ElevatorVoltageOverrideCommand(s_Elevator, () -> 1));
        elevatorDown.whileTrue(new ElevatorVoltageOverrideCommand(s_Elevator, () -> -1));

        zeroArm.onTrue(new InstantCommand(() -> States.armState = ArmStates.azero));
        al1.onTrue(new InstantCommand(() -> States.armState = ArmStates.al1));
        al1.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        l2.onTrue(new InstantCommand(() -> States.armState = ArmStates.al2));
        l3.onTrue(new InstantCommand(() -> States.armState = ArmStates.al3));
        al4.onTrue(new InstantCommand(() -> States.armState = ArmStates.al4));
        apickup.onTrue(new InstantCommand(() -> States.armState = ArmStates.apickup));
        apickup.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        pickup.onTrue(new InstantCommand(() -> States.armState = ArmStates.cpickup));
        pickup.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        highAlgae.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.highAlgae));
        lowAlgae.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.lowAlgae));
        highAlgae.onTrue(new InstantCommand(() -> States.armState = ArmStates.armAlgae));
        lowAlgae.onTrue(new InstantCommand(() -> States.armState = ArmStates.armAlgae));
        CoralAlgae.onTrue(new InstantCommand(() -> States.armState = ArmStates.CoralAlgae));
        Processor.onTrue(new InstantCommand(() -> States.armState = ArmStates.Processor));

        armUp.whileTrue(new ArmVoltageOverrideCommand(s_Arm, () -> 3));
        armDown.whileTrue(new ArmVoltageOverrideCommand(s_Arm, () -> -3));

        intakec.onTrue(new InstantCommand(() -> States.intakeState = IntakeStates.intakec));
        intakec.onFalse(new InstantCommand(() -> States.intakeState = IntakeStates.idle));
        //intakec.and(() -> s_Intake.holdingWithCurrent()).onTrue(new InstantCommand(() -> States.intakeState = IntakeStates.idle));

        outtake.whileTrue(new IntakeVoltageOverrideCommand(s_Intake, () -> -10));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}