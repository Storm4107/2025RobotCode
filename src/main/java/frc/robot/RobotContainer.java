package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.States.AlgaeArmStates;
import frc.robot.States.ElevatorStates;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorCommands.ElevatorStateCommand;
import frc.robot.commands.ElevatorCommands.ElevatorVoltageOverrideCommand;
import frc.robot.commands.ElevatorCommands.ZeroElevatorCommand;
import frc.robot.commands.ArmCommands.ArmStateCommand;
import frc.robot.commands.ArmCommands.ArmVoltageOverrideCommand;
import frc.robot.commands.ArmCommands.ZeroArmCommand;
import frc.robot.commands.IntakeCommands.IntakeStateCommand;
import frc.robot.commands.IntakeCommands.IntakeVoltageOverrideCommand;
import frc.robot.commands.AlgaeIntakeCommands.*;
import frc.robot.commands.AlgaeArmCommands.*;
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

    private final JoystickButton dampen = new JoystickButton(driver, 18);

    private final JoystickButton zeroElevator = new JoystickButton(driver, 8);
    private final JoystickButton l1 = new JoystickButton(driver, 7);
    private final JoystickButton l2 = new JoystickButton(driver, 6);
    private final JoystickButton l3 = new JoystickButton(driver, 5);
    private final JoystickButton l4 = new JoystickButton(driver, 4);

    private final JoystickButton ResetElevator = new JoystickButton(driver, 1);
    private final JoystickButton elevatorUp = new JoystickButton(driver, 3);
    private final JoystickButton elevatorDown = new JoystickButton(driver, 4);


    private final JoystickButton armUp = new JoystickButton(operator, 11);
    private final JoystickButton armDown = new JoystickButton(operator, 7);

    private final JoystickButton intake = new JoystickButton(operator, 1);
    private final JoystickButton outtake = new JoystickButton(operator, 2);

    private final JoystickButton algaeIntake = new JoystickButton(operator, 3);
    private final JoystickButton algaeOuttake = new JoystickButton(operator, 4);

    private final JoystickButton algaeUp = new JoystickButton(operator, 5);
    private final JoystickButton algaeDown = new JoystickButton(operator, 6);

    private final JoystickButton zero = new JoystickButton(operator, 9);
    private final JoystickButton processor = new JoystickButton(operator, 8);
    private final JoystickButton pickup = new JoystickButton(operator, 10);



    //private final JoystickButton DynamicLock = new JoystickButton(driver, XboxController.Button.kX.value);

    //private final Trigger forwardHold = new Trigger(() -> (driver.getRawAxis(4) > 0.75));
    //private final Trigger backwardHold = new Trigger(() -> (driver.getRawAxis(4) < -0.75));

    /* Subsystems */
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();
    private final Swerve s_Swerve = new Swerve(s_PoseEstimator);
    private final Elevator s_Elevator = new Elevator();
    private final Arm s_Arm = new Arm();
    private final Intake s_Intake = new Intake();
    private final AlgaeIntake s_AlgaeIntake = new AlgaeIntake();
    private final AlgaeArm s_AlgaeArm = new AlgaeArm();

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

        s_AlgaeIntake.setDefaultCommand(
            new AlgaeIntakeStateCommand(s_AlgaeIntake)
        );

        s_AlgaeArm.setDefaultCommand(
            new AlgaeArmStateCommand(s_AlgaeArm)
        );

        // Configure the button bindings
        configureButtonBindings();


        //Pathplanner commands - templates
        NamedCommands.registerCommand("marker1", Commands.print("Passed marker 1"));
        NamedCommands.registerCommand("marker2", Commands.print("Passed marker 2"));
        NamedCommands.registerCommand("print hello", Commands.print("hello"));
    
        
        //Auto chooser
        autoChooser = AutoBuilder.buildAutoChooser("New Auto"); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        zeroElevator.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.zero));
        l1.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l1));
        l2.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l2));
        l3.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l3));
        l4.onTrue(new InstantCommand(() -> States.elevatorState = ElevatorStates.l4));

        ResetElevator.whileTrue(new ZeroElevatorCommand(s_Elevator));

        elevatorUp.whileTrue(new ElevatorVoltageOverrideCommand(s_Elevator, () -> 1));
        elevatorDown.whileTrue(new ElevatorVoltageOverrideCommand(s_Elevator, () -> -1));

        armUp.whileTrue(new ArmVoltageOverrideCommand(s_Arm, () -> 8));
        armDown.whileTrue(new ArmVoltageOverrideCommand(s_Arm, () -> -8));

        intake.whileTrue(new IntakeVoltageOverrideCommand(s_Intake, () -> 6));
        outtake.whileTrue(new IntakeVoltageOverrideCommand(s_Intake, () -> -6));

        algaeIntake.whileTrue(new AlgaeIntakeVoltageOverrideCommand(s_AlgaeIntake, () -> 3));
        algaeOuttake.whileTrue(new AlgaeIntakeVoltageOverrideCommand(s_AlgaeIntake, () -> -3));

        algaeUp.whileTrue(new AlgaeArmVoltageOverrideCommand(s_AlgaeArm, () -> 1));
        algaeDown.whileTrue(new AlgaeArmVoltageOverrideCommand(s_AlgaeArm, () -> -1));

        processor.onTrue(new InstantCommand(() -> States.algaeArmState = AlgaeArmStates.processor));
        pickup.onTrue(new InstantCommand(() -> States.algaeArmState = AlgaeArmStates.pickup));
        zero.onTrue(new InstantCommand(() -> States.algaeArmState = AlgaeArmStates.zero));

        
        
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