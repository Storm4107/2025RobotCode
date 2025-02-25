// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.AlgaeIntakeStates;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeVoltageOverrideCommand extends Command {
  /** Creates a new AlgaeIntakeStateCommand. */
  AlgaeIntake s_AlgaeIntake;
  DoubleSupplier Voltage;
  public AlgaeIntakeVoltageOverrideCommand(AlgaeIntake s_AlgaeIntake, DoubleSupplier Voltage) {
    this.s_AlgaeIntake = s_AlgaeIntake;
    this.Voltage = Voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Voltage override");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_AlgaeIntake.setVoltage(Voltage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_AlgaeIntake.setVoltage(0);
    States.algaeIntakeState = AlgaeIntakeStates.idle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
