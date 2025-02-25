// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.AlgaeArmStates;
import frc.robot.subsystems.AlgaeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmVoltageOverrideCommand extends Command {
  /** Creates a new AlgaeArmStateCommand. */
  AlgaeArm s_AlgaeArm;
  DoubleSupplier Voltage;
  public AlgaeArmVoltageOverrideCommand(AlgaeArm s_AlgaeArm, DoubleSupplier Voltage) {
    this.s_AlgaeArm = s_AlgaeArm;
    this.Voltage = Voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_AlgaeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Voltage override");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_AlgaeArm.setVoltage(Voltage.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_AlgaeArm.setVoltage(0);
    States.algaeArmState = AlgaeArmStates.idle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
