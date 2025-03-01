// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.subsystems.AlgaeArm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeArmStateCommand extends Command {
  /** Creates a new AlgaeArmStateCommand. */
  AlgaeArm s_AlgaeArm;
  public AlgaeArmStateCommand(AlgaeArm s_AlgaeArm) {
    this.s_AlgaeArm = s_AlgaeArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_AlgaeArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(States.algaeArmState) {

      case idle:
        s_AlgaeArm.setVoltage(0);
      break;
    
      case zero:
        s_AlgaeArm.runToSetpoint(0);
      break;

      case processor:
        s_AlgaeArm.runToSetpoint(-45);
      break;

      case pickup:
        s_AlgaeArm.runToSetpoint(-90);
      break;
    
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
