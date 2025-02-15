// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmStateCommand extends Command {
  /** Creates a new ArmStateCommand. */
  Arm s_Arm;
  public ArmStateCommand(Arm s_Arm) {
    this.s_Arm = s_Arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(States.armState) {

      case idle:
        s_Arm.setVoltage(0);
      break;
    
      case zero:
        s_Arm.runToSetpoint(0);
      break;

      case l1:
        s_Arm.runToSetpoint(10);
      break;

      case l2:
        s_Arm.runToSetpoint(15);
      break;

      case l3:
        s_Arm.runToSetpoint(25);
      break;

      case l4:
        s_Arm.runToSetpoint(30);
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
