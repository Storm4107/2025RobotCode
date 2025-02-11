// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorStateCommand extends Command {
  /** Creates a new ElevatorStateCommand. */
  Elevator s_Elevator;
  public ElevatorStateCommand(Elevator s_Elevator) {
    this.s_Elevator = s_Elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(States.elevatorState) {

      case idle:
        s_Elevator.setVoltage(0);
      break;
    
      case zero:
        s_Elevator.runToSetpoint(0);
      break;

      case l1:
        s_Elevator.runToSetpoint(10);
      break;

      case l2:
        s_Elevator.runToSetpoint(15);
      break;

      case l3:
        s_Elevator.runToSetpoint(25);
      break;

      case l4:
        s_Elevator.runToSetpoint(30);
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
