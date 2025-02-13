// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.ElevatorStates;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroElevatorCommand extends Command {
  /** Creates a new ElevatorStateCommand. */
  Elevator s_Elevator;
  public ZeroElevatorCommand(Elevator s_Elevator) {
    this.s_Elevator = s_Elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Zeroing elevator");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Elevator.setVoltage(-0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Elevator.setVoltage(0);
    s_Elevator.setPosition(0);
    States.elevatorState = ElevatorStates.idle;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
