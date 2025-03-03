// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AlgaeIntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.AlgaeIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeStateCommand extends Command {
  /** Creates a new AlgaeIntakeStateCommand. */
  AlgaeIntake s_AlgaeIntake;
  public AlgaeIntakeStateCommand(AlgaeIntake s_AlgaeIntake) {
    this.s_AlgaeIntake = s_AlgaeIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(States.algaeIntakeState) {

      case idle:
      if (s_AlgaeIntake.holdingWithCurrent()) {
        s_AlgaeIntake.setVoltage(4);
      } 
      else{
       s_AlgaeIntake.setVoltage(0);
   }      
    //s_AlgaeIntake.setVoltage(0);
      break;
      case intakea:
        if (!s_AlgaeIntake.holdingWithCurrent()) {
            s_AlgaeIntake.setVoltage(6);
          } 
         else{
           s_AlgaeIntake.setVoltage(4); 
         States.intakeState = IntakeStates.idle;
      //  s_AlgaeIntake.setVoltage(3);      
         }
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
