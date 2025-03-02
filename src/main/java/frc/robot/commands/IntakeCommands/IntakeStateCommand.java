// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.States;
import frc.robot.States.IntakeStates;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeStateCommand extends Command {
  /** Creates a new IntakeStateCommand. */
  Intake s_Intake;
  public IntakeStateCommand(Intake s_Intake) {
    this.s_Intake = s_Intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(States.intakeState) {

      case idle:

        if (s_Intake.holdingWithCurrent()) {
         s_Intake.setVoltage(2);
       } 
       else{
        s_Intake.setVoltage(0);
    }      
      break;
      
      case intakec:
        if (s_Intake.coralSensor()) {
          s_Intake.setVoltage(8);
        } 
        else{
         s_Intake.setVoltage(0);
        States.intakeState = IntakeStates.idle; 
      }      
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
