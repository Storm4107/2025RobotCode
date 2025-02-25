// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;
import frc.robot.Robot;
import frc.robot.States;

public class AlgaeArm extends SubsystemBase {
  /** Creates a new AlgaeArm. */

  private SparkMax algaearm = new SparkMax(Constants.superstructureConstants.algaeArmID, MotorType.kBrushless);
  private PIDController algaearmController = new PIDController(
    Constants.superstructureConstants.algaeArmkP,
   Constants.superstructureConstants.algaeArmkI,
    Constants.superstructureConstants.algaeArmkD);
  private RelativeEncoder algaearmEncoder = algaearm.getEncoder();
  public double algaearmSetpoint;

  public AlgaeArm() {
    algaearm.configure(Robot.hardwareConfigs.algaeArmConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    double input = algaearmController.calculate(algaearmEncoder.getPosition(), setpoint) * Constants.superstructureConstants.algaeArmkF;
    setVoltage(input);
    algaearmSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    algaearm.setVoltage(voltage);
  }

  public double getPosition() {
    return algaearmEncoder.getPosition();
  }

  public double getCurrent() {
    return algaearm.getOutputCurrent();
  }

  public double getError() {
    return (algaearmSetpoint - getPosition());
  }

  public double getVoltage() {
    return algaearm.getAppliedOutput() * 12;
  }

  public void setPosition(double inches) {
    algaearmEncoder.setPosition(inches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AlgaeArm position", getPosition());
    SmartDashboard.putNumber("AlgaeArm voltage", getVoltage());
    SmartDashboard.putNumber("AlgaeArm error", getError());
    SmartDashboard.putNumber("AlgaeArm setpoint", algaearmSetpoint);

    SmartDashboard.putBoolean("motor 1",  algaearm.getInverted());
  }
}
