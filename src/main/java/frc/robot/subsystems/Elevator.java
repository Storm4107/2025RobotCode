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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;
import frc.robot.Robot;
import frc.robot.States;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private SparkMax elevator1 =new SparkMax(Constants.superstructureConstants.elevator1ID, MotorType.kBrushless);
  private SparkMax elevator2 =new SparkMax(Constants.superstructureConstants.elevator2ID, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController1 = elevator1.getClosedLoopController();
  private SparkClosedLoopController elevatorController2 = elevator2.getClosedLoopController();
  private RelativeEncoder elevatorEncoder1 = elevator1.getEncoder();
  private RelativeEncoder elevatorEncoder2 = elevator2.getEncoder();
  public double elevatorSetpoint;

  public Elevator() {
    elevator1.configure(Robot.hardwareConfigs.elevatorConfig, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    elevator2.configure(Robot.hardwareConfigs.elevator2Config, com.revrobotics.spark.SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    elevatorController1.setReference(setpoint, ControlType.kPosition);
    elevatorController2.setReference(setpoint, ControlType.kPosition);
    elevatorSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    elevator1.setVoltage(voltage);
    elevator2.setVoltage(voltage);
  }

  public double getPosition() {
    return elevatorEncoder1.getPosition();
  }

  public double getCurrent() {
    return elevator1.getOutputCurrent();
  }

  public double getError() {
    return (elevatorSetpoint - getPosition());
  }

  public double getVoltage() {
    return elevator1.getAppliedOutput() * 12;
  }

  public void setPosition(double inches) {
    elevatorEncoder1.setPosition(inches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator position", getPosition());
    SmartDashboard.putNumber("Elevator voltage", getVoltage());
    SmartDashboard.putNumber("Elevator error", getError());
    SmartDashboard.putNumber("Elevator setpoint", elevatorSetpoint);

    SmartDashboard.putBoolean("motor 1",  elevator1.getInverted());
    SmartDashboard.putBoolean("motor 2",  elevator2.getInverted());
  }
}
