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

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private SparkMax arm = new SparkMax(Constants.superstructureConstants.armID, MotorType.kBrushless);
  private PIDController armController = new PIDController(
    Constants.superstructureConstants.armkP,
   Constants.superstructureConstants.armkI,
    Constants.superstructureConstants.armkD);
  private RelativeEncoder armEncoder = arm.getEncoder();
  public double armSetpoint;

  public Arm() {
    arm.configure(Robot.hardwareConfigs.armConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    double input = armController.calculate(armEncoder.getPosition(), setpoint) * Constants.superstructureConstants.armkF;
    setVoltage(input);
    armSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public double getPosition() {
    return armEncoder.getPosition();
  }

  public double getCurrent() {
    return arm.getOutputCurrent();
  }

  public double getError() {
    return (armSetpoint - getPosition());
  }

  public double getVoltage() {
    return arm.getAppliedOutput() * 12;
  }

  public void setPosition(double inches) {
    armEncoder.setPosition(inches);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm position", getPosition());
    SmartDashboard.putNumber("Arm voltage", getVoltage());
    SmartDashboard.putNumber("Arm error", getError());
    SmartDashboard.putNumber("Arm setpoint", armSetpoint);

    SmartDashboard.putBoolean("motor 1",  arm.getInverted());
  }
}
