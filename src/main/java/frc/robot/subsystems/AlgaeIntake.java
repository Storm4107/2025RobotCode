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

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;
import frc.robot.Robot;
import frc.robot.States;

public class AlgaeIntake extends SubsystemBase {
  /** Creates a new AlgaeIntake. */

  private SparkMax algaeIntake = new SparkMax(Constants.superstructureConstants.algaeIntakeID, MotorType.kBrushless);
  private PIDController algaeIntakeController = new PIDController(
    Constants.superstructureConstants.algaeIntakekP,
    Constants.superstructureConstants.algaeIntakekI,
    Constants.superstructureConstants.algaeIntakekD);
  private RelativeEncoder algaeIntakeEncoder = algaeIntake.getEncoder();
  public double algaeIntakeSetpoint;

  public DigitalInput algaeSensor = new DigitalInput(Constants.superstructureConstants.algaeSensor);
  private Debouncer currentDebouncer = new Debouncer(0.25);

  public AlgaeIntake() {
    algaeIntake.configure(Robot.hardwareConfigs.armConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    double input = algaeIntakeController.calculate(algaeIntakeEncoder.getPosition(), setpoint) * Constants.superstructureConstants.armkF;
    setVoltage(input);
    algaeIntakeSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    algaeIntake.setVoltage(voltage);
  }

  public double getPosition() {
    return algaeIntakeEncoder.getPosition();
  }

  public double getCurrent() {
    return algaeIntake.getOutputCurrent();
  }

  public double getError() {
    return (algaeIntakeSetpoint - getPosition());
  }

  public double getVoltage() {
    return algaeIntake.getAppliedOutput() * 12;
  }

  public void setPosition(double inches) {
    algaeIntakeEncoder.setPosition(inches);
  }

  public boolean algaeSensor() {
    return algaeSensor.get();
  }

  public boolean holdingWithCurrent() {
    if (currentDebouncer.calculate(getCurrent() > 35)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("AlgaeIntake position", getPosition());
    SmartDashboard.putNumber("AlgaeIntake voltage", getVoltage());
    SmartDashboard.putNumber("AlgaeIntake error", getError());
    SmartDashboard.putNumber("AlgaeIntake Current", getCurrent());
    SmartDashboard.putNumber("AlgaeIntake setpoint", algaeIntakeSetpoint);

    SmartDashboard.putBoolean("algaeSensor", algaeSensor()); 
    SmartDashboard.putBoolean("algae Current limit", holdingWithCurrent()); 

    SmartDashboard.putBoolean("motor 1",  algaeIntake.getInverted());
  }
}
