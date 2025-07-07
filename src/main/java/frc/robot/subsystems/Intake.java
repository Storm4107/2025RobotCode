// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.HardwareConfigs;
import frc.robot.Robot;
import frc.robot.States;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private SparkMax intake1 =new SparkMax(Constants.superstructureConstants.intake1ID, MotorType.kBrushless);
  private SparkMax intake2 =new SparkMax(Constants.superstructureConstants.intake2ID, MotorType.kBrushless);
  private PIDController intakeController = new PIDController(
    Constants.superstructureConstants.intakekP,
    Constants.superstructureConstants.intakekI,
    Constants.superstructureConstants.intakekD);
  private RelativeEncoder intakeEncoder1 = intake1.getEncoder();
  public double intakeSetpoint;
  private Debouncer currentDebouncer = new Debouncer(0.25);

  public DigitalInput coralSensor = new DigitalInput(Constants.superstructureConstants.coralSensor);
public Command intake;


  public Intake() {
    intake1.configure(Robot.hardwareConfigs.intakeConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake2.configure(Robot.hardwareConfigs.intake2Config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setPosition(0);
  }

  public void runToSetpoint(double setpoint) {
    double input = intakeController.calculate(intakeEncoder1.getPosition(), setpoint) * Constants.superstructureConstants.intakekF;
    setVoltage(input);
    intakeSetpoint = setpoint;
  }

  public void setVoltage(double voltage) {
    intake1.setVoltage(voltage);
  }

  public double getPosition() {
    return intakeEncoder1.getPosition();
  }

  public double getCurrent() {
    return intake1.getOutputCurrent();
  }

  public double getError() {
    return (intakeSetpoint - getPosition());
  }

  public double getVoltage() {
    return intake1.getAppliedOutput() * 12;
  }

  public void setPosition(double inches) {
    intakeEncoder1.setPosition(inches);
  }

  public boolean coralSensor() {
    return coralSensor.get();
  }

  public boolean holdingWithCurrent() {
    if (currentDebouncer.calculate(getCurrent() >25)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intake position", getPosition());
    SmartDashboard.putNumber("intake voltage", getVoltage());
    SmartDashboard.putNumber("intake error", getError());
    SmartDashboard.putNumber("intae Current", getCurrent());
    SmartDashboard.putNumber("intake setpoint", intakeSetpoint);
    SmartDashboard.putBoolean("Intake current limit", holdingWithCurrent()); 

    SmartDashboard.putBoolean("coralSensor", coralSensor()); 

    SmartDashboard.putBoolean("motor 1",  intake1.getInverted());
    SmartDashboard.putBoolean("motor 2",  intake2.getInverted());
  }
}
