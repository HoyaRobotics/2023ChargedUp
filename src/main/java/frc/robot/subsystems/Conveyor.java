// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyor. */
  CANSparkMax bottomBelt = new CANSparkMax(Constants.ConveyorConstants.BOTTOM_CONVEYOR_CONVEYOR, MotorType.kBrushless);
  CANSparkMax leftBelt = new CANSparkMax(Constants.ConveyorConstants.LEFT_CONVEYOR_CONVEYOR, MotorType.kBrushless);
  CANSparkMax rightBelt = new CANSparkMax(Constants.ConveyorConstants.RIGHT_CONVEYOR_CONVEYOR, MotorType.kBrushless);

  public Conveyor() {
    bottomBelt.restoreFactoryDefaults();
    bottomBelt.setIdleMode(IdleMode.kCoast);
    bottomBelt.setSmartCurrentLimit(30);
    bottomBelt.setInverted(false);
    bottomBelt.enableVoltageCompensation(10);
    bottomBelt.setOpenLoopRampRate(0.2);
    leftBelt.restoreFactoryDefaults();
    leftBelt.setIdleMode(IdleMode.kCoast);
    leftBelt.setSmartCurrentLimit(20);
    leftBelt.setInverted(false);
    leftBelt.enableVoltageCompensation(10);
    leftBelt.setOpenLoopRampRate(0.2);
    rightBelt.restoreFactoryDefaults();
    rightBelt.setIdleMode(IdleMode.kCoast);
    rightBelt.setSmartCurrentLimit(20);
    rightBelt.setInverted(true);
    rightBelt.enableVoltageCompensation(10);
    rightBelt.setOpenLoopRampRate(0.2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double bottom, double left, double right) {
    bottomBelt.set(bottom);
    leftBelt.set(left);
    rightBelt.set(right);
  }

  public void stopConveyors() {
    bottomBelt.stopMotor();
    leftBelt.stopMotor();
    rightBelt.stopMotor();
  }
}
