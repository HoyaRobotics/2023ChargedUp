// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */
  TalonSRX bottomBelt = new TalonSRX(Constants.StorageConstants.BOTTOM_STORAGE_CONVEYOR);
  CANSparkMax leftBelt = new CANSparkMax(Constants.StorageConstants.LEFT_STORAGE_CONVEYOR, MotorType.kBrushless);
  CANSparkMax rightBelt = new CANSparkMax(Constants.StorageConstants.RIGHT_STORAGE_CONVEYOR, MotorType.kBrushless);

  public Storage() {
    bottomBelt.configFactoryDefault();
    bottomBelt.setNeutralMode(NeutralMode.Brake);
    bottomBelt.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 40, 0.5));
    bottomBelt.setInverted(false);
    bottomBelt.configVoltageCompSaturation(10);
    bottomBelt.enableVoltageCompensation(true);
    leftBelt.restoreFactoryDefaults();
    leftBelt.setIdleMode(IdleMode.kBrake);
    leftBelt.setSmartCurrentLimit(20);
    leftBelt.setInverted(false);
    leftBelt.enableVoltageCompensation(10);
    rightBelt.restoreFactoryDefaults();
    rightBelt.setIdleMode(IdleMode.kBrake);
    rightBelt.setSmartCurrentLimit(20);
    rightBelt.setInverted(true);
    rightBelt.enableVoltageCompensation(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double bottom, double left, double right) {
    bottomBelt.set(ControlMode.PercentOutput, bottom);
    leftBelt.set(left);
    rightBelt.set(right);
  }

  public void stopConveyors() {
    bottomBelt.set(ControlMode.PercentOutput, 0.0);
    leftBelt.stopMotor();
    rightBelt.stopMotor();
  }
}
