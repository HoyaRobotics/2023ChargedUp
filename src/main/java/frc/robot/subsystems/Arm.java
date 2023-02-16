// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  CANSparkMax leftArmMotor = new CANSparkMax(Constants.ArmConstants.LEFT_ARM_MOTOR, MotorType.kBrushless);
  CANSparkMax rightArmMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
  CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmConstants.EXTENSION_MOTOR, MotorType.kBrushless);
  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setSmartCurrentLimit(30);
    leftArmMotor.setInverted(false);
    leftArmMotor.enableVoltageCompensation(10);
    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setSmartCurrentLimit(30);
    rightArmMotor.setInverted(false);
    rightArmMotor.enableVoltageCompensation(10);
    rightArmMotor.follow(leftArmMotor);
    
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setSmartCurrentLimit(20);
    extensionMotor.setInverted(false);
    extensionMotor.enableVoltageCompensation(10);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmAngle(double angle) {}
}
