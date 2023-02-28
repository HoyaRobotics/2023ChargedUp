// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class Storage extends SubsystemBase {
  /** Creates a new Storage. */
  VictorSPX bottomBelt = new VictorSPX(Constants.StorageConstants.BOTTOM_STORAGE_CONVEYOR);
  CANSparkMax leftBelt = new CANSparkMax(Constants.StorageConstants.LEFT_STORAGE_CONVEYOR, MotorType.kBrushless);
  CANSparkMax rightBelt = new CANSparkMax(Constants.StorageConstants.RIGHT_STORAGE_CONVEYOR, MotorType.kBrushless);

  public Storage() {
    bottomBelt.configFactoryDefault();
    bottomBelt.setNeutralMode(NeutralMode.Coast);
    bottomBelt.setInverted(false);
    bottomBelt.configVoltageCompSaturation(10);
    bottomBelt.enableVoltageCompensation(true);
    bottomBelt.configOpenloopRamp(0.1);
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
    //bottomBelt.set(ControlMode.PercentOutput, bottom);
    bottomBelt.set(VictorSPXControlMode.PercentOutput, bottom);
    if(GlobalVariables.INTAKE_LOWERED == true) {
      leftBelt.set(left);
      rightBelt.set(right);
    }
  }

  public void stopConveyors() {
    bottomBelt.set(ControlMode.PercentOutput, 0.0);
    leftBelt.stopMotor();
    rightBelt.stopMotor();
  }
}
