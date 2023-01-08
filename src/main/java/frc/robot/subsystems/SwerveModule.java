// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.SwerveModuleConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {  
  public int moduleNumber;
  private double angleOffset;
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward;

  //Creates the Swerve modules Motors and Encoders
  //Relies on the moduleConstants class found in SwerveModuleConstants.java
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Configuration */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, "carnivore");
    configAngleEncoder();

    /* Angle Motor Configuration */
    angleMotor = new TalonFX(moduleConstants.angleMotorID, "carnivore");
    configAngleMotor();

    /* Drive Motor Configuration */
    driveMotor = new TalonFX(moduleConstants.driveMotorID, "carnivore");
    configDriveMotor();

    lastAngle = getState().angle.getDegrees();
  }


  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

  }

  public void setDesiredStateAbs() {

  }

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    angleMotor.set(ControlMode.PercentOutput, 0);
  }

  private void resetToAbsolute() {

  }

  private void configAngleEncoder() {

  }

  private void configAngleMotor() {

  }

  private void configDriveMotor() {

  }

  public Rotation2d getCanCoder() {

    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {

    return new SwerveModuleState();
  }

  /*public SwerveModulePosition getPosition() {

    return new SwerveModulePosition(distance, angle);
  }*/

}