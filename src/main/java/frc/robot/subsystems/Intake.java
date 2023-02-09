// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax frontRoller;
  CANSparkMax backRoller;

  TalonFX retractor;

  /** Creates a new Intake. */
  public Intake() {
    this.frontRoller = new CANSparkMax(Constants.SwerveConstants.FRONT_INTAKE_ROLLER, MotorType.kBrushless);
    this.backRoller = new CANSparkMax(Constants.SwerveConstants.BACK_INTAKE_ROLLER, MotorType.kBrushless);

    this.retractor = new TalonFX(Constants.SwerveConstants.INTAKE_RETRACTION, "canivore");
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeSpeed(double frontVoltage, double backVoltage) {
    frontRoller.setVoltage(frontVoltage);
    backRoller.setVoltage(backVoltage);
  }

  public void intakeRotationSpeed(double voltage) {
    //retractor.set(voltage);
  }

  public void intakeStop() {
    frontRoller.stopMotor();
    backRoller.stopMotor();
  }

  public void retractorStop() {
    //retractor.stopMotor();
  }

  public double getIntakeRetractorPosition() {
    return retractor.getSelectedSensorPosition();
  }
}
