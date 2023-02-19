// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  private CANSparkMax leftArmMotor = new CANSparkMax(Constants.ArmConstants.LEFT_ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController leftArmPID;

  private CANSparkMax rightArmMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController rightArmPID;

  private CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmConstants.EXTENSION_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController extensionPID;

  private DutyCycleEncoder armEncoder = new DutyCycleEncoder(0);

  private double gearRatio = (10/50) * (14/68) * (22/72);
  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setSmartCurrentLimit(30);
    leftArmMotor.setInverted(false);
    leftArmMotor.enableVoltageCompensation(10);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    leftArmPID = leftArmMotor.getPIDController();
    leftArmPID.setP(0.0);
    leftArmPID.setI(0.0);
    leftArmPID.setD(0.0);
    leftArmPID.setIZone(0.0);
    leftArmPID.setFF(0.0);
    leftArmPID.setOutputRange(-1, 1);
    leftArmPID.setSmartMotionMaxVelocity(0.0, 0);
    leftArmPID.setSmartMotionMaxAccel(0.0, 0);
    leftArmPID.setSmartMotionAllowedClosedLoopError(0.0, 0);

    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setSmartCurrentLimit(30);
    rightArmMotor.setInverted(true);
    rightArmMotor.enableVoltageCompensation(10);
    rightArmPID = rightArmMotor.getPIDController();
    rightArmPID.setP(0.0);
    rightArmPID.setI(0.0);
    rightArmPID.setD(0.0);
    rightArmPID.setIZone(0.0);
    rightArmPID.setFF(0.0);
    rightArmPID.setOutputRange(-1, 1);
    rightArmPID.setSmartMotionMaxVelocity(0.0, 0);
    rightArmPID.setSmartMotionMaxAccel(0.0, 0);
    rightArmPID.setSmartMotionAllowedClosedLoopError(0.0, 0);
    
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setSmartCurrentLimit(20);
    extensionMotor.setInverted(false);
    extensionMotor.enableVoltageCompensation(10);
    extensionPID = extensionMotor.getPIDController();
    extensionPID.setP(0.0);
    extensionPID.setI(0.0);
    extensionPID.setD(0.0);
    extensionPID.setIZone(0.0);
    extensionPID.setFF(0.0);
    extensionPID.setOutputRange(-1, 1);
    extensionPID.setSmartMotionMaxVelocity(0.0, 0);
    extensionPID.setSmartMotionMaxAccel(0.0, 0);
    extensionPID.setSmartMotionAllowedClosedLoopError(0.0, 0);

    setArmEncoder();
    setExtensionEncoder(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setArmAngleSmartMotion(double angle) {
    leftArmPID.setReference(angle, CANSparkMax.ControlType.kSmartMotion, 0);
    rightArmPID.setReference(angle, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public void setArmAnglePID(double angle) {
    leftArmPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0);
    rightArmPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0);
  }

  public void setArmEncoder() {
    double encoderPosition = armEncoder.getAbsolutePosition();
    double motorPosition = encoderPosition/gearRatio;
    leftArmMotor.getEncoder().setPosition(motorPosition);
    rightArmMotor.getEncoder().setPosition(motorPosition);
  }

  public double getLeftArmAngle() {
    return leftArmMotor.getEncoder().getPosition();
  }

  public double getRightArmAngle() {
    return rightArmMotor.getEncoder().getPosition();
  }

  public double getArmEncoderAngle() {
    return armEncoder.getAbsolutePosition();
  }

  public void setExtensionSmartMotion(double extension) {
    extensionPID.setReference(extension, CANSparkMax.ControlType.kSmartMotion, 0);
  }

  public void setExtensionPID(double extension) {
    extensionPID.setReference(extension, CANSparkMax.ControlType.kPosition, 0);
  }

  public void setExtensionEncoder(double extension) {
    extensionMotor.getEncoder().setPosition(extension);
  }

  public double getExtensionPosition() {
    return extensionMotor.getEncoder().getPosition();
  }
}
