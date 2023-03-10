// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class Arm extends SubsystemBase {
  private CANSparkMax leftArmMotor = new CANSparkMax(Constants.ArmConstants.LEFT_ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController leftArmPID;

  private CANSparkMax rightArmMotor = new CANSparkMax(Constants.ArmConstants.RIGHT_ARM_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController rightArmPID;

  private CANSparkMax extensionMotor = new CANSparkMax(Constants.ArmConstants.EXTENSION_MOTOR, MotorType.kBrushless);
  private SparkMaxPIDController extensionPID;

  /** Creates a new Arm. */
  public Arm() {
    leftArmMotor.restoreFactoryDefaults();
    leftArmMotor.setIdleMode(IdleMode.kCoast);
    leftArmMotor.setSmartCurrentLimit(30);
    leftArmMotor.setInverted(false);
    leftArmMotor.enableVoltageCompensation(10);
    leftArmMotor.setOpenLoopRampRate(0.3);
    leftArmMotor.setClosedLoopRampRate(0.3);
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kForward, 2);
    leftArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    leftArmMotor.setSoftLimit(SoftLimitDirection.kReverse, -30);
    leftArmPID = leftArmMotor.getPIDController();
    leftArmPID.setP(0.05);
    leftArmPID.setI(0.0);
    leftArmPID.setD(0.0);
    leftArmPID.setIZone(0.0);
    leftArmPID.setFF(0.0);
    leftArmPID.setOutputRange(-0.7, 0.7);

    rightArmMotor.restoreFactoryDefaults();
    rightArmMotor.setIdleMode(IdleMode.kCoast);
    rightArmMotor.setSmartCurrentLimit(30);
    rightArmMotor.setInverted(true);
    rightArmMotor.enableVoltageCompensation(10);
    rightArmMotor.setOpenLoopRampRate(0.3);
    rightArmMotor.setClosedLoopRampRate(0.3);
    rightArmMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    rightArmMotor.setSoftLimit(SoftLimitDirection.kForward, 2);
    rightArmMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    rightArmMotor.setSoftLimit(SoftLimitDirection.kReverse, -30);
    rightArmPID = rightArmMotor.getPIDController();
    rightArmPID.setP(0.05);
    rightArmPID.setI(0.0);
    rightArmPID.setD(0.0);
    rightArmPID.setIZone(0.0);
    rightArmPID.setFF(0.0);
    rightArmPID.setOutputRange(-0.7, 0.7);
    
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.setIdleMode(IdleMode.kCoast);
    extensionMotor.setSmartCurrentLimit(30);
    extensionMotor.setInverted(false);
    extensionMotor.enableVoltageCompensation(10);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    extensionMotor.setSoftLimit(SoftLimitDirection.kReverse, 3.0f);
    extensionMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    extensionMotor.setSoftLimit(SoftLimitDirection.kForward, 110.0f);
    extensionMotor.setOpenLoopRampRate(0.2);
    extensionMotor.setClosedLoopRampRate(0.2);
    extensionPID = extensionMotor.getPIDController();
    extensionPID.setP(0.07);
    extensionPID.setI(0.0);
    extensionPID.setD(0.0);
    extensionPID.setIZone(0.0);
    extensionPID.setFF(0.0);
    extensionPID.setOutputRange(-1, 1);

    setArmEncoder();
    setExtensionEncoder(0.0);
    setArmAnglePID(Constants.pickupArmPosition);
    setExtensionPID(Constants.pickupExtensionPosition); //was 26
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", getArmAngle());
    SmartDashboard.putNumber("ArmExtension", getExtensionPosition());
  }

  public void setArmAnglePID(double angle) {
    leftArmPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0);
    rightArmPID.setReference(angle, CANSparkMax.ControlType.kPosition, 0);
  }

  public void setArmEncoder() {
    leftArmMotor.getEncoder().setPosition(0.0);
    rightArmMotor.getEncoder().setPosition(0.0);
  }

  public double getLeftArmAngle() {
    return leftArmMotor.getEncoder().getPosition();
  }

  public double getRightArmAngle() {
    return rightArmMotor.getEncoder().getPosition();
  }

  public double getArmAngle() {
    return leftArmMotor.getEncoder().getPosition();
  }

  public void setExtensionPID(double extension) {
    extensionPID.setReference(extension, CANSparkMax.ControlType.kPosition, 0);
  }

  public boolean isExtensionInPosition(double position) {
    if(position <= extensionMotor.getEncoder().getPosition()+1 && position >= extensionMotor.getEncoder().getPosition()-1) {
      return true;
    }else{
      return false;
    }
  }

  public boolean isArmInPosition(double position) {
    if(position <= leftArmMotor.getEncoder().getPosition()+1 && position >= leftArmMotor.getEncoder().getPosition()-1) {
      return true;
    }else{
      return false;
    }
  }

  public void setExtensionEncoder(double extension) {
    extensionMotor.getEncoder().setPosition(extension);
  }

  public double getExtensionPosition() {
    return extensionMotor.getEncoder().getPosition();
  }

  public void setArmSpeed(double speed) {
    rightArmMotor.set(speed);
    leftArmMotor.set(speed);
  }

  public void stopArm() {
    rightArmMotor.stopMotor();
    leftArmMotor.stopMotor();
  }

  public void setExtensionSpeed(double speed) {
    extensionMotor.set(speed);
  }

  public void stopExtension() {
    extensionMotor.stopMotor();
  }

  public void setExtensionPIDValue(double pGain) {
    extensionPID.setP(pGain);
  }

  public void moveGridTarget(boolean goUp){
    if(goUp){
      GlobalVariables.upDownPosition = (GlobalVariables.upDownPosition++)%3;
    }else{
      GlobalVariables.upDownPosition = (GlobalVariables.upDownPosition--)%3;

    }

    //prevent error from being out of range
    if(GlobalVariables.upDownPosition < 0){
      GlobalVariables.upDownPosition = 2;
    }else if(GlobalVariables.upDownPosition > 2){
      GlobalVariables.upDownPosition = 0;
    }
    SmartDashboard.putNumber("Up Down Peg", (GlobalVariables.upDownPosition));
  }

  public void moveGridTargetIf(boolean goUp){
    if(goUp && GlobalVariables.upDownPosition < 2){
      GlobalVariables.upDownPosition++;
    }else if(goUp && GlobalVariables.upDownPosition == 2){
      GlobalVariables.upDownPosition = 0;
    }else if(!goUp && GlobalVariables.upDownPosition > 0){
      GlobalVariables.upDownPosition--;
    }else if(!goUp && GlobalVariables.upDownPosition == 0){
      GlobalVariables.upDownPosition = 2;
    }
  }
}
