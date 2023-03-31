// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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

  Mechanism2d mechanism1 = new Mechanism2d(Units.inchesToMeters(2), Units.inchesToMeters(49.625000));
  MechanismRoot2d root;
  MechanismLigament2d wrist1;
  MechanismLigament2d wrist2;

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
    leftArmPID.setOutputRange(-0.75, 0.75); //0.7

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
    rightArmPID.setOutputRange(-0.75, 0.75);
    
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
    setArmAnglePID(Constants.pickupConeArmPosition);
    setExtensionPID(Constants.pickupConeExtensionPosition); //was 26

    root = mechanism1.getRoot("rotation", 0, Units.inchesToMeters(-49.625000));

    double armGearRatio = (10.0/50.0) * (14.0/68.0) * (22.0/72.0) * (360);
    double armAngle = getLeftArmAngle() * armGearRatio;

    double extensionGearRatio = (1.0/25.0) * 1.273;
    double extensionLength = (getExtensionPosition() * extensionGearRatio * -1.0) + 120.0;

    wrist1 = root.append(new MechanismLigament2d("Stinger Rotate 1", extensionLength, armAngle));
    wrist2 = root.append(new MechanismLigament2d("Stinger Rotate 2", 120.0 - extensionLength, 180.0 + armAngle));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ArmAngle", getArmAngle());
    SmartDashboard.putNumber("ArmExtension", getExtensionPosition());

    Logger.getInstance().recordOutput("Arm Angle", getLeftArmAngle());
    Logger.getInstance().recordOutput("Arm Extension", getExtensionPosition());

    if(getLeftArmAngle() < -4.5 && GlobalVariables.maxSpeed != 0.45) {
      GlobalVariables.maxSpeed = 0.45;
    }else if( getLeftArmAngle() >= -4.5 && GlobalVariables.maxSpeed != Constants.DRIVE_SPEED){
      GlobalVariables.maxSpeed = Constants.DRIVE_SPEED;
    }

    double armGearRatio = (10.0/50.0) * (14.0/68.0) * (22.0/72.0) * (360);
    double armAngle = getLeftArmAngle() * armGearRatio;

    double extensionGearRatio = (1.0/25.0) * 1.273;
    double extensionLength = (getExtensionPosition() * extensionGearRatio * -1.0) + 120.0;

    wrist1.setAngle(armAngle);
    wrist2.setAngle(armAngle + 180.0);
    wrist1.setLength(extensionLength);
    wrist2.setLength(120.0 - extensionLength);

    SmartDashboard.putData("Stinger", mechanism1);
    Logger.getInstance().recordOutput("Stinger", mechanism1);
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

  public boolean isExtensionPastPositon(double position, boolean greaterThen) {
    if(greaterThen && position <= extensionMotor.getEncoder().getPosition()) {
      return true;
    }else if(!greaterThen && position >= extensionMotor.getEncoder().getPosition()) {
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

  public boolean isArmPastPosition(double position, boolean greaterThen) {
    if(greaterThen && position <= leftArmMotor.getEncoder().getPosition()){
      return true;
    }else if(!greaterThen && position >= leftArmMotor.getEncoder().getPosition()) {
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

  public void setBrakeMode() {
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setCoastMode() {
    leftArmMotor.setIdleMode(IdleMode.kCoast);
    rightArmMotor.setIdleMode(IdleMode.kCoast);
    extensionMotor.setIdleMode(IdleMode.kCoast);
  }
}
