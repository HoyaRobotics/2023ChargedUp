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
import edu.wpi.first.wpilibj.Timer;
import frc.lib.CTREModuleState;
import frc.lib.Conversions;
import frc.lib.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {  
  public int moduleNumber;
  private double angleOffset;
  private TalonFX angleMotor;
  private TalonFX driveMotor;
  private CANCoder angleEncoder;
  private double lastAngle;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.driveKS, ModuleConstants.driveKV, ModuleConstants.driveKA);

  //Creates the Swerve modules Motors and Encoders
  //Relies on the moduleConstants class found in SwerveModuleConstants.java
  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Configuration */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, "canivore");
    configAngleEncoder();

    /* Angle Motor Configuration */
    angleMotor = new TalonFX(moduleConstants.angleMotorID, "canivore");
    configAngleMotor();

    /* Drive Motor Configuration */
    driveMotor = new TalonFX(moduleConstants.driveMotorID, "canivore");
    configDriveMotor();

    lastAngle = getPosition().angle.getDegrees();
    stop();
  }


  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState = CTREModuleState.optimize(desiredState, getState().angle); //Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.01)) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
    angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.ModuleConstants.angleGearRatio)); 
    lastAngle = angle;
}

public void setDesiredStateAbs(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    
    if(isOpenLoop){
        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    else {
        double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
        driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
    }

    angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(desiredState.angle.getDegrees(), Constants.ModuleConstants.angleGearRatio)); 
    lastAngle = desiredState.angle.getDegrees();
}

  public void stop() {
    driveMotor.set(ControlMode.PercentOutput, 0);
    angleMotor.set(ControlMode.PercentOutput, 0);
  }


  private void resetToAbsolute() {
    double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.ModuleConstants.angleGearRatio);
    angleMotor.setSelectedSensorPosition(absolutePosition);
  }


  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    angleEncoder.configSensorDirection(Constants.ModuleConstants.canCoderInvert);
    angleEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
  }


  private void configAngleMotor() {
    angleMotor.configFactoryDefault();
    angleMotor.config_kP(0, Constants.ModuleConstants.angleKP);
    angleMotor.config_kI(0, Constants.ModuleConstants.angleKI);
    angleMotor.config_kD(0, Constants.ModuleConstants.angleKD);
    angleMotor.config_kF(0, Constants.ModuleConstants.angleKF);
    angleMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(Constants.ModuleConstants.angleEnableCurrentLimit, Constants.ModuleConstants.angleContinuousCurrentLimit, Constants.ModuleConstants.anglePeakCurrentLimit, Constants.ModuleConstants.anglePeakCurrentDuration));
    angleMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    angleMotor.configNeutralDeadband(Constants.ModuleConstants.angleNeutralDeadband);
    //angleMotor.configOpenloopRamp(0.01);
    //angleMotor.configClosedloopRamp(0.01);

    angleMotor.setInverted(Constants.ModuleConstants.angleMotorInvert);
    angleMotor.setNeutralMode(Constants.ModuleConstants.angleNeutralMode);
    Timer.delay(1.0);
    resetToAbsolute();
  }


  private void configDriveMotor() {
    driveMotor.configFactoryDefault();
    driveMotor.config_kP(0, Constants.ModuleConstants.driveKP);
    driveMotor.config_kI(0, Constants.ModuleConstants.driveKI);
    driveMotor.config_kD(0, Constants.ModuleConstants.driveKD);
    driveMotor.config_kF(0, Constants.ModuleConstants.driveKF);
    driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(Constants.ModuleConstants.driveEnableCurrentLimit, Constants.ModuleConstants.driveContinuousCurrentLimit, Constants.ModuleConstants.drivePeakCurrentLimit, Constants.ModuleConstants.drivePeakCurrentDuration));
    driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    driveMotor.configOpenloopRamp(Constants.ModuleConstants.openLoopRamp);
    driveMotor.configClosedloopRamp(Constants.ModuleConstants.closedLoopRamp);

    driveMotor.setInverted(Constants.ModuleConstants.driveMotorInvert);
    driveMotor.setNeutralMode(Constants.ModuleConstants.driveNeutralMode);
    driveMotor.setSelectedSensorPosition(0);
  }


  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  
  //Should not use anymore use getPosition() instead
  public SwerveModuleState getState() {
    double velocity = Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.ModuleConstants.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
  }


  public SwerveModulePosition getPosition() {
    double distance = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), Constants.ModuleConstants.wheelCircumference, Constants.ModuleConstants.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.ModuleConstants.angleGearRatio));
    return new SwerveModulePosition(distance, angle);
  }

}