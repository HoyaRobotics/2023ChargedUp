// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.lib.SwerveModuleConstants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {

  public SwerveModule[] swerveModules;

  private PIDController xController, yController, thetaController;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    swerveModules = new SwerveModule[] {
      new SwerveModule(0, new SwerveModuleConstants(
      SwerveConstants.FRONT_LEFT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_MOTOR, 
      SwerveConstants.FRONT_LEFT_STEER_ENCODER, 
      SwerveConstants.FRONT_LEFT_STEER_OFFSET)),

      new SwerveModule(1, new SwerveModuleConstants(
      SwerveConstants.FRONT_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_MOTOR, 
      SwerveConstants.FRONT_RIGHT_STEER_ENCODER, 
      SwerveConstants.FRONT_RIGHT_STEER_OFFSET)),

      new SwerveModule(2, new SwerveModuleConstants(
      SwerveConstants.BACK_LEFT_DRIVE_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_MOTOR, 
      SwerveConstants.BACK_LEFT_STEER_ENCODER, 
      SwerveConstants.BACK_LEFT_STEER_OFFSET)),

      new SwerveModule(3, new SwerveModuleConstants(
      SwerveConstants.BACK_RIGHT_DRIVE_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_MOTOR, 
      SwerveConstants.BACK_RIGHT_STEER_ENCODER, 
      SwerveConstants.BACK_RIGHT_STEER_OFFSET))
    };

    if(DriverStation.getAlliance() == Alliance.Blue) {
      GlobalVariables.isBlue = true;
    }else{
      GlobalVariables.isBlue = false;
    }
  }

  //periodic() runs once per command scheduler loop
  @Override
  public void periodic() {
    //SmartDashboard.putString("FLPosition", getPosition(0).toString());
    //SmartDashboard.putString("FRPosition", getPosition(1).toString());
    //SmartDashboard.putString("BLPosition", getPosition(2).toString());
    //SmartDashboard.putString("BRPosition", getPosition(3).toString());
    SmartDashboard.putNumber("Chassis Heading", getCurrentChassisHeading().getDegrees());
    SmartDashboard.putNumber("Chassis Speed", getCurrentChassisSpeeds());
    SmartDashboard.putNumber("Max Drive Speed", GlobalVariables.maxSpeed);

    //Logger.getInstance().recordOutput("Swerve Modules", getStates());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
      SwerveModuleState[] states = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
      setModuleStates(states);
    }

  public void setModuleStates(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND);
    swerveModules[0].setDesiredState(states[0], true);
    swerveModules[1].setDesiredState(states[1], true);
    swerveModules[2].setDesiredState(states[2], true); 
    swerveModules[3].setDesiredState(states[3], true); 
  }
  
  //Calls methods in SwerveModule.java to stop motor movement for each module
  public void stop(){
    swerveModules[0].stop();
    swerveModules[1].stop();
    swerveModules[2].stop();
    swerveModules[3].stop();
  }

  public void lock(){
    swerveModules[0].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
    swerveModules[1].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[2].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)), false);
    swerveModules[3].setDesiredStateAbs(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)), false);
  }

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
      swerveModules[0].getPosition(), swerveModules[1].getPosition(), swerveModules[2].getPosition(), swerveModules[3].getPosition()
    };
  }

  public SwerveModulePosition getPosition(int swerveModule) {
    return swerveModules[swerveModule].getPosition();
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      swerveModules[0].getState(), swerveModules[1].getState(), swerveModules[2].getState(), swerveModules[3].getState()
    };
  }

  protected static PathPlannerTrajectory loadPathPlannerTrajectory(String trajectoryName, double maxVel, double maxAccel) throws IOException {
    return PathPlanner.loadPath(trajectoryName, maxVel, maxAccel);
  }

  public Command createCommandForTrajectory(PathPlannerTrajectory trajectory) {
    xController = new PIDController(AutoConstants.kPXController, 0, 0);
    yController = new PIDController(AutoConstants.kPYController, 0, 0);
    thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0); //Kp value, Ki=0, Kd=0
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
      trajectory,
      () -> PoseEstimator.getCurrentPose(),
      SwerveConstants.KINEMATICS,
      xController,
      yController,
      thetaController,
      this::setModuleStates,
      this);
    return swerveControllerCommand.andThen(() -> stop());
  }

  public double getCurrentChassisSpeeds() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates());
    double linearVelocity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) + (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVelocity;
  }

  public Rotation2d getCurrentChassisHeading() {
    ChassisSpeeds currentSpeeds = SwerveConstants.KINEMATICS.toChassisSpeeds(this.getStates());
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    Rotation2d currentHeading = robotHeading.plus(PoseEstimator.getCurrentPose().getRotation());
    return currentHeading;
  }
}
