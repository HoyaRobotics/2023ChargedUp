// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoDriveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToLoadingStation extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  private final CANdleSubsystem candleSubsystem;
  private boolean PathCreated = false;
  private Pose2d currentPose;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier relative;
  private final DoubleSupplier maxSpeed;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;
  /** Creates a new DriveToClosestPeg. */
  public DriveToLoadingStation(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator, CANdleSubsystem candleSubsystem, DoubleSupplier translationX, 
  DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, 
  DoubleSupplier maxSpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    this.candleSubsystem = candleSubsystem;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.relative = relative;
    this.maxSpeed = maxSpeed;
    this.xLimiter = new SlewRateLimiter(2.0);
    this.yLimiter = new SlewRateLimiter(2.0);
    this.turnLimiter = new SlewRateLimiter(2.0);
    addRequirements(swerveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      GlobalVariables.isBlue = true;
    }else{
      GlobalVariables.isBlue = false;
    }
    PathCreated = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.currentPose = this.poseEstimator.getPose();
    if(this.currentPose.getX() >= 5.5 && this.currentPose.getY() >= 4.5 && this.currentPose.getX() <= 11.0 && GlobalVariables.isBlue) {
      GlobalVariables.trajectory = PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
        new PathPoint(new Translation2d(12.3, 6.31), Rotation2d.fromDegrees(14.14), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 6.6), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 7.48), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0)));
      poseEstimator.setTrajectoryField2d(GlobalVariables.trajectory);
      //candleSubsystem.setLED(0, 255, 0, 0, 7);
      swerveSubsystem.createCommandForTrajectory(GlobalVariables.trajectory).schedule();
      PathCreated = true;
    }else if(this.currentPose.getX() > 11.0 && this.currentPose.getY() >= 6.0 && GlobalVariables.isBlue) {
      GlobalVariables.trajectory = PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
        new PathPoint(new Translation2d(14.2, 6.6), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 7.48), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(90.0)));
      poseEstimator.setTrajectoryField2d(GlobalVariables.trajectory);
      //candleSubsystem.setLED(0, 255, 0, 0, 7);
      swerveSubsystem.createCommandForTrajectory(GlobalVariables.trajectory).schedule();
      PathCreated = true;
    }


    else if(this.currentPose.getX() >= 5.5 && this.currentPose.getY() <= 3.5 && this.currentPose.getX() <= 11.0 && !GlobalVariables.isBlue) {
      GlobalVariables.trajectory = PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
        new PathPoint(new Translation2d(12.3, 1.69), Rotation2d.fromDegrees(-14.14), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 1.42), Rotation2d.fromDegrees(90.0), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 0.54), Rotation2d.fromDegrees(-90.0), Rotation2d.fromDegrees(-90.0)));
      poseEstimator.setTrajectoryField2d(GlobalVariables.trajectory);
      //candleSubsystem.setLED(0, 255, 0, 0, 7);
      swerveSubsystem.createCommandForTrajectory(GlobalVariables.trajectory).schedule();
      PathCreated = true;
    }else if(this.currentPose.getX() > 11.0 && this.currentPose.getY() <= 2.0 && !GlobalVariables.isBlue) {
      GlobalVariables.trajectory = PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
        new PathPoint(new Translation2d(14.2, 1.42), Rotation2d.fromDegrees(-90.0), Rotation2d.fromDegrees(180)),
        new PathPoint(new Translation2d(14.2, 0.54), Rotation2d.fromDegrees(-90.0), Rotation2d.fromDegrees(-90.0)));
      poseEstimator.setTrajectoryField2d(GlobalVariables.trajectory);
      //candleSubsystem.setLED(0, 255, 0, 0, 7);
      swerveSubsystem.createCommandForTrajectory(GlobalVariables.trajectory).schedule();
      PathCreated = true;
    }
    
    
    else{
      // manual drive
      if(relative.getAsBoolean()){
        swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        poseEstimator.getPoseRotation()));
      } else {
        swerveSubsystem.drive(new ChassisSpeeds(
          modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
          modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
          modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return PathCreated;
  }

  public double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter){
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    if(Math.abs(value)*Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND <= Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND*0.01){
      value = 0.0;
    }
    return value;
  }
}
