// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoDriveCommands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToClosestPeg extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  private boolean PathCreated = false;
  private Pose2d currentPose;
  private Pose2d endPose;
  private int position;
  private double distance = 100;
  /** Creates a new DriveToClosestPeg. */
  public DriveToClosestPeg(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = poseEstimator.getPose();
    if(currentPose.getX() <= 2.5 && currentPose.getY() <= 5) {
      // find closest position
      for(int i = 0; i < 9; i++) {
        double yPosition = Constants.PEG_POSE.get(i).getY();
        double tempDistance = Math.abs(currentPose.getY() - yPosition);
        if(tempDistance < distance) {
          distance = tempDistance;
          position = i;
        }
      }
      endPose = Constants.PEG_POSE.get(position);
      GlobalVariables.trajectory = PathPlanner.generatePath(
        new PathConstraints(2, 2),
        new PathPoint(new Translation2d(poseEstimator.getPoseX(), poseEstimator.getPoseY()), swerveSubsystem.getCurrentChassisHeading(), poseEstimator.getPoseRotation(), swerveSubsystem.getCurrentChassisSpeeds()),
        new PathPoint(new Translation2d(endPose.getX()+0.25, endPose.getY()), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
        new PathPoint(new Translation2d(endPose.getX(), endPose.getY()), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
      poseEstimator.setTrajectoryField2d(GlobalVariables.trajectory);
      PathCreated = true;
    }else{
      // manual drive
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
}