// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoDriveCommands;

import java.util.List;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveToPeg extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  private Pose2d currentPose;
  private List<PathPoint> pathPoints;

  /** Creates a new DriveToPeg. */
  public DriveToPeg(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = poseEstimator.getPose();
    if(currentPose.getX() <= 5 && currentPose.getY() <= 5) {
      //create path
      if(currentPose.getX() >= 2.45) {
        //create longest path
        if(currentPose.getY() <= 2.8) {
          //create longest right path
          pathPoints.add(new PathPoint(new Translation2d(2.45, 0.8), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)));
        }else{
          //create longest left path
        }
      }else{
        //create shorter path
      }
    }else{
      //drive manualy
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
