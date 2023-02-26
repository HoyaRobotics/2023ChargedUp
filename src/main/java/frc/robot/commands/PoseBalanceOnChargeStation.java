// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class PoseBalanceOnChargeStation extends CommandBase {
  private final PIDController x, y, yaw;

  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  /** Creates a new PIDBalanceOnChargeStation. */
  public  PoseBalanceOnChargeStation(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;

    x = new PIDController(0.01, 0, 0);
    y = new PIDController(0.01, 0, 0);
    yaw = new PIDController(0.08, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(DriverStation.getAlliance() == Alliance.Blue) {
      x.setSetpoint(3.9);
    }else{
      x.setSetpoint(12.6);
    }
    x.setTolerance(0.1);
    y.setSetpoint(2.75);
    y.setTolerance(0.1);
    yaw.setSetpoint(0);
    yaw.setTolerance(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(yaw.atSetpoint() == false || y.atSetpoint() == false) {
      swerveSubsystem.drive(new ChassisSpeeds(0, y.calculate(poseEstimator.getPoseY()), yaw.calculate(poseEstimator.getPoseTheta())));
    }else{
      swerveSubsystem.drive(new ChassisSpeeds(x.calculate(poseEstimator.getPoseX()), y.calculate(poseEstimator.getPoseY()), yaw.calculate(poseEstimator.getPoseTheta())));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.lock();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
