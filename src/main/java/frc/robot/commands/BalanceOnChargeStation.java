// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceOnChargeStation extends CommandBase {
  private final Pigeon2Subsystem pigeon2Subsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;

  private double pitchState;
  private double rollState;

  /** Creates a new BalanceOnChargeStation. */
  public BalanceOnChargeStation(Pigeon2Subsystem pigeon2Subsystem, SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.pigeon2Subsystem = pigeon2Subsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(pigeon2Subsystem.evaluatePitch()){
      case 0:
      pitchState = 0;
      break;
      case 1:
      pitchState = 0.1;
      break;
      case 2:
      pitchState = -0.1;
      break;
    }

    switch(pigeon2Subsystem.evaluateRoll()){
      case 0:
      rollState = 0;
      break;
      case 1:
      rollState = 0.1;
      break;
      case 2:
      rollState = -0.1;
      break;
    }
    swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(pitchState, rollState, 0, poseEstimator.getPoseRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
