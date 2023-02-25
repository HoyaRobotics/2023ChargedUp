// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDBalanceOnChargeStation extends CommandBase {
  private final PIDController pidController;
  private final PIDController yaw;

  private final Pigeon2Subsystem pigeon2Subsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  /** Creates a new PIDBalanceOnChargeStation. */
  public  PIDBalanceOnChargeStation(Pigeon2Subsystem pigeon2Subsystem, SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.pigeon2Subsystem = pigeon2Subsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;

    pidController = new PIDController(0.01, 0, 0);
   // pidController = new PIDController(-0.0125, 0, 0);
    yaw = new PIDController(0.08, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);
    pidController.setTolerance(3.5);
    yaw.setSetpoint(0);
    yaw.setTolerance(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(yaw.atSetpoint() == false) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, yaw.calculate(poseEstimator.getPoseTheta())));
    }else{
      swerveSubsystem.drive(new ChassisSpeeds(pidController.calculate(pigeon2Subsystem.getPigeonPitch()), 0, yaw.calculate(poseEstimator.getPoseTheta())));
    }
    
    if (yaw.atSetpoint()) {
      SmartDashboard.putBoolean("ALIGNED?", true);
    } else {
      SmartDashboard.putBoolean("ALIGNED?", false);
    }
    if (pidController.atSetpoint()) {
      SmartDashboard.putBoolean("LEVEL?", true);
    } else {
      SmartDashboard.putBoolean("LEVEL?", false);
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
