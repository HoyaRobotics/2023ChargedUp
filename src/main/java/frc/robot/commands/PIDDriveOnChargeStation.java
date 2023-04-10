// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class PIDDriveOnChargeStation extends CommandBase {
  private final PIDController pidController = new PIDController(0.09, 0, 0);
  //0.045 works
  //0.06 works but had to relevel once or twice
  private final PIDController yaw = new PIDController(0.3, 0, 0);

  private final Pigeon2Subsystem pigeon2Subsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;
  /** Creates a new PIDBalanceOnChargeStation. */
  public  PIDDriveOnChargeStation(Pigeon2Subsystem pigeon2Subsystem, SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    this.pigeon2Subsystem = pigeon2Subsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;

    pidController.reset();
    yaw.reset();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.setSetpoint(0);
    pidController.setTolerance(10.0); //was 10
    yaw.setSetpoint(0);
    yaw.setTolerance(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!yaw.atSetpoint()) {
      swerveSubsystem.drive(new ChassisSpeeds(0, 0, yaw.calculate(poseEstimator.getPoseTheta())));
      //System.out.println("Not Oriented");
    }else{
      //System.out.println("Oriented");
      if(!pidController.atSetpoint()) {
        //System.out.println("Not Level");
        swerveSubsystem.drive(new ChassisSpeeds(pidController.calculate(pigeon2Subsystem.getPigeonPitch()), 0, yaw.calculate(poseEstimator.getPoseTheta())));
      } else {
        //System.out.println("Level");
        pidController.calculate(pigeon2Subsystem.getPigeonPitch());
        swerveSubsystem.stop();
      }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pidController.reset();
    yaw.reset();
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(yaw.atSetpoint() && pidController.atSetpoint()) {
      return true;
    }else{
      return false;
    }
  }
}
