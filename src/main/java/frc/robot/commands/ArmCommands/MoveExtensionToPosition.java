// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveExtensionToPosition extends CommandBase {
  private final Arm arm;
  private DoubleSupplier extension;
  /** Creates a new MoveExtensionToPosition. */
  public MoveExtensionToPosition(Arm arm, DoubleSupplier extension) {
    this.arm = arm;
    this.extension = extension;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setExtensionPID(extension.getAsDouble());
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
    if(arm.isExtensionInPosition(extension.getAsDouble())) {
      return true;
    }else{
      return false;
    }
  }
}
