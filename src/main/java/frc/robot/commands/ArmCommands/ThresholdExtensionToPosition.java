// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ThresholdExtensionToPosition extends CommandBase {
  private final Arm arm;
  private DoubleSupplier extension;
  private DoubleSupplier threshold;
  private BooleanSupplier retracting;
  /** Creates a new MoveExtensionToPosition. */
  public ThresholdExtensionToPosition(Arm arm, DoubleSupplier extension, DoubleSupplier threshold, BooleanSupplier retracting) {
    this.arm = arm;
    this.extension = extension;
    this.threshold = threshold;
    this.retracting = retracting;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setExtensionPID(extension.getAsDouble());
    //System.out.println("Extension position is set");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Extension is past threshold");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isExtensionPastPositon(threshold.getAsDouble(), retracting.getAsBoolean());
  }
}
