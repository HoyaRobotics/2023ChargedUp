// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Storage;

public class StopConveyor extends CommandBase {
  private final Storage storage;
  /** Creates a new StopConveyor. */
  public StopConveyor(Storage storage) {
    this.storage = storage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(storage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    storage.stopConveyors();
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
    return true;
  }
}
