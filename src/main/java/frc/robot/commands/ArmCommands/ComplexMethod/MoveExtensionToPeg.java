// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.ComplexMethod;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import frc.robot.subsystems.Arm;

public class MoveExtensionToPeg extends CommandBase {
  private final Arm arm;
  private int pegPosition;
  /** Creates a new RotateArmToAngle. */
  public MoveExtensionToPeg(Arm arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pegPosition = GlobalVariables.upDownPosition;
    arm.setExtensionPID(Constants.EXTENSION_POSITIONS.get(pegPosition));
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
    if(arm.isExtensionInPosition(Constants.EXTENSION_POSITIONS.get(pegPosition))) {
      return true;
    }else{
      return false;
    }
  }
}
