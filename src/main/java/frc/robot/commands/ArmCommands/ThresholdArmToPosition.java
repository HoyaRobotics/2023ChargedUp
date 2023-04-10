// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ThresholdArmToPosition extends CommandBase {
  private final Arm arm;
  private DoubleSupplier angle;
  private DoubleSupplier threshold;
  private BooleanSupplier goingDown;
  /** Creates a new MoveArmToPosition. */
  public ThresholdArmToPosition(Arm arm, DoubleSupplier angle, DoubleSupplier threshold, BooleanSupplier goingDown) {
    this.arm = arm;
    this.angle = angle;
    this.threshold = threshold;
    this.goingDown = goingDown;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setArmAnglePID(angle.getAsDouble());
    //System.out.println("Arm position is set");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Arm is past threshold");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmPastPosition(threshold.getAsDouble(), goingDown.getAsBoolean());
  }
}
