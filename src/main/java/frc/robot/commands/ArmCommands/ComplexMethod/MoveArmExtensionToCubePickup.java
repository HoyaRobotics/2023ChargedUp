// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.ComplexMethod;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Classifier;
import frc.robot.subsystems.Pincher;

public class MoveArmExtensionToCubePickup extends CommandBase {
  private final Arm arm;
  private final Pincher pincher;
  private final Classifier classifier;
  private boolean inPosition = false;
  private boolean hasProcessed = false;
  private double extensionPosition = Constants.pickupCubeExtensionPosition;
  private double armPosition = Constants.pickupCubeArmPosition;
  /** Creates a new MoveArmToCubePickup. */
  public MoveArmExtensionToCubePickup(Arm arm, Pincher pincher, Classifier classifier) {
    this.arm = arm;
    this.pincher = pincher;
    this.classifier = classifier;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(classifier.getObject().equals("Cube") && arm.isArmInPosition(Constants.pickupConeArmPosition) && arm.isExtensionInPosition(Constants.pickupConeExtensionPosition)) {
      arm.setArmAnglePID(Constants.pickupCubeArmPosition);
      arm.setExtensionPID(Constants.pickupCubeExtensionPosition);
      armPosition = Constants.pickupCubeArmPosition;
      extensionPosition = Constants.pickupCubeExtensionPosition;
      hasProcessed = true;
    }
    
    else if(classifier.getObject().equals("Cube") && arm.isArmInPosition(Constants.pickupCubeArmPosition) && arm.isExtensionInPosition(Constants.pickupCubeExtensionPosition)) {
      arm.setArmAnglePID(Constants.pickupCubeArmPosition);
      arm.setExtensionPID(Constants.pickupCubeExtensionPosition);
      armPosition = Constants.pickupCubeArmPosition;
      extensionPosition = Constants.pickupCubeExtensionPosition;
      hasProcessed = true;
    }
    
    else{
      hasProcessed = true;
      inPosition = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(hasProcessed == true && arm.isArmInPosition(armPosition) && arm.isExtensionInPosition(extensionPosition)) {
      inPosition = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pincher.togglePincher();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inPosition;
  }
}
