// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.AutoSpecific;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArmToPosition;
import frc.robot.commands.ArmCommands.MoveExtensionToPosition;
import frc.robot.commands.ArmCommands.ThresholdArmToPosition;
import frc.robot.commands.ArmCommands.ThresholdExtensionToPosition;
import frc.robot.subsystems.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractFromPositionAuto extends SequentialCommandGroup {
  /** Creates a new RetractFromLevelAuto. */
  public RetractFromPositionAuto(Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> arm.setArmPIDValue(Constants.returnArmPGain), arm),
      //new MoveExtensionToPosition(arm, () -> Constants.placeExtensionPosition),
      new ThresholdExtensionToPosition(arm, () -> Constants.placeExtensionPosition, () -> 80.0, () -> true),
      //new MoveArmToPosition(arm, () -> Constants.pickupConeArmPosition),
      new ThresholdArmToPosition(arm, () -> Constants.pickupConeArmPosition, () -> -2.0, () -> true),
      new MoveExtensionToPosition(arm, () -> Constants.pickupConeExtensionPosition)
    );
  }
}
