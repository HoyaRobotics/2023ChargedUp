// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.AutoSpecific;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveExtensionToPosition;
import frc.robot.commands.ArmCommands.ThresholdArmToPosition;
import frc.robot.commands.ArmCommands.ThresholdExtensionToPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pincher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceOnPositionAuto extends SequentialCommandGroup {
  /** Creates a new PlaceOnPositionAuto. */
  public PlaceOnPositionAuto(Arm arm, Pincher pincher, IntSupplier level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> arm.setArmPIDValue(Constants.placeArmPGain), arm),
      new ThresholdExtensionToPosition(arm, () -> Constants.placeExtensionPosition, () -> 80, () -> true),
      new ThresholdArmToPosition(arm, () -> Constants.ARM_POSITIONS.get(level.getAsInt()), () -> Constants.ARM_POSITIONS.get(level.getAsInt()) + 5.0, () -> false),
      new MoveExtensionToPosition(arm, () -> Constants.EXTENSION_POSITIONS.get(level.getAsInt())),
      new WaitCommand(0.25)
    );
  }
}
