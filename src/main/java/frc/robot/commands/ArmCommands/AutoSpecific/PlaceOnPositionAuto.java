// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands.AutoSpecific;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ArmCommands.MoveArmToPosition;
import frc.robot.commands.ArmCommands.MoveExtensionToPosition;
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
      new MoveExtensionToPosition(arm, () -> Constants.placeExtensionPosition),
      new MoveArmToPosition(arm, () -> Constants.ARM_POSITIONS.get(level.getAsInt())),
      new MoveExtensionToPosition(arm, () -> Constants.EXTENSION_POSITIONS.get(level.getAsInt()))
    );
  }
}
