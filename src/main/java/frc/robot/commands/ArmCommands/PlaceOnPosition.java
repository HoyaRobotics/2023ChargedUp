// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pincher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceOnPosition extends SequentialCommandGroup {
  /** Creates a new PlaceOn3rd. */
  public PlaceOnPosition(Arm arm, Pincher pincher, IntSupplier level) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Grip(pincher),
      //new MoveExtensionToPosition(arm, () -> Constants.placeExtensionPosition),
      new ThresholdExtensionToPosition(arm, () -> Constants.placeExtensionPosition, () -> 70, () -> true), //80 is safe
      //new MoveArmToPosition(arm, () -> Constants.ARM_POSITIONS.get(level.getAsInt())),
      new ThresholdArmToPosition(arm, () -> Constants.ARM_POSITIONS.get(level.getAsInt()), () -> Constants.ARM_POSITIONS.get(level.getAsInt()) + 7.0, () -> false),
      new MoveExtensionToPosition(arm, () -> Constants.EXTENSION_POSITIONS.get(level.getAsInt()))
    );
  }
}
