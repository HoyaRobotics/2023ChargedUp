// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pincher;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropConeAgain extends SequentialCommandGroup {
  /** Creates a new DropConeAgain. */
  public DropConeAgain(Arm arm, Pincher pincher) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      //faster pid
      new InstantCommand(() -> arm.setArmPIDValue(Constants.returnArmPGain), arm),
      new MoveExtensionToPosition(arm, () -> Constants.holdExtensionPosition),
      new MoveArmToPosition(arm, () -> Constants.pickupConeArmPosition),
      new Release(pincher),
      new WaitCommand(0.2),
      new MoveExtensionToPosition(arm, () -> Constants.pickupConeExtensionPosition)
    );
  }
}
