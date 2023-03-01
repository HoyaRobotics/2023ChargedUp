// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropConeAgain extends SequentialCommandGroup {
  /** Creates a new DropConeAgain. */
  public DropConeAgain(Arm arm, Grabber grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveExtensionToPosition(arm, 100),
      new MoveArmToPosition(arm, 1),
      new MoveExtensionToPosition(arm, 80),
      new Release(grabber),
      new WaitCommand(0.5),
      new MoveArmToPosition(arm, 0.5),
      new MoveExtensionToPosition(arm, 26.5)
    );
  }
}