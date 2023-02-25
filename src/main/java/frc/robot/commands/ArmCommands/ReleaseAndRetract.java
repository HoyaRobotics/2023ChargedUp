// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Grabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReleaseAndRetract extends SequentialCommandGroup {
  /** Creates a new ReleaseAndRetract. */
  public ReleaseAndRetract(Grabber grabber, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Release(grabber),
      new WaitCommand(0.2),
      new InstantCommand(() -> arm.setExtensionPIDValue(0.06), arm),
      new MoveExtensionToPosition(arm, 110),
      new MoveArmToPosition(arm, 7.0),
      new InstantCommand(() -> arm.setExtensionPIDValue(0.02), arm),
      new MoveExtensionToPosition(arm, 25.5),
      new InstantCommand(() -> arm.setExtensionPIDValue(0.06), arm)
    );
  }
}