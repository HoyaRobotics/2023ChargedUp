// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PincherConstants;

public class Pincher extends SubsystemBase {
  Compressor Compressor = new Compressor(PincherConstants.PNUMATICS_MODULE_ID, PincherConstants.PNEUMATICS_MODULE_TYPE);
  DoubleSolenoid pincherDoubleSolenoid = new DoubleSolenoid(PincherConstants.PNUMATICS_MODULE_ID, PincherConstants.PNEUMATICS_MODULE_TYPE, PincherConstants.GRIPPER_OPEN, PincherConstants.GRIPPER_CLOSE);
  /** Creates a new Pincher. */
  public Pincher() {
    Compressor.enableDigital();
    openPincher();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openPincher() {
    pincherDoubleSolenoid.set(Value.kReverse);
  }

  public void closePincher() {
    pincherDoubleSolenoid.set(Value.kForward);
  }

  public void togglePincher() {
    pincherDoubleSolenoid.toggle();
  }
}
;