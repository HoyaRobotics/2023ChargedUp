// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GrabberConstants;

public class Grabber extends SubsystemBase {
  Compressor phCompressor = new Compressor(GrabberConstants.PNUMATICS_HUB, PneumaticsModuleType.REVPH);
  DoubleSolenoid grabberDoubleSolenoid = new DoubleSolenoid(GrabberConstants.PNUMATICS_HUB, PneumaticsModuleType.REVPH, GrabberConstants.GRIPPER_OPEN, GrabberConstants.GRIPPER_CLOSE);
  /** Creates a new Grabber. */
  public Grabber() {
    phCompressor.enableDigital();
    openGrabber();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openGrabber() {
    grabberDoubleSolenoid.set(Value.kReverse);
  }

  public void closeGrabber() {
    grabberDoubleSolenoid.set(Value.kForward);
  }
}
;