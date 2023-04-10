// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Classifier extends SubsystemBase {
  static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  static NetworkTableEntry objectNumber = table.getEntry("tclass");
  /** Creates a new Classifier. */
  public Classifier() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Object", getObject());
    //Logger.getInstance().recordOutput("Object", getObject());
  }

  public String getObject() {
    //return objectNumber.getClass();
    return objectNumber.getString("None");
  }
}
