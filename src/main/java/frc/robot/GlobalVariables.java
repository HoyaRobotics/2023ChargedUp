// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;

/** Add your docs here. */
public class GlobalVariables {
    public static boolean INTAKE_LOWERED = false;
    public static double maxSpeed = Constants.DRIVE_SPEED;
    public static boolean fieldRelative = true;
    public static int upDownPosition = 2;
    public static int leftRightPosition = 1;
    public static boolean isCone = false;
    public static PathPlannerTrajectory trajectory;
    public static boolean isBlue = true;
    public static boolean hasConnectedToDS = false;
    public static double pigeonPitch = 0.0;
}
