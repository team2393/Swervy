// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/** Main class */
public final class Main
{
    /** Main initialization function. Launches robot code. */
    public static void main(String... args)
    {
        RobotBase.startRobot(ModuleTestRobot::new);
    }
}
