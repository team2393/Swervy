// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Speed limits, zero headings etc. */
public class Settings
{    
    /** Joystick deadband */
    public static final double DEAD_STICK = 0.15;

    /** Maximum swerve speed [m/s] */
    // Actual maximum is about 3 m/s
    public static final double MAX_SPEED = 0.3;

    /** Maximum rotational speed of modules [degrees/s] */
    // Actual maximum is around 600 deg/s
    public static final double MAX_ROTATION = 360.0;

    /** Maximum rotational voltage */
    public static final double MAX_ROTATION_VOLTAGE = 8.0;

    /** Maximum rotational speed when using joystick to turn [degrees/s] */
    public static final double MAX_MANUAL_ROTATION = 30.0;

    /** Zero heading for module 0-3 */
    public static final double[] MODULE_ZERO = { 342.0,
                                                  89.0,
                                                 198.0,
                                                 254.0 };

    private static NetworkTableEntry nt_max_speed, nt_max_manual_rotation;

    static
    {
        nt_max_speed = SmartDashboard.getEntry("Max Speed");
        nt_max_speed.setDefaultDouble(Settings.MAX_SPEED);
        nt_max_manual_rotation = SmartDashboard.getEntry("Max Manual Rotation");
        nt_max_manual_rotation.setDefaultDouble(Settings.MAX_MANUAL_ROTATION);
    }

    /** @return Maximum speed [m/s] */
    public static double getMaxSpeed()
    {
        return nt_max_speed.getDouble(Settings.MAX_SPEED);
    }

    /** @return Maximum rotational speed for manual drive [degree/s] */
    public static double getMaxManualRotation()
    {
        return nt_max_manual_rotation.getDouble(Settings.MAX_MANUAL_ROTATION);
    }    
}