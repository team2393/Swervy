// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/** Speed limits, zero headings etc. */
public class Settings
{    
    /** Joystick deadband */
    public static final double DEAD_STICK = 0.15;

    /** Maximum swerve speed [m/s] */
    // Actual maximum is about 3 m/s
    public static final double MAX_SPEED = 1.5;

    /** Maximum rotational speed [degrees/s] */
    // Actual maximum is around 600 deg/s
    public static final double MAX_ROTATION = 500.0;

    /** Maximum rotational voltage */
    public static final double MAX_ROTATION_VOLTAGE = 8.0;

    /** Zero heading for module 0-3 */
    public static final double[] MODULE_ZERO = { 342.0,
                                                  89.0,
                                                 198.0,
                                                 254.0 };
}