// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

/** Speed limits as they're used to operate the robot
 * 
 *  May be lower than actual maximum
 */
public class Limits
{
    /** Maximum swerve speed [m/s] */
    public static final double MAX_SPEED = 2.5;

    /** Maximum rotational speed [degrees/s] */
    // Actual maximum is around 600 deg/s
    public static final double MAX_ROTATION = 500.0;

    /** Maximum rotational voltage */
    public static final double MAX_ROTATION_VOLTAGE = 8.0;

}