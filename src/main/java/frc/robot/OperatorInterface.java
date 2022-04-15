// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;

/** Joystick etc. as used by driver(s) */
public class OperatorInterface
{
    public final static XboxController joystick = new XboxController(0);

    private final static double DEADBAND = 0.05;

    /** @return Forward (positive) or backwards (negative) speed [m/s] */
    public static double getForwardBackward()
    {
        // Max speed 1 m/s
        return -Settings.getMaxSpeed() * MathUtil.applyDeadband(joystick.getRightY(), DEADBAND);
    }

    /** @return Left (positive) or right (negative) speed [m/s] */
    public static double getLeftRight()
    {
        return -Settings.getMaxSpeed() * MathUtil.applyDeadband(joystick.getRightX(), DEADBAND);
    }

    /** @return Rotation, positive for counter-clockwise [degrees/s]*/
    public static double getRotation()
    {
        return -Settings.getMaxManualRotation() * MathUtil.applyDeadband(joystick.getLeftX(), DEADBAND);
    }

    /** @return Pivot around left front? */
    public static boolean pivotLeft()
    {
        return joystick.getLeftBumper();
    }

    /** @return Pivot around right front? */
    public static boolean pivotRight()
    {
        return joystick.getRightBumper();
    }
}
