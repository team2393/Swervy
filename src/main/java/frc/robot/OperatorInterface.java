// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/** Joystick etc. as used by driver(s) */
public class OperatorInterface
{
    public final static XboxController joystick = new XboxController(0);

    /** @return Forward (positive) or backwards (negative) speed */
    public static double getForwardBackward()
    {
        return -joystick.getRightY();
    }

    /** @return Left (positive) or right (negative) speed */
    public static double getLeftRight()
    {
        return -joystick.getRightX();
    }

    public static double getRotation()
    {
        return joystick.getLeftX();
    }
}
