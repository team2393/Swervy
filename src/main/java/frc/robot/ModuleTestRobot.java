// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.drivetrain.SparkMini;

/** Robot for testing single swerve module */
public class ModuleTestRobot extends TimedRobot
{
    private final SparkMini rotator = new SparkMini(0);

    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        // "Forward" for positive
        rotator.set(MathUtil.applyDeadband(-OperatorInterface.joystick.getRightY(),
                                           0.05));
    }
}
