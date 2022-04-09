// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.Rotator;

/** Robot for testing single swerve module's rotator
 * 
 *  Left joysick: Rotate
 *  Left bumper: Reset heading to zero
 *  Hold right bumber: Closed loop -180..180
 * 
 *  "Raw Angle" can be used to determine the zero offset of a module
 */
public class RotatorTestRobot extends TimedRobot
{
    private final Rotator rotator = new Rotator(0, 0.0);
                                             
    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");

        SmartDashboard.setDefaultNumber("heading P", 0.5);
        SmartDashboard.setDefaultNumber("heading I", 0);
        SmartDashboard.setDefaultNumber("heading D", 0);
    }

    private double last_angle = 0.0;

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        final double angle = rotator.getRawHeading();
        final double diff = (Math.abs(angle - last_angle) + 360.0) % 360.0;
        final double rot_speed = diff / TimedRobot.kDefaultPeriod;
        last_angle = angle;
        SmartDashboard.putNumber("Rotation Speed", rot_speed);
        SmartDashboard.putNumber("Raw Angle", angle);
        SmartDashboard.putNumber("Heading", rotator.getHeading().getDegrees());
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        if (OperatorInterface.joystick.getLeftBumperPressed())
            rotator.reset();

        final double input = -OperatorInterface.joystick.getLeftX();
        SmartDashboard.putNumber("Joystick", input);
        final double rotation = MathUtil.applyDeadband(input, Limits.DEAD_STICK);
        if (OperatorInterface.joystick.getRightBumper())
        {
            rotator.configureHeadingPID(SmartDashboard.getNumber("heading P", 0),
                                        SmartDashboard.getNumber("heading I", 0),
                                        SmartDashboard.getNumber("heading D", 0));
            // Avoid 180 because of wraparound
            rotator.setHeading(rotation * 170.0);
        }
        else
            rotator.setRotation(rotation);
    }
}
