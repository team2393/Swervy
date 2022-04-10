// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.Driver;
import frc.robot.drivetrain.Rotator;

/** Robot for testing single swerve module
 * 
 *  Left joysick: Rotate
 *  Left Trigger: Rotate real slow
 *  Left bumper: Reset heading and position to zero
 *  Hold right bumber: Closed loop -180..180

 *  Right joysick: Driver speed
 * 
 *  "Raw Angle" can be used to determine the zero offset of a module
 */
public class ModuleTestRobot extends TimedRobot
{
    private final int MODULE = 1;
    private final Rotator rotator = new Rotator(MODULE, Settings.MODULE_ZERO[MODULE]);
    private final Driver driver = new Driver(MODULE+1);
    
    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");
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

        SmartDashboard.putNumber("Position", driver.getPosition());
        SmartDashboard.putNumber("Speed", driver.getSpeed());
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        if (OperatorInterface.joystick.getLeftBumperPressed())
        {
            rotator.reset();
            driver.reset();
        }

        double input = -OperatorInterface.joystick.getLeftX();
        SmartDashboard.putNumber("Joystick Rotation", input);
        double rotation = MathUtil.applyDeadband(input, Settings.DEAD_STICK);
        if (OperatorInterface.joystick.getLeftTriggerAxis() > 0.5)
            rotation /= 4;
        if (OperatorInterface.joystick.getRightBumper())
        {
            // Avoid 180 because of wraparound
            rotator.setHeading(rotation * 170.0);
        }
        else
            rotator.setRotation(rotation);

        input = -OperatorInterface.joystick.getRightY();
        SmartDashboard.putNumber("Joystick Speed", input);
        final double speed = MathUtil.applyDeadband(input, Settings.DEAD_STICK);
        driver.setSpeed(Settings.MAX_SPEED * speed);
    }
}
