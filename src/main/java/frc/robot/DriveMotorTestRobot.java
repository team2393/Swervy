// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.drivetrain.Driver;

/** Robot for testing single swerve module's drive motor
 * 
 *  Right joysick: Drive
 *  Left bumper: Reset position to zero
 *  Hold right bumber: Closed loop -1..1 m/s
 */
public class DriveMotorTestRobot extends TimedRobot
{
    private final Driver driver = new Driver(1);

    final CommandBase reset = new InstantCommand(driver::reset);
                                             
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
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Speed", driver.getSpeed());
        SmartDashboard.putNumber("Position", driver.getPosition());
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        if (OperatorInterface.joystick.getLeftBumperPressed())
            reset.schedule();

        final double input = -OperatorInterface.joystick.getRightY();
        SmartDashboard.putNumber("Joystick", input);
        final double speed = MathUtil.applyDeadband(input, Settings.DEAD_STICK);
        if (OperatorInterface.joystick.getRightBumper())
        {
            driver.setSpeed(Settings.MAX_SPEED*speed);
            SmartDashboard.putNumber("Request", Settings.MAX_SPEED*speed);
        }
        else
        {
            driver.setOutput(speed);
            SmartDashboard.putNumber("Request", speed);
        }
    }
}
