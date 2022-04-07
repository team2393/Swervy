// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.drivetrain.Module;

/** Robot for testing single swerve module */
public class ModuleTestRobot extends TimedRobot
{
    private final Module module = new Module("Swerve1",
                                             new Translation2d(1, 0),
                                             0);

    final CommandBase reset = new InstantCommand(module::reset);
                                             
    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");

        // runner.configFactoryDefault();
        // runner.clearStickyFaults();
        // runner.configOpenloopRamp(1.0);

        SmartDashboard.setDefaultNumber("heading P", 0.5);
        SmartDashboard.setDefaultNumber("heading I", 0);
        SmartDashboard.setDefaultNumber("heading D", 0);
    }

    private double last_angle = 0.0;

    @Override
    public void robotPeriodic()
    {
        CommandScheduler.getInstance().run();

        final double angle = module.getHeading().getDegrees();
        final double rot_speed = (angle - last_angle) / TimedRobot.kDefaultPeriod;
        last_angle = angle;
        SmartDashboard.putNumber("Rotation Speed", rot_speed);
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

        // "Forward" for positive
        final double speed = MathUtil.applyDeadband(-OperatorInterface.joystick.getRightY(),
                                                    0.05);
        final double rotation = MathUtil.applyDeadband(-OperatorInterface.joystick.getLeftX(),
                                                       0.05);
        if (OperatorInterface.joystick.getRightBumper())
        {
            module.configureHeadingPID(SmartDashboard.getNumber("heading P", 0),
                                       SmartDashboard.getNumber("heading I", 0),
                                       SmartDashboard.getNumber("heading D", 0));
            // Avoid 180 because of wraparound
            module.drive(speed, rotation * 170.0);
        }
        else
            module.driveDirect(speed, rotation);
    }
}
