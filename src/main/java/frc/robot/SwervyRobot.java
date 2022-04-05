// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.drivetrain.Drivetrain;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class SwervyRobot extends TimedRobot
{
    private final Drivetrain drivetrain = new Drivetrain();
    Field2d field = new Field2d();

    @Override
    public void robotInit()
    {
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void robotPeriodic()
    {
        // This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    @Override
    public void teleopInit()
    {
        drivetrain.reset(7.64, 1.764, 90);
    }

    @Override
    public void teleopPeriodic()
    {
        int pivot;
        if (OperatorInterface.pivotLeft())
            pivot = 0;
        else if (OperatorInterface.pivotRight())
            pivot = 1;
        else
            pivot = -1;
        drivetrain.drive(OperatorInterface.getForwardBackward(),
                         OperatorInterface.getLeftRight(),
                         OperatorInterface.getRotation(),
                      
                         pivot);
        
        field.setRobotPose(drivetrain.getPose());
    }

    @Override
    public void autonomousPeriodic()
    {
        final Pose2d pose = field.getRobotPose();
        field.setRobotPose(pose.getX() + 1 * TimedRobot.kDefaultPeriod,
                           pose.getY(),
                           pose.getRotation());
    }
}
