// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.DriveByJoystickCommand;
import frc.robot.drivetrain.Drivetrain;

/** Robot for testing swerve drive */
public class SwervyRobot extends TimedRobot
{
    private final Drivetrain drivetrain = new Drivetrain();
    private final DriveByJoystickCommand joydrive = new DriveByJoystickCommand(drivetrain);
    private final Field2d field = new Field2d();

    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Drivetrain", drivetrain);
    }

    @Override
    public void robotPeriodic()
    {
        // This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update field with latest robot location
        field.setRobotPose(drivetrain.getPose());
    }

    @Override
    public void teleopInit()
    {
        drivetrain.reset(7.64, 1.764, 90);
        joydrive.schedule();
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void autonomousInit()
    {
        final SequentialCommandGroup auto = new SequentialCommandGroup();
        
        // Create trajectory
        // The heading of each waypoint is used to guide the
        // trajectory along the path,
        // it is NOT the actual heading of the robot because
        // robot can swerve!
        
        // Start trajectory at current robot position
        // trajectory = trajectory.transformBy(
        //     new Transform2d(field.getRobotPose().getTranslation(),
        //                     field.getRobotPose().getRotation()));
        // field.getObject("traj").setTrajectory(trajectory);
        // final double robot_heading = field.getRobotPose().getRotation().getDegrees() + 90.0;

        auto.addCommands(drivetrain.createFollower(45,
                                                   7.64, 1.764, 90.0,
                                                   7.64, 2.764, 90.0));

        auto.addCommands(drivetrain.createFollower(90, 
                                                   7.64, 2.764, -90.0,
                                                   7.64, 1.764, -90.0,
                                                   8.374, 0.453,-90.0));
        
        auto.addCommands(drivetrain.createFollower(40+180, 
                                                   8.374, 0.453, 179.5,
                                                   7.144, 0.453, 179.5,
                                                   5.09,  1.96, 115.0));

        auto.addCommands(drivetrain.createFollower(41.5,
                                                   5.09, 1.96, 40.0+180,
                                                   2.61, 1.09, 13.0+180,
                                                   1.0, 1.17, -45.0+180));

        auto.schedule();
    }

    @Override
    public void autonomousPeriodic()
    {
    }
}
