// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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

        field.setRobotPose(drivetrain.getPose());
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
    }

    @Override
    public void autonomousInit()
    {
        // Max speed used for the created trajectory
        TrajectoryConfig config = new TrajectoryConfig(Limits.MAX_SPEED/2.0, Limits.MAX_SPEED/2.0);
        // Create trajectory
        // The heading of each waypoint is used to guide the
        // trajectory along the path,
        // it is NOT the actual heading of the robot because
        // robot can swerve!
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1, 0, Rotation2d.fromDegrees(45.0)),
                new Pose2d(1, 1, Rotation2d.fromDegrees(45+90.0)),
                new Pose2d(0, 1, Rotation2d.fromDegrees(45+180.0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0.0))
                   ),
            config);
        
        // Start trajectory at current robot position
        trajectory = trajectory.transformBy(
            new Transform2d(field.getRobotPose().getTranslation(),
                            field.getRobotPose().getRotation()));
        // field.getObject("traj").setTrajectory(trajectory);

        final double robot_heading = field.getRobotPose().getRotation().getDegrees() + 90.0;
        drivetrain.createFollower(robot_heading, trajectory).schedule();
    }

    @Override
    public void autonomousPeriodic()
    {
        // final Pose2d pose = field.getRobotPose();
        // field.setRobotPose(pose.getX() + 1 * TimedRobot.kDefaultPeriod,
        //                    pose.getY(),
        //                    pose.getRotation());
    }
}
