// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.DriveByJoystickCommand;
import frc.robot.drivetrain.Drivetrain;
import frc.robot.drivetrain.ResetCommand;

/** Robot for testing swerve drive */
public class SwervyRobot extends TimedRobot
{
    private final Drivetrain drivetrain = new Drivetrain();
    private final DriveByJoystickCommand joydrive = new DriveByJoystickCommand(drivetrain);

    /** Location of robot on field */
    private final Field2d field = new Field2d();

    /** Offsets between robot's internal odometry and field location */
    private final NetworkTableEntry nt_field_x = SmartDashboard.getEntry("Field X"),
                                    nt_field_y = SmartDashboard.getEntry("Field Y"),
                                    nt_field_heading = SmartDashboard.getEntry("Field Heading");

    private final SendableChooser<CommandBase> auto_options = new SendableChooser<>();

    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");

        nt_field_x.setDefaultDouble(0.0);
        nt_field_y.setDefaultDouble(0.0);
        nt_field_heading.setDefaultDouble(0.0);

        SmartDashboard.putData("Field", field);
        SmartDashboard.putData("Drivetrain", drivetrain);

        SmartDashboard.putData(new ResetCommand(drivetrain));

        auto_options.setDefaultOption("Nothing", new InstantCommand());

        CommandBase option = new SequentialCommandGroup(
            new ResetCommand(drivetrain),
            drivetrain.createFollower(0,
                                      0.00, 0.00, 0.0,
                                      0.50, 0.00, 0.0));
        option.setName("0.5 Fwd");
        auto_options.addOption(option.getName(), option);


        option = new SequentialCommandGroup(
            new ResetCommand(drivetrain),
            drivetrain.createFollower(-90,
                                      0.00, 0.00, 0.0,
                                      0.50, 0.00, 0.0));
        option.setName("0.5 FwdRot");
        auto_options.addOption(option.getName(), option);


        option = new SequentialCommandGroup(
            new ResetCommand(drivetrain),
            drivetrain.createFollower(-1*45,
                                      0.00, 0.00, 0.0,
                                      0.50, 0.00, 0.0),
            drivetrain.createFollower(-2*45,
                                      0.50, 0.00, 90.0,
                                      0.50, 0.50, 90.0),
            drivetrain.createFollower(-1*45,
                                      0.50, 0.50, 180.0,
                                      0.00, 0.50, 180.0),
            drivetrain.createFollower(0,
                                      0.00, 0.50, -90.0,
                                      0.00, 0.00, -90.0));
        option.setName("0.5 Square");
        auto_options.addOption(option.getName(), option);

/*
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

        auto.addCommands(drivetrain.createFollower(0,
                                                   0.00, 0.00, -155.0,
                                                   -1.35, -0.64, -155.0));
        auto.addCommands(drivetrain.createFollower(-50,
                                                   -1.35, -0.64, 90.0,
                                                   -1.35,  0.64, 90.0,
                                                    0.12,  2.58, 45.0 ));
        auto.addCommands(drivetrain.createFollower(-45,
                                                     0.12,  2.58, 135.0,
                                                    -0.80,  5.73, 80.0,
                                                    -0.55,  6.55, 42.0 ));
*/

        SmartDashboard.putData("Auto Options", auto_options);
    }

    @Override
    public void robotPeriodic()
    {
        // This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        // Update field with latest robot location
        final Pose2d field_pos = new Pose2d(nt_field_x.getDouble(0.0),
                                            nt_field_y.getDouble(0.0),
                                            Rotation2d.fromDegrees(nt_field_heading.getDouble(0.0)));
        final Pose2d robot_pos = drivetrain.getPose();
        final Pose2d robot_on_field = field_pos.plus(new Transform2d(robot_pos.getTranslation(), robot_pos.getRotation()));        
        field.setRobotPose(robot_on_field);
    }

    @Override
    public void teleopInit()
    {
        nt_field_x.setDouble(7.64);
        nt_field_y.setDouble(1.764);
        nt_field_heading.setDouble(90);
        drivetrain.reset();
        joydrive.schedule();
    }

    @Override
    public void teleopPeriodic()
    {
    }

    @Override
    public void autonomousInit()
    {
        auto_options.getSelected().schedule();
    }

    @Override
    public void autonomousPeriodic()
    {
    }
}
