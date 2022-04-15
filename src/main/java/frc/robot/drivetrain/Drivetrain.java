// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Settings;

/** Drivetrain, made up of swerve modules */
public class Drivetrain extends SubsystemBase
{
    /** Distance of each module from center of robot */
    private static final double module_distance = Math.sqrt(0.5*0.5 + 0.5*0.5);

    private static final Translation2d center = new Translation2d();

    /** Number of swerve modules */
    private static final int N = 4;

    /** Swerve modules #0, 1, 2, 3 in 4 corners
     * 
     *      Y
     *     /|\
     *      |
     *      |
     * 3    |    0
     *      |
     * -----*--------->  X
     *      |
     * 2    |    1
     */
    private final SwerveModule[] modules = new SwerveModule[]
    {
        new SwerveModule("Swerve0", new Translation2d(module_distance, Rotation2d.fromDegrees(45)),     0, Settings.MODULE_ZERO[0],  1),
        new SwerveModule("Swerve1", new Translation2d(module_distance, Rotation2d.fromDegrees(-45)),    1, Settings.MODULE_ZERO[1],  2),
        new SwerveModule("Swerve2", new Translation2d(module_distance, Rotation2d.fromDegrees(-90-45)), 2, Settings.MODULE_ZERO[2],  3),
        new SwerveModule("Swerve3", new Translation2d(module_distance, Rotation2d.fromDegrees(90+45)),  3, Settings.MODULE_ZERO[3],  4)
    };

    private final NetworkTableEntry[] nt_heading = new NetworkTableEntry[N];

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modules[0].getLocation(),
                                                                               modules[1].getLocation(),
                                                                               modules[2].getLocation(),
                                                                               modules[3].getLocation());

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(0), new Pose2d());

    /** Simulated heading [degrees] */
    private double sim_heading = 0.0;

    public Drivetrain()
    {
        System.out.println("Module locations:");
        for (int i=0; i<N; ++i)
        {
            System.out.format("#%d: X=%5.2f m, Y=%5.2f m\n",
                              i,
                              modules[i].getLocation().getX(),
                              modules[i].getLocation().getY());
            nt_heading[i] = SmartDashboard.getEntry("Heading" + i);
        }
        

        // By default, tell motors to stay put
        setDefaultCommand(new IdleCommand(this));
    }

    /** Drive!
     * 
     *  @param vx X (forward/back) speed [m/s]
     *  @param vy Y (left/right) speed [m/s]
     *  @param vr Rotation (counter-clockwise) speed [degrees/s]
     * @param pivot_module Module around which to rotate. -1 for center of robot
     */
    public void drive(final double vx, final double vy, final double vr,
                      final int pivot_module)
    {
        // TODO Offer variant for drive-by-joystick that uses coordinates relative to robot
        // For example, change this to accept ChassisSpeeds and then call with
        // ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, vr, getHeading)

        Translation2d pivot;
        if (pivot_module < 0)
            pivot = center;
        else
            pivot = modules[pivot_module].getLocation();

        final SwerveModuleState states[] = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, Math.toRadians(vr)),
                                                                           pivot);
                                                                           
        // TODO SwerveDriveKinematics.desaturateWheelSpeeds(states, some_max_speed);
        for (int i=0; i<N; ++i)
            modules[i].drive(states[i].speedMetersPerSecond, states[i].angle.getDegrees());
        
        if (RobotBase.isSimulation())
            sim_heading += vr * TimedRobot.kDefaultPeriod;
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    /** Reset position
     *  @param x Field position X [m]
     *  @param y Field position Y [m]
     *  @param heading Field heading [degrees]
     */
    public void reset(final double x, final double y, final double heading)
    {
        sim_heading = heading;
        odometry.resetPosition(new Pose2d(x, y, Rotation2d.fromDegrees(heading)),
                               Rotation2d.fromDegrees(sim_heading));
    }
    
    @Override
    public void periodic()
    {
        // Update estimated position from module states and gyro
        // TODO Read from gyro
        final double heading;
        if (RobotBase.isSimulation())
            heading = sim_heading;
        else
            heading = 0.0;
        odometry.update(Rotation2d.fromDegrees(heading),
                        modules[0].getState(),
                        modules[1].getState(),
                        modules[2].getState(),
                        modules[3].getState());
        
        for (int i=0; i<N; ++i)
            nt_heading[i].setDouble(modules[i].getHeading().getDegrees());
    }

    /** Create command that follows a trajectory
     *  @param robot_heading Desired robot heading at endppoint [degrees]
     *  @param xyh Waypoints X, Y, Heading
     *  @return Command that follows the trajectory
     */
    public CommandBase createFollower(final double robot_heading,
                                      final double... xyh)
    {        
        // Max speed used for the created trajectory
        final TrajectoryConfig config = new TrajectoryConfig(Settings.MAX_SPEED, 2*Settings.MAX_SPEED);

        if (xyh.length % 3 != 0)
            throw new IllegalArgumentException("Expected X, Y, Heading[], got " + xyh.length + " elements");
        final List<Pose2d> waypoints = new ArrayList<>();
        for (int i=0; i<xyh.length; i+=3)
            waypoints.add(new Pose2d(xyh[i], xyh[i+1], Rotation2d.fromDegrees(xyh[i+2])));
        final Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
        return createFollower(robot_heading, trajectory);
    }

    /** Create command that follows a trajectory
     *  @param robot_heading Desired robot heading at endppoint [degrees]
     *  @param trajectory Trajectory to follow
     *  @return Command that follows the trajectory
     */
    public CommandBase createFollower(final double robot_heading, final Trajectory trajectory)
    {
        final Rotation2d heading_rad = Rotation2d.fromDegrees(robot_heading);
        
        // PID controllers for swerving in X, Y and rotation
        // TODO Tune
        final PIDController xpid = new PIDController(1, 0, 0);
        final PIDController ypid = new PIDController(1, 0, 0);
        // angle controller uses radians
        final ProfiledPIDController anglepid = new ProfiledPIDController(1, 0, 0,
                new TrapezoidProfile.Constraints(Math.toRadians(Settings.MAX_ROTATION),
                                                 Math.toRadians(Settings.MAX_ROTATION)));
        anglepid.enableContinuousInput(-Math.PI, Math.PI);

        // Called by SwerveControllerCommand with desired swerve module states
        final Consumer<SwerveModuleState[]> update_modules = states ->
        {
            // Pass on to respective module
            for (int i=0; i<N; ++i)
                modules[i].drive(states[i]);
            // Simulate rotation
            if (RobotBase.isSimulation())
                sim_heading += Math.toDegrees(kinematics.toChassisSpeeds(states).omegaRadiansPerSecond) * TimedRobot.kDefaultPeriod;
        };
        return new SwerveControllerCommand(trajectory,
           this::getPose,
           kinematics, xpid, ypid, anglepid, () -> heading_rad, update_modules, this);
   }
}
