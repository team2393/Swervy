// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Limits;

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
    private final Module[] modules = new Module[]
    {
        new Module("Swerve0", new Translation2d(module_distance, Rotation2d.fromDegrees(45))),
        new Module("Swerve1", new Translation2d(module_distance, Rotation2d.fromDegrees(-45))),
        new Module("Swerve2", new Translation2d(module_distance, Rotation2d.fromDegrees(-90-45))),
        new Module("Swerve3", new Translation2d(module_distance, Rotation2d.fromDegrees(90+45)))
    };

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
            System.out.format("#%d: X=%5.2f m, Y=%5.2f m\n",
                              i,
                            modules[i].getLocation().getX(),
                            modules[i].getLocation().getY());
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
        Translation2d pivot;
        if (pivot_module < 0)
            pivot = center;
        else
            pivot = modules[pivot_module].getLocation();
        final SwerveModuleState states[] = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, Math.toRadians(vr)),
                                                                           pivot);
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
        final ProfiledPIDController anglepid = new ProfiledPIDController(10, 0, 0,
                new TrapezoidProfile.Constraints(Math.toRadians(Limits.MAX_ROTATION),
                                                 Math.toRadians(Limits.MAX_ROTATION)));
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
