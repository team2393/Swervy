// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase
{
    /** Distance of each module from center of robot */
    private static final double module_distance = Math.sqrt(0.5*0.5 + 0.5*0.5);

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

    public Drivetrain()
    {
        System.out.println("Module locations:");
        for (int i=0; i<N; ++i)
            System.out.format("#%d: X=%5.2f m, Y=%5.2f m\n",
                              i,
                            modules[i].getLocation().getX(),
                            modules[i].getLocation().getY());
    }

    public void drive(final double vx, final double vy, final double vr)
    {
        final SwerveModuleState states[] = kinematics.toSwerveModuleStates(new ChassisSpeeds(vx, vy, Math.toRadians(vr)));
        for (int i=0; i<N; ++i)
            modules[i].drive(states[i].speedMetersPerSecond, states[i].angle.getDegrees());
    }

    @Override
    public void periodic()
    {
        final double gyro = 0; // TODO
        odometry.update(Rotation2d.fromDegrees(gyro),
                        modules[0].getState(),
                        modules[1].getState(),
                        modules[2].getState(),
                        modules[3].getState());
    }

    public Pose2d getPose()
    {
        return odometry.getPoseMeters();
    }

    public void reset(double x, double y, int heading)
    {
        final double gyro = 0; // TODO
        odometry.resetPosition(new Pose2d(x, y, Rotation2d.fromDegrees(heading)),
                               Rotation2d.fromDegrees(gyro));
    }
}
