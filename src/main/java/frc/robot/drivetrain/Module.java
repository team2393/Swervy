// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** One swerve module
 * 
 *  Supports controlling the speed and heading
 *  of one module.
 */
public class Module
{
    private final Translation2d location;

    private SwerveModuleState desired_state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    private NetworkTableEntry nt_speed, nt_heading;
    
    /** @param name Module name, used for network table entries
     *  @param location Location relative to center of robot
     */
    public Module(final String name, final Translation2d location)
    {
        this.location = location;

        nt_speed = SmartDashboard.getEntry(name + "_speed");
        nt_heading = SmartDashboard.getEntry(name + "_heading");
    }

    /** @return Location relative to center of robot */
    public Translation2d getLocation()
    {
        return location;
    }
    
    /** @return Current speed and heading */
    public SwerveModuleState getState()
    {
        return desired_state;
    }

    /** Drive module with desired speed and heading
     *  @param speed Speed of movement [m/s]
     *  @param heading Direction of movement [degrees]
     */
    public void drive(final double speed, final double heading)
    {
        drive(new SwerveModuleState(speed, Rotation2d.fromDegrees(heading)));
    }

    /** Drive module with desired speed and heading
     *  @param state Speed and heading
     */
    public void drive(final SwerveModuleState state)
    {
        desired_state = state;

        nt_speed.setDouble(desired_state.speedMetersPerSecond);
        nt_heading.setDouble(desired_state.angle.getDegrees());
    }
}
