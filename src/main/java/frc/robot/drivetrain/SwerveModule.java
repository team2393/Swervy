// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** One swerve module
 * 
 *  Supports controlling the speed and heading
 *  of one module.
 */
public class SwerveModule
{
    private final Translation2d location;
    private final Rotator rotator;
    private final Driver driver;

    private SwerveModuleState desired_state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    private final NetworkTableEntry nt_zero, nt_distance, nt_speed, nt_heading;
    
    /** @param name Module name, used for network table entries
     *  @param location Location relative to center of robot
     *  @param rotator_channel Channel used for analog encoder and rotation PWM [0-3]
     *  @param zero_heading Zero heading of rotator
     *  @param driver_id Driver CAN ID [1-4]
     */
    public SwerveModule(final String name, final Translation2d location,
                        final int rotator_channel, final double zero_heading,
                        final int driver_id)
    {
        this.location = location;
        rotator = new Rotator(rotator_channel, zero_heading);
        driver = new Driver(driver_id);

        nt_zero = SmartDashboard.getEntry(name + "_zero");
        nt_zero.setDefaultNumber(zero_heading);
        nt_distance = SmartDashboard.getEntry(name + "_distance");
        nt_speed = SmartDashboard.getEntry(name + "_speed");
        nt_heading = SmartDashboard.getEntry(name + "_heading");
    }

    /** @return Location relative to center of robot */
    public Translation2d getLocation()
    {
        return location;
    }    
    
    private double getSpeed()
    {
        if (RobotBase.isSimulation())
            return desired_state.speedMetersPerSecond;
        else
            return driver.getSpeed();
    }

    /** @return Zeroed heading -180..+180 degrees */
    public Rotation2d getHeading()
    {
        if (RobotBase.isSimulation())
            return desired_state.angle;
        else
            return rotator.getHeading();
    }

    /** @return Current speed and heading */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getSpeed(), getHeading());
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
        rotator.setZero(nt_zero.getDouble(0.0));

        if (RobotBase.isSimulation())
        {
            nt_speed.setDouble(desired_state.speedMetersPerSecond);
            // Not simulating driver.getDistance()...
        }
        else
        {
            final SwerveModuleState optimized = SwerveModuleState.optimize(state, getHeading());
            rotator.setHeading(optimized.angle.getDegrees());
            driver.setSpeed(optimized.speedMetersPerSecond);
            nt_speed.setDouble(driver.getSpeed());
            nt_distance.setDouble(driver.getDistance());
        }
        nt_heading.setDouble(getHeading().getDegrees());
    }
}
