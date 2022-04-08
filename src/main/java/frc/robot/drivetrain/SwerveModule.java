// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limits;

/** One swerve module
 * 
 *  Supports controlling the speed and heading
 *  of one module.
 */
public class SwerveModule
{
    private final Translation2d location;
    private final AnalogInput encoder;
    private SparkMini rotator;

    private double zero = 0.0;
    private SwerveModuleState desired_state = new SwerveModuleState(0, Rotation2d.fromDegrees(0));

    private final NetworkTableEntry nt_speed, nt_heading;
    
    private final ProfiledPIDController heading_pid = new ProfiledPIDController(0.5, 0, 0,
        new TrapezoidProfile.Constraints(Limits.MAX_ROTATION, Limits.MAX_ROTATION));

    /** @param name Module name, used for network table entries
     *  @param location Location relative to center of robot
     *  @param channel Channel used for analog encoder, rotation PWM and drive CAN
     */
    public SwerveModule(final String name, final Translation2d location,
                  final int channel)
    {
        this.location = location;
        encoder = new AnalogInput(channel);
        rotator = new SparkMini(channel);

        heading_pid.enableContinuousInput(-180.0, 180.0);

        nt_speed = SmartDashboard.getEntry(name + "_speed");
        nt_heading = SmartDashboard.getEntry(name + "_heading");
    }

    public void reset()
    {
        zero = getRawHeading();
        heading_pid.reset(0.0);
    }

    public void configureHeadingPID(double kp, double ki, double kd)
    {
        heading_pid.setPID(kp, ki, kd);
    }

    private double getSpeed()
    {
        if (RobotBase.isSimulation())
            return desired_state.speedMetersPerSecond;
        else
            return 0.0; // TODO
    }

    /** @return Heading 0..360 degrees, not zeroed */
    private double getRawHeading()
    {
        // Raw value should be 0..5V, scale relative to actual 5V rail
        final double raw = encoder.getVoltage() / RobotController.getVoltage5V();
        return raw * 360.0;
    }

    /** @return Zeroed heading -180..+180 degrees */
    public Rotation2d getHeading()
    {
        if (RobotBase.isSimulation())
            return desired_state.angle;
        else
            // Map to -180..180 degrees
            return Rotation2d.fromDegrees(Math.IEEEremainder(getRawHeading() - zero, 360.0));
    }

    /** @return Location relative to center of robot */
    public Translation2d getLocation()
    {
        return location;
    }
    
    /** @return Current speed and heading */
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getSpeed(), getHeading());
    }

    /** Drive module open-loop
     *  @param speed Speed (-1..1), positive is "Forward"
     *  @param rotation Rotation (-1..1), positive is "counter clockwise"
     */
    public void driveDirect(final double speed, final double rotation)
    {
        // TODO speed
        rotator.set(rotation);
        nt_heading.setDouble(getHeading().getDegrees());
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

        // TODO SwerveModuleState.optimize(state, currentAngle)
        
        if (RobotBase.isSimulation())
        {
            nt_speed.setDouble(desired_state.speedMetersPerSecond);
        }
        else
        {
            double voltage = heading_pid.calculate(getHeading().getDegrees(), state.angle.getDegrees());
            voltage = MathUtil.clamp(voltage, -15, 5.0);
            rotator.setVoltage(voltage);
        }
        nt_heading.setDouble(getHeading().getDegrees());
    }
}
