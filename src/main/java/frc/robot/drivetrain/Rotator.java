// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Settings;

/** Rotator, part of a swerve module */
public class Rotator
{
    private final AnalogInput encoder;
    private final SparkMini rotator;
    private final ProfiledPIDController heading_pid = new ProfiledPIDController(0.5, 0, 0,
        new TrapezoidProfile.Constraints(Settings.MAX_ROTATION, 2*Settings.MAX_ROTATION));

    /** Heading that's defined as "zero"
     * 
     *  In simulation, this is the desired heading
     */
    private double zero = 0.0;
    
    /** @param channel Channel used for analog encoder and rotation PWM
     *  @param zero Zero heading [degrees]
     */
    public Rotator(final int channel, final double zero)
    {
        encoder = new AnalogInput(channel);
        rotator = new SparkMini(channel);
        heading_pid.enableContinuousInput(-180.0, 180.0);
        this.zero = zero;
    }

    public void reset()
    {
        zero = getRawHeading();
        heading_pid.reset(0.0);
    }

    public void configureHeadingPID(final double kp, final double ki, final double kd)
    {
        heading_pid.setPID(kp, ki, kd);
    }

    /** @return Heading 0..360 degrees, not zeroed */
    public double getRawHeading()
    {
        // Raw value should be 0..5V, scale relative to actual 5V rail
        final double raw = encoder.getVoltage() / RobotController.getVoltage5V();
        return raw * 360.0;
    }

    /** @return Zeroed heading -180..+180 degrees */
    public Rotation2d getHeading()
    {
        if (RobotBase.isSimulation())
            return Rotation2d.fromDegrees(zero);
        else
            // Map to -180..180 degrees
            return Rotation2d.fromDegrees(Math.IEEEremainder(getRawHeading() - zero, 360.0));
    }

    /** Drive open-loop
     *  @param rotation Rotation (-1..1), positive is "counter clockwise"
     */
    public void setRotation(final double rotation)
    {
        rotator.set(rotation);
    }

    /** Drive closed-loop
     *  @param heading Desired heading [degrees]
     */
    public void setHeading(final double heading)
    {
        if (RobotBase.isSimulation())
            zero = heading;
        else
        {
            double voltage = heading_pid.calculate(getHeading().getDegrees(), heading);
            voltage = MathUtil.clamp(voltage, -Settings.MAX_ROTATION_VOLTAGE, Settings.MAX_ROTATION_VOLTAGE);
            rotator.setVoltage(voltage);
        }
    }
}
