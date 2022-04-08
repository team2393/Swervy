// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;

/** Driver, part of a swerve module */
public class Driver
{
    // 2048 ticks per revolution, 6.67:1 gearing, 4 inch wheels
    private final static double COUNTS_PER_METER = 2048.0 * 6.67 / Units.inchesToMeters(Math.PI*4.0);
    private final WPI_TalonFX driver;
    private double offset = 0;
    
    /** @param id CAN ID */
    public Driver(final int id)
    {
        driver = new WPI_TalonFX(id);

        driver.configFactoryDefault();
        driver.clearStickyFaults();
        driver.configOpenloopRamp(0.5);
        driver.configClosedloopRamp(0.5);
        driver.configVoltageCompSaturation(10.0);
        driver.enableVoltageCompensation(true);

        // 1m/s = 4200 units/100ms = 0.19 output
        // kF = 1023 * 0.19 / 4200 = 0.0462
        // Then test in Phoenix tuner
        driver.config_kF(0, 0.054);
        driver.config_IntegralZone(0, 400);
        driver.config_kI(0, 0.0005);
    }

    public void reset()
    {
        offset = driver.getSelectedSensorPosition();
    }

    public double getPosition()
    {
        return (driver.getSelectedSensorPosition() - offset) / COUNTS_PER_METER;
    }

    public double getSpeed()
    {
        return driver.getSelectedSensorVelocity() / COUNTS_PER_METER * 10.0;
    }

    /** Drive open-loop
     *  @param output Desired drive output -1..1
     */
    public void setOutput(final double output)
    {
        driver.set(TalonFXControlMode.PercentOutput, output);
    }

    /** Drive closed-loop
     *  @param speed Desired speed [m/s]
     */
    public void setSpeed(final double speed)
    {
        final double velo = speed * COUNTS_PER_METER / 10.0;
        driver.set(TalonFXControlMode.Velocity, velo);
    }
}
