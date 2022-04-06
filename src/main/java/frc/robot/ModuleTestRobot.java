// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.drivetrain.SparkMini;

/** Robot for testing single swerve module */
public class ModuleTestRobot extends TimedRobot
{
    private final SparkMini rotator = new SparkMini(0);
    private final WPI_TalonFX runner = new WPI_TalonFX(1);
    private final AnalogEncoder encoder = new AnalogEncoder(0);

    @Override
    public void robotInit()
    {
        System.out.println("********************************");
        System.out.println("********************************");
        System.out.println("** " + getClass().getName());
        System.out.println("********************************");
        System.out.println("********************************");

        runner.configFactoryDefault();
        runner.clearStickyFaults();
        runner.configOpenloopRamp(1.0);
    }

    @Override
    public void robotPeriodic()
    {
        SmartDashboard.putNumber("Position", encoder.getAbsolutePosition());
        SmartDashboard.putNumber("Rotations", encoder.get());
    }

    @Override
    public void teleopInit()
    {
    }

    @Override
    public void teleopPeriodic()
    {
        // "Forward" for positive
        runner.set(MathUtil.applyDeadband(-OperatorInterface.joystick.getRightY(),
                                          0.05));
     
        rotator.set(MathUtil.applyDeadband(OperatorInterface.joystick.getLeftX(),
                                           0.05));
    }
}
