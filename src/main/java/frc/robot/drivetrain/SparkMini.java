// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.PWM;

/** REV SparkMini motor controller */
public class SparkMini extends PWMMotorController
{
    public SparkMini(final int channel)
    {
        super("SparkMini", channel);
        // Spark Mini manual describes PWM range of 500 .. 2500 us,
        // but 0.500 ms as lower end results in either runtime error
        // "HAL: The PWM Scale Factors are out of range",
        // and/or a setting of -1 for speed gets converted to +1,
        // never reaching full reverse speed.
        // A low end of 0.503 seems to avoid these problems
        // and set(-1) gets full reverse speed.
        m_pwm.setBounds(2.500, 1.55, 1.50, 1.45, 0.503);
        m_pwm.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
        m_pwm.setSpeed(0.0);
        m_pwm.setZeroLatch();
        m_pwm.enableDeadbandElimination(false);
    }
}
