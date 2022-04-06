// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.RunCommand;

/** Stay put */
public class IdleCommand extends RunCommand
{
    public IdleCommand(final Drivetrain drivetrain)
    {
        super(() ->
        {
            drivetrain.drive(0, 0, 0, -1);
        }, drivetrain);
    }
}
