// Copyright (c) FIRST Team 2393 and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.OperatorInterface;

/** Stay put */
public class DriveByJoystickCommand extends RunCommand
{
    public DriveByJoystickCommand(final Drivetrain drivetrain)
    {
        super(() ->
        {
            int pivot;
            if (OperatorInterface.pivotLeft())
                pivot = 0;
            else if (OperatorInterface.pivotRight())
                pivot = 1;
            else
                pivot = -1;
            drivetrain.drive(OperatorInterface.getForwardBackward(),
                             OperatorInterface.getLeftRight(),
                             OperatorInterface.getRotation(),                          
                             pivot);
            }, drivetrain);
    }
}
