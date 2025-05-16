// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.EndEffectorSetpointConstants;
import frc.robot.utils.EndEffectorSetpoints;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomatedLidarScoring extends SequentialCommandGroup {
  /** Creates a new AutomatedLidarScoring. */
  public AutomatedLidarScoring(
      Drive drivebase,
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      Supplier<EndEffectorSetpointConstants> endEffectorSetpoint,
      boolean goingRight,
      double alignSpeed,
      BooleanSupplier expelButton) {
    addCommands(
        drivebase.getSwerveAlignCoral(
            getXMetersPerSecond, getYMetersPerSecond, goingRight, alignSpeed),
        Commands.waitUntil(
            () ->
                ((endEffectorSetpoint.get() == EndEffectorSetpoints.CORAL_L2)
                        || (endEffectorSetpoint.get() == EndEffectorSetpoints.CORAL_L3))
                    && expelButton.getAsBoolean()),
        Commands.waitSeconds(0.1)
        // TODO, add expell command here
        );
  }
}
