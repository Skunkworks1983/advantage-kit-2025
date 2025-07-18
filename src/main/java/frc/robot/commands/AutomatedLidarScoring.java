// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.collector.Collector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.constants.EndEffectorSetpointConstants;
import frc.robot.utils.constants.EndEffectorToSetpointConstants;

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
      Collector collector,
      DoubleSupplier getXMetersPerSecond,
      DoubleSupplier getYMetersPerSecond,
      Supplier<EndEffectorToSetpointConstants> endEffectorSetpoint,
      boolean goingRight,
      double alignSpeed,
      BooleanSupplier expelButton) {
    addCommands(
        drivebase.getSwerveAlignCoral(
            getXMetersPerSecond, getYMetersPerSecond, goingRight, alignSpeed),
        Commands.waitUntil(
            () ->
                ((endEffectorSetpoint.get() == EndEffectorSetpointConstants.CORAL_L2)
                        || (endEffectorSetpoint.get() == EndEffectorSetpointConstants.CORAL_L3)
                            || (endEffectorSetpoint.get() == EndEffectorSetpointConstants.CORAL_L4))
                    && expelButton.getAsBoolean()),
        Commands.waitSeconds(0.1),
        collector.expelCoralCommand(
            true,
            endEffectorSetpoint).withTimeout(2)
        );
  }
}
