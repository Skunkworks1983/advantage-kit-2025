// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.DrivebaseConstants;

public class RotateToHeadingOffset extends Command {

  Drive drivebase;
  double headingOffset;
  PIDController rotController;
  double actualSetpoint;

  public RotateToHeadingOffset(Drive drivebase, Rotation2d headingOffset) {
    this.drivebase = drivebase;
    this.headingOffset = headingOffset.getDegrees();
    rotController =
        new PIDController(
            DrivebaseConstants.PIDs.HEADING_CONTROL_kP,
            DrivebaseConstants.PIDs.HEADING_CONTROL_kI,
            DrivebaseConstants.PIDs.HEADING_CONTROL_kD);

    // Possibly 0, 360
    rotController.enableContinuousInput(-180, 180); // The PID controller is in degrees\
    rotController.setTolerance(1.0);
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.actualSetpoint = (drivebase.getPose().getRotation().getDegrees() + this.headingOffset);
    rotController.setSetpoint(drivebase.getPose().getRotation().getDegrees() + this.headingOffset);
  }

  @Override
  public void execute() {
    drivebase.runVelocity(
        new ChassisSpeeds(
            0.0,
            0.0,
            rotController.calculate(
                drivebase.getPose().getRotation().getDegrees(), this.actualSetpoint)));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return rotController.atSetpoint();
  }
}
