// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;

public class TrapezoidProfileDriveStraight extends Command {

  Drive drivebase;
  Timer timeElasped = new Timer();
  TrapezoidProfile profile = new TrapezoidProfile(new Constraints(2.0, 1.0));
  State startState = new State();
  State goalState;
  boolean allianceFlip;

  public TrapezoidProfileDriveStraight(Drive drivebase, double endPos, boolean allianceFlip) {
    this.drivebase = drivebase;
    goalState = new State(endPos, 0.0);
    this.allianceFlip = allianceFlip;
    addRequirements(drivebase);
  }

  double cachedHeadingForCommand;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cachedHeadingForCommand = drivebase.getPose().getRotation().getDegrees();

    if (allianceFlip) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
        drivebase.setPose(
            new Pose2d(
                drivebase.getPose().getMeasureX(),
                drivebase.getPose().getMeasureY(),
                Rotation2d.fromDegrees(180)));
        cachedHeadingForCommand = 180;
      } else {
        drivebase.setPose(
            new Pose2d(
                drivebase.getPose().getMeasureX(),
                drivebase.getPose().getMeasureY(),
                Rotation2d.fromDegrees(0)));
        cachedHeadingForCommand = 0;
      }
    }

    timeElasped.reset();
    timeElasped.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = profile.calculate(timeElasped.get(), startState, goalState).velocity;
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            xVelocity, 0.0, drivebase.calculateWithHeadingController(cachedHeadingForCommand));
    drivebase.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeElasped.get() > profile.totalTime();
  }
}
