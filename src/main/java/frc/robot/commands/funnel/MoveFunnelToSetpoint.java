// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.funnel;

import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.constants.FunnelConstants;

public class MoveFunnelToSetpoint extends Command {
  frc.robot.subsystems.funnel.Funnel funnel;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State startPosition;
  double setPoint;
  double newSetPoint;

  final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(100, 500));

  Timer timePassed;

  public MoveFunnelToSetpoint(frc.robot.subsystems.funnel.Funnel funnel, double setPoint) {
    this.funnel = funnel;
    this.setPoint = setPoint;

    addRequirements(funnel);

    timePassed = new Timer();
    timePassed.stop();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DataLogManager.log("Initialized funnel move to position command");
    timePassed.reset();
    timePassed.start();

    goal = new TrapezoidProfile.State(setPoint, 0);
    positionVoltage = new PositionVoltage(0);
    startPosition = new TrapezoidProfile.State(funnel.getPos(), funnel.getVelocity());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    State positionGoal = profile.calculate(timePassed.get(), startPosition, goal);
    ConditionalSmartDashboard.putNumber("Funnel/Desired position", positionGoal.position);
    ConditionalSmartDashboard.putNumber("Funnel/Desired velocity", positionGoal.velocity);
    funnel.setFunnelSetPoint(positionGoal.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("End of funnel move to position command");
  }

  @Override
  public boolean isFinished() {
    return Math.abs(funnel.getPos() - goal.position) < FunnelConstants.FUNNEL_TOLERANCE;
  }
}
