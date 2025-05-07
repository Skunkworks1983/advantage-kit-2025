// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.wrist;

import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.constants.WristConstants;
import frc.robot.utils.ConditionalSmartDashboard;


public class MoveWristToSetpoint extends Command {
  frc.robot.subsystems.wrist.Wrist wrist;

  PositionVoltage positionVoltage;
  TrapezoidProfile.State goal;
  TrapezoidProfile.State startPosition;
  double setPoint;

  final TrapezoidProfile profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(frc.robot.utils.constants.WristConstants.Wrist.WRIST_MAX_VELOCITY, frc.robot.utils.constants.WristConstants.Wrist.WRIST_MAX_ACCELERATION));
  
  Timer timePassed;

  public MoveWristToSetpoint(frc.robot.subsystems.wrist.Wrist wrist, double setPoint) {
    this.setPoint = setPoint;
    this.wrist = wrist;

    addRequirements(wrist);

    timePassed = new Timer();
    timePassed.stop();
  }
  
  @Override
  public void initialize() {
    DataLogManager.log("MoveWristToSetpoint command initialized");

    timePassed.reset(); 
    timePassed.start();

    double newSetPoint = setPoint;
    

    goal = new TrapezoidProfile.State(newSetPoint,0);
    positionVoltage = new PositionVoltage(0);
    startPosition = new TrapezoidProfile.State(wrist.getPosition(),wrist.getWristVelocity());
  }

  @Override
  public void execute() {
    State positionGoal = profile.calculate(timePassed.get(), startPosition, goal);
    positionVoltage.Position = positionGoal.position * frc.robot.utils.constants.WristConstants.Wrist.WRIST_GEAR_RATIO;
    wrist.setWristMotorControl(positionVoltage);

    ConditionalSmartDashboard.putNumber("Wrist position goal (motor rotations)", positionGoal.position);
    ConditionalSmartDashboard.putNumber("Wrist velocity goal (motor rotations per second)", positionGoal.velocity);
  }
  
  @Override
  public void end(boolean interrupted) {
    DataLogManager.log("MoveWristToSetpoint command ended");
  }
  
  @Override
  public boolean isFinished() {
    return (Math.abs(wrist.getPosition() - setPoint)) < frc.robot.utils.constants.WristConstants.Wrist.WRIST_TOLERANCE;
  }
}
