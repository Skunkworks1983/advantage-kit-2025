// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.movecommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.MoveElevatorToSetpointCommand;
import frc.robot.commands.wrist.MoveWristToSetpoint;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.utils.constants.EndEffectorSetpointConstants;

public class Coral2 extends SequentialCommandGroup {
  
  Elevator elevator;
  Wrist wrist;
  // sequental command group to move the wrist and elevator to set point to remove point of error from pathplanner
  public Coral2() {
    addCommands(
      new MoveElevatorToSetpointCommand(elevator, 
            EndEffectorSetpointConstants.EndEffectorSetpoints.CORAL_L2.elevatorSetpoint),
      new MoveWristToSetpoint(wrist, EndEffectorSetpointConstants.EndEffectorSetpoints.CORAL_L2.wristSetpoint)
      );
  }
}
