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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Coral3 extends SequentialCommandGroup {

  Elevator elevator;
  Wrist wrist;
  /** Creates a new Coral1. */
  public Coral3() {
    addCommands(
      new MoveElevatorToSetpointCommand(elevator, 
            EndEffectorSetpointConstants.EndEffectorSetpoints.CORAL_L3.elevatorSetpoint),
      new MoveWristToSetpoint(wrist, EndEffectorSetpointConstants.EndEffectorSetpoints.CORAL_L3.wristSetpoint)
      );
  }
}
