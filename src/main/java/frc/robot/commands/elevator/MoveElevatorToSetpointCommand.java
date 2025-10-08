// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.utils.constants.ElevatorConstants;

// This command moves to the elevator from its current position to the argument
// targetHeight (meters). This command will end when the elevator is within some
// tolorence of the desired position. This command uses the trapazoid motion profile.
// This command is long and requires a fair amount of state so it is not defined within the
// Elevator subsystem.
public class MoveElevatorToSetpointCommand extends Command {
  Timer timeElapsed;
  State startState;
  State targetState;
  Elevator elevator;
  boolean isGoingUp;

  private static final TrapezoidProfile motionProfile =
      new TrapezoidProfile(
          new Constraints(
              ElevatorConstants.Profile.MAX_VELOCITY, ElevatorConstants.Profile.MAX_ACCELERATION));

  public MoveElevatorToSetpointCommand(Elevator elevator, double targetHeight) {
    targetState = new State(targetHeight * ElevatorConstants.METERS_TO_MOTOR_ROTATIONS, 0.0);
    this.elevator = elevator;
    timeElapsed = new Timer();
    timeElapsed.stop();
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    startState =
        new State(
            elevator.getElevatorPosition() * ElevatorConstants.METERS_TO_MOTOR_ROTATIONS, 0.0);
    isGoingUp =
        targetState.position
            > elevator.getElevatorPosition() * ElevatorConstants.METERS_TO_MOTOR_ROTATIONS;
    elevator.setFinalPosition(targetState.position);
    timeElapsed.reset();
    timeElapsed.start();
    System.out.println("Move elevator to Setpoint Command Initialize");
  }

  @Override
  public void execute() {

    State newState =
        motionProfile.calculate(
            timeElapsed.get(), // Time is the only variable that changes throughout each run
            startState,
            targetState);

    elevator.setMotorTrapezoidProfileSafe(newState.position, newState.velocity);
  }

  @Override
  public void end(boolean interrupted) {
    if (elevator.getBottomLimitSwitch() && !isGoingUp) {
      elevator.setMotorTrapezoidProfileSafe(0.0, 0.0);
    }
    System.out.println("Move elevator to Setpoint Command End");
  }

  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber(
        "Abs elevotor POS: ", Math.abs(targetState.position - elevator.getElevatorPosition()));
    return (ElevatorConstants.TOLERENCE_METERS_FOR_MOVE_TO_POSITION
            > Math.abs(targetState.position - elevator.getElevatorPosition()))
        || ((elevator.getBottomLimitSwitch() && !isGoingUp)
        // || (elevator.getTopLimitSwitch() && isGoingUp)
        );
  }
}
