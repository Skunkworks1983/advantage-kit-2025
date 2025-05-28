// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.tests;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.constants.TestingConstants.Testing;
import java.util.function.DoubleSupplier;

public class JoystickElevatorVelocity extends Command {
  DoubleSupplier elevatorPositionChange;

  frc.robot.subsystems.elevator.Elevator elevator;

  public JoystickElevatorVelocity(
      frc.robot.subsystems.elevator.Elevator elevator, DoubleSupplier elevatorPositionChange) {
    addRequirements(elevator);
    this.elevatorPositionChange = elevatorPositionChange;
    this.elevator = elevator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double elevatorVelocity = elevatorPositionChange.getAsDouble() * Testing.ELEVATOR_MAX_SPEED;

    elevator.setSpeeds(elevatorVelocity);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
