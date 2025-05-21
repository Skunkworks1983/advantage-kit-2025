// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ConditionalSmartDashboard;
import frc.robot.utils.PIDControllers.SmartPIDControllerTalonFX;
import frc.robot.utils.constants.ClimberConstants;
import frc.robot.utils.constants.CurrentLimitConstants;

public class Climber extends SubsystemBase {
  // climber motor, Talon FX
  TalonFX climbMotor;

  // magnet sensors for the climber
  private DigitalInput magnetSensor1;
  private DigitalInput magnetSensor2;

  // smart pid code for the climber motor
  private SmartPIDControllerTalonFX climberSmartPID;
  // position voltage for climber
  private PositionVoltage positionVoltage = new PositionVoltage(0);
  // the base climber set point
  private double climberSetPointMeters = ClimberConstants.CLIMBER_MIN;

  public Climber() {
    // instantiates and sets the position of the climber motor
    climbMotor = new TalonFX(ClimberConstants.IDs.CLIMBER_KRAKEN_MOTOR);
    climbMotor.setPosition(0.0);

    // instantiates the climber magnet sensors
    magnetSensor1 = new DigitalInput(ClimberConstants.IDs.CLIMBER_MAGNET_SENSOR_1);
    magnetSensor2 = new DigitalInput(ClimberConstants.IDs.CLIMBER_MAGNET_SENSOR_2);

    // configs for climber
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits = CurrentLimitConstants.MINI_KRAKEN_CURRENT_LIMIT_CONFIG;
    climbMotor.getConfigurator().apply(config);

    // smart pid for climber
    climberSmartPID =
        new SmartPIDControllerTalonFX(
            ClimberConstants.PIDs.CLIMBER_KP,
            ClimberConstants.PIDs.CLIMBER_KI,
            ClimberConstants.PIDs.CLIMBER_KD,
            ClimberConstants.PIDs.CLIMBER_KF,
            "Climb Motor",
            ClimberConstants.CLIMBER_SMARTPID_ACTIVE,
            climbMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // smartdashboard for climber
    SmartDashboard.putNumber("Climber/Motor position", getHeight());
    ConditionalSmartDashboard.putBoolean("Climber/Motor Connected", isMotorConnected());
    SmartDashboard.putNumber("Climber/Motor Current", getClimberMotorCurrent());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 1", magnetSensor1Tripped());
    SmartDashboard.putBoolean("Climber/Magnet Sensor 2", magnetSensor2Tripped());
    ConditionalSmartDashboard.putNumber("Climber/Set Point", getSetPointMeters());
    ConditionalSmartDashboard.putBoolean("Climber/At Set Point", isAtSetpoint());
  }

  // Checks is first magnet sensor is activated. returns true if activated
  public boolean magnetSensor1Tripped() {
    return !magnetSensor1.get();
  }

  // Checks is second magnet sensor is activated. returns true if activated
  public boolean magnetSensor2Tripped() {
    return !magnetSensor2.get();
  }

  // returns height of climber in meters
  public double getHeight() {
    return climbMotor.getPosition().getValueAsDouble()
        * ClimberConstants.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT;
  }

  // Checks if the motor is connected
  public boolean isMotorConnected() {
    return climbMotor.isConnected();
  }

  // sets the climber to a set point defined in parameter. parameter is in meters.
  public void setClimberSetPointMeters(double newSetPointMeters) {
    climberSetPointMeters = newSetPointMeters;
    climbMotor.setControl(
        positionVoltage
            .withPosition(
                newSetPointMeters / ClimberConstants.CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT)
            .withEnableFOC(true));
    ConditionalSmartDashboard.putNumber("Motor position", getHeight());
  }

  // returns the last set point in meters
  public double getSetPointMeters() {
    ConditionalSmartDashboard.putNumber("climber set point", climberSetPointMeters);
    return climberSetPointMeters;
  }

  // returns true if the difference between the first 2 parameter is less than the last parameter
  public boolean approxEquals(double value1, double value2, double tolerance) {
    return Math.abs(value1 - value2) < tolerance;
  }

  // returns true if climber is at set point
  public boolean isAtSetpoint() {
    return approxEquals(getHeight(), climberSetPointMeters, ClimberConstants.CLIMBER_TOLERANCE);
  }

  // returns the climber current in amps
  public double getClimberMotorCurrent() {
    return climbMotor.getSupplyCurrent().getValueAsDouble();
  }

  // if the magnet sensors are true, go to parameter (position in meters)
  public Command goToPositionAfterMagnetSensor(double position) {
    return Commands.startEnd(
            () -> {
              // waitUntilMagnetSensorsAreTrue().finallyDo(
              setClimberSetPointMeters(position);
            },
            () -> {})
        .until(
            () -> {
              return isAtSetpoint();
            });
  }

  // ends if magnet sensors are true
  public Command waitUntilMagnetSensorsAreTrue() {
    return Commands.waitUntil(
        () -> {
          return magnetSensor1Tripped() && magnetSensor2Tripped();
        });
  }
}
