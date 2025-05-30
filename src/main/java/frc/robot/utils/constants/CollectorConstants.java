// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.constants;

/** Add your docs here. */
public class CollectorConstants {
  public class IDs {
    public static final int RIGHT_MOTOR = 42; // 42 is the real id
    public static final int LEFT_MOTOR = 11;
    public static final int DIGITAL_INPUT_CHANNEL = 1;
  }

  public class Speeds {
    public static final double CORAL_INTAKE_SLOW_SPEED = 8.0; // meters per sec
    public static final double CORAL_INTAKE_FAST_SPEED = 18.0; // meters per sec
    public static final double CORAL_EXPEL_SLOW_SPEED = 3.0; // meters per sec
    public static final double CORAL_EXPEL_FAST_SPEED = 10.0; // meters per sec
    public static final double ALGAE_INTAKE_SPEED_SLOW = 0.25; // throttle pct output
    public static final double ALGAE_INTAKE_SPEED_FAST = 0.5;
    public static final double ALGAE_EXPEL_SPEED = -20.0; // meters per sec

    public static final double SPEED_MULIPILER_LEFT = 0.75;
  }

  public static final double COLLECTOR_ROTATIONS_PER_METER = 0.0762 * Math.PI;
  public static final double END_COUNT_TICK_COUNTER_ALGAE = 3;
  public static final double END_COUNT_TICK_COUNTER_CORAL = 2.0;
  public static final double COLLECTOR_AMPS_BEFORE_CUTTOF = 5.0;
  public static final double ALGAE_AMP_CUT_OFF = 10.0;

  public static final boolean SMART_PID_ENABLED = false;

  public class PositionControlMode {
    public static final double KP = 5.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KF = 0.0;
    public static final double KV = 1.3;
    public static final double KA = 0.0;
    public static final double KS = 0.0;
  }

  public class VelocityControlMode {
    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;
    public static final double KF = 0.0;
    public static final double KV = 1.3;
    public static final double KA = 0.0;
    public static final double KS = 0.0;
  }
}
