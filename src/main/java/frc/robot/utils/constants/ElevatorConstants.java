package frc.robot.utils.constants;

public final class ElevatorConstants {

  // For determining right and left, look at the elevator from the side paralel to the one that
  // the elevator is on
  public static final int MOTOR_RIGHT_ID = 27;
  public static final int MOTOR_LEFT_ID = 28;
  public static final int BOTTOM_LIMIT_SWITCH_ID = 5;
  public static final int TOP_LIMIT_SWITCH_ID = 6;

  // This tolerance value will be used for deciding if the elevator
  // should target to its setpoint or if the setpoint is too far
  // away and the elevator should just maintain its current position.
  public static final double TOLORENCE_METERS_FOR_SETPOINT = 0.0;
  // This tolerance value will be used for moving to a setpoint
  // using the MoveToPositionCommand.
  public static final double TOLERENCE_METERS_FOR_MOVE_TO_POSITION = 0.25;

  // In meters
  public static final double MAX_HEIGHT_CARRIAGE = 1.527175;
  public static final double MAX_HEIGHT_STAGE_ONE = 0.7366;
  public static final double STAGE_ONE_TO_CARRIAGE_HEIGHT =
      MAX_HEIGHT_CARRIAGE / MAX_HEIGHT_STAGE_ONE;
  public static final double GEAR_RATIO = 1.0 / 6.25;
  public static final double ROTATIONS_TO_METERS = 0.1016 * STAGE_ONE_TO_CARRIAGE_HEIGHT;
  public static final double MOTOR_ROTATIONS_TO_METERS = 1;
  public static final double METERS_TO_MOTOR_ROTATIONS = 1;

  public static final double ELEVATOR_BUMP_UP = 1.0;
  public static final double ELEVATOR_BUMP_DOWN = -1.0;

  public class PIDs {
    public static final double ELEVATOR_kP = 1.6;
    public static final double ELEVATOR_kI = 0.125;
    public static final double ELEVATOR_kD = 0.0;
    public static final double ELEVATOR_kF = 0.58;
    public static final double ELEVATOR_kV = 0.0;
    public static final double ELEVATOR_kA = 0.0;
    public static final double ELEVATOR_kS = 0.0;

    public static final boolean SMART_PID_ENABLED = false;
  }

  public class Profile {
    public static final double MAX_VELOCITY = 50.0;
    public static final double MAX_ACCELERATION = 115.0 * 1.05;
    public static final double MAX_VELOCITY_NET = 60.0;
  }
}
