package frc.robot.utils.constants;

public class TestingConstants {
  public class Testing {
    // if ENSURE_COMPETITION_READY_SUBSYSTEMS is true, all subystems
    // must be constructed and assigned to the correct variable in Robot.java.
    // If some subsystems are not created and this value is true, an exeption
    // will be thrown.
    public static final boolean ENSURE_COMPETITION_READY_SUBSYSTEMS = true;
    public static final boolean SMART_PID_ENABLED = false;

    public static enum Robot {
      Comp2024,
      Comp2025
    }

    // Change this to test on the 2024 robot's drivebase.
    public static Robot ROBOT = Robot.Comp2025;

    public static final double NUMBER_OF_MOTOR_ROTATIONS_FOR_MODULE_TEST = 1.0;
    public static final double TURN_MOTOR_ROTATION_SPEED = 0.15;
    public static final double TURN_MOTOR_AND_ENCODER_TOLERANCE = 0.05;

    public static final double CLIMBER_HEIGHT_CHANGE = 0.05;
    public static final double CLIMBER_CURRENT_TOLERANCE = 10; // TODO find tolerance
    public static final double ELEVATOR_MAX_SPEED = .08;
  }
}
