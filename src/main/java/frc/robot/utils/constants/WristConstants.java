package frc.robot.utils.constants;

public class WristConstants {
  public class Wrist {
    public class IDs {
      public static final int WRIST_KRAKEN_MOTOR_ID = 4;
      public static final int WRIST_TOP_MAGNET_SENSOR =
          0; // Magnet Sensor ids are currently not working
      public static final int WRIST_BOTTOM_MAGNET_SENSOR = 2;
    }

    public class PIDs {
      public static final double WRIST_KA = 0.0;
      public static final double WRIST_KS = 0.0;
      public static final double WRIST_KV = 0.0;
      public static final double WRIST_KP = 3.0;
      public static final double WRIST_KD = 0.0;
      public static final double WRIST_KI = 0.0;
      public static final double WRIST_KF = 0.0;

      public static final boolean WRIST_SMARTPID_ACTIVE = false;
    }

    public static final double WRIST_MAX_VELOCITY = 0.7;
    public static final double WRIST_MAX_ACCELERATION = 1.75;

    public static final double WRIST_TOLERANCE = 0.02;

    public static final int WRIST_GEAR_RATIO = 56; // 56 motor rotations to 1 wrist rotation

    public static final double WRIST_MIDPOINT_ROTATIONS =
        0.2 * WRIST_GEAR_RATIO; // TODO: figure out postitions
    public static final double WRIST_MIN_ROTATIONS = 0;
    public static final double WRIST_MAX_ROTATIONS = 0.4 * WRIST_GEAR_RATIO;
  }
}
