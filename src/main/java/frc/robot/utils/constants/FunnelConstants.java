package frc.robot.utils.constants;

public class FunnelConstants {
  public class IDs {
    public static final int PIVOT_MOTOR_ID = 23;
  }

  public static final double FUNNEL_TOLERANCE = 0.1;

  public class PIDs {
    public static final double FUNNEL_KP = .1;
    public static final double FUNNEL_KD = 0.0;
    public static final double FUNNEL_KI = 0.0;
    public static final double FUNNEL_KF = 0.0;

    public static final boolean FUNNEL_SMARTPID_ACTIVE = false;
  }

  public static final double PIVOT_MOTOR_GEAR_RATIO = 1.0 / 100.0;
  public static final double FUNNEL_POSITION_LOW_CONVERTED =
      0.0; // TODO FIGURE OUT POSITIONS IN ROTATIONS
  public static final double FUNNEL_POSITION_HIGH_IN_DEGREES = 40.0;
  public static final double FUNNEL_POSITION_HIGH_CONVERTED =
      (FUNNEL_POSITION_HIGH_IN_DEGREES / 360) / PIVOT_MOTOR_GEAR_RATIO;
}
