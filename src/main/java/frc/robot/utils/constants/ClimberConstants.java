package frc.robot.utils.constants;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {

  public class IDs {
    public static final int CLIMBER_KRAKEN_MOTOR = 13;
    public static final int CLIMBER_MAGNET_SENSOR_1 = 3;
    public static final int CLIMBER_MAGNET_SENSOR_2 = 4;
  }

  public class PIDs {
    public static final double CLIMBER_KP = 1; // TODO tune constants
    public static final double CLIMBER_KD = 0.0;
    public static final double CLIMBER_KI = 0.0;
    public static final double CLIMBER_KF = 0.0;
  }

  public static final boolean CLIMBER_SMARTPID_ACTIVE = false;

  public static final double CLIMBER_MAX =
      Units.inchesToMeters(12); // in meters TODO figure out max height
  public static final double CLIMBER_MIN = 0.0; // in meters

  public static final double CLIMBER_TOLERANCE = 0.001;

  public static final double CLIMBER_GEAR_RATIO =
      1.0 / 9.0; // TODO check with vince (he said 20 to 1, i think i did the math right but idk)
  public static final double CLIMBER_ROTATIONS_TO_METERS = Units.inchesToMeters(0.25);
  public static final double CLIMBER_MOTOR_ROTATIONS_TO_CLIMBER_HEIGHT =
      CLIMBER_GEAR_RATIO * CLIMBER_ROTATIONS_TO_METERS;
}
