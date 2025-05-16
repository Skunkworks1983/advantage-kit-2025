package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class LidarDrivebaseConstants {
  public static final double AUTO_ALIGN_DRIVE_SPEED_TELEOP = 0.5;

  public static final double FIELD_X_LENGTH = 17.55; // Meters
  public static final double FIELD_Y_LENGTH = 8.05; // Meters

  public class TeleopFeature {
    public static final Translation2d FIELD_CENTER =
        new Translation2d(FIELD_X_LENGTH / 2.0, FIELD_Y_LENGTH / 2.0);
    public static final Translation2d APPROXIMATE_BLUE_START_POSE =
        FIELD_CENTER.plus(new Translation2d(0.0, -1.5));
    public static final Translation2d APPROXIMATE_RED_START_POSE =
        FIELD_CENTER.plus(new Translation2d(0.0, 1.5));
    public static final Translation2d REEF_BLUE = new Translation2d(4.0259, 4.48945);
    public static final Translation2d REEF_RED =
        new Translation2d(FIELD_X_LENGTH - 4.0259, 4.48945);
    public static final Rotation2d BLUE_LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-45);
    public static final Rotation2d BLUE_RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(45);
    public static final Rotation2d RED_LEFT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(135);
    public static final Rotation2d RED_RIGHT_CORAL_STATION_ANGLE = Rotation2d.fromDegrees(-135);
    public static final double REEF_SMALL_RADIUS = Units.inchesToMeters(93.50 / 2);
    public static final double BRANCH_LINEUP_HORIZONTAL_OFFSET = 1.0;
    public static final double BRANCH_LINEUP_DIST_FROM_REEF = 1.0;
    public static final double MAX_DIST_FROM_REEF_CENTER = 3.0;
  }

  public static final int LIDAR_RIGHT_DATA_PORT = 8;
  public static final int LIDAR_RIGHT_TRIGGER_DISTANCE = 60;
  public static final int LIDAR_RIGHT_DATA_CUTOFF = 2000;
  public static final int LIDAR_LEFT_DATA_PORT = 4;
  public static final int LIDAR_LEFT_TRIGGER_DISTANCE = 60;
  public static final int LIDAR_LEFT_DATA_CUTOFF = 2000;
  public static final int LIDAR_LEFT_TRIGGER_PORT = 3;
  public static final int LIDAR_RIGHT_TRIGGER_PORT = 7;
}
