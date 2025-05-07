package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.visionIOConstants.VisionIOConstants;
import frc.robot.subsystems.vision.visionIOConstants.VisionIOConstantsPhotonVision;

public class VisionConstants {
  public class TripleMount {
    /*
     * Camera_0: The front facing camera
     * Camera_1: The topmost camera
     * Camera_2: The reverse facing camera
     */
    private static final String CAMERA_0_NAME = "Camera_0";
    private static final String CAMERA_1_NAME = "Camera_1";
    private static final String CAMERA_2_NAME = "Camera_2";

    private static final Transform3d MOUNT_TO_CAMERA_0 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.177841),
                Units.inchesToMeters(-0.464944),
                Units.inchesToMeters(0.0)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-19.275000),
                Units.degreesToRadians(-30.00)));

    private static final Transform3d MOUNT_TO_CAMERA_1 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.750618),
                Units.inchesToMeters(1.889800),
                Units.inchesToMeters(1.907574)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-27.225000),
                Units.degreesToRadians(55.0000)));

    private static final Transform3d MOUNT_TO_CAMERA_2 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-2.083605),
                Units.inchesToMeters(2.270524),
                Units.inchesToMeters(0.046923)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-27.225000),
                Units.degreesToRadians(135.00)));

    private static final Transform3d ROBOT_TO_MOUNT =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    private static final Transform3d ROBOT_TO_CAMERA_0 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_0);
    private static final Transform3d ROBOT_TO_CAMERA_1 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_1);
    private static final Transform3d ROBOT_TO_CAMERA_2 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_2);

    public static final VisionIOConstants[] VISION_IO_CONSTANTS = {
      new VisionIOConstantsPhotonVision(CAMERA_0_NAME, ROBOT_TO_CAMERA_0),
      new VisionIOConstantsPhotonVision(CAMERA_1_NAME, ROBOT_TO_CAMERA_1),
      new VisionIOConstantsPhotonVision(CAMERA_2_NAME, ROBOT_TO_CAMERA_2)
    };
  }

  public class DoubleMount {
    /*
     * Camera_0: The front facing camera
     * Camera_1: The side facing camera
     */
    private static final String CAMERA_0_NAME = "Camera_0";
    private static final String CAMERA_1_NAME = "Camera_1";

    private static final Transform3d MOUNT_TO_CAMERA_0 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(1.351),
                Units.inchesToMeters(-1.268),
                Units.inchesToMeters(-0.81)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-19.27),
                Units.degreesToRadians(-15.0)));

    private static final Transform3d MOUNT_TO_CAMERA_1 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-1.050),
                Units.inchesToMeters(1.365078),
                Units.inchesToMeters(-0.762394)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-27.225),
                Units.degreesToRadians(97.0)));

    private static final Transform3d ROBOT_TO_MOUNT =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    private static final Transform3d ROBOT_TO_CAMERA_0 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_0);
    private static final Transform3d ROBOT_TO_CAMERA_1 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_1);

    public static final VisionIOConstants[] VISION_IO_CONSTANTS = {
      new VisionIOConstantsPhotonVision(CAMERA_0_NAME, ROBOT_TO_CAMERA_0),
      new VisionIOConstantsPhotonVision(CAMERA_1_NAME, ROBOT_TO_CAMERA_1)
    };
  }

  public class SwerveModuleMount {
    /*
     * Camera_0: The camera mounted on the front right module, facing right.
     * Camera_1: The camera mounted on the front left module, facing fowards.
     */
    private static final String CAMERA_0_NAME = "Camera_0";
    private static final String CAMERA_1_NAME = "Camera_1";

    private static final Transform3d MOUNT_TO_CAMERA_0 =
        new Transform3d(
            new Translation3d(
                Units.feetToMeters(0.925),
                Units.feetToMeters(-0.8041666),
                Units.inchesToMeters(8.0)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(-90.0)));

    private static final Transform3d MOUNT_TO_CAMERA_1 =
        new Transform3d(
            new Translation3d(
                Units.feetToMeters(0.925),
                Units.feetToMeters(0.8041666),
                Units.inchesToMeters(8.0)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-25.0),
                Units.degreesToRadians(0.0)));

    private static final Transform3d ROBOT_TO_MOUNT =
        new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0));

    private static final Transform3d ROBOT_TO_CAMERA_0 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_0);
    private static final Transform3d ROBOT_TO_CAMERA_1 = ROBOT_TO_MOUNT.plus(MOUNT_TO_CAMERA_1);

    public static final VisionIOConstants[] VISION_IO_CONSTANTS = {
      new VisionIOConstantsPhotonVision(CAMERA_0_NAME, ROBOT_TO_CAMERA_0),
      new VisionIOConstantsPhotonVision(CAMERA_1_NAME, ROBOT_TO_CAMERA_1)
    };
  }

  public class Comp2025Mount {
    /*
     * Camera_0: FRONT RIGHT module
     * Camera_1: FRONT LEFT module
     * Camera_2: BACK RIGHT module
     */
    private static final String CAMERA_0_NAME = "FR_Camera";
    private static final String CAMERA_1_NAME = "Camera_1";
    private static final String CAMERA_2_NAME = "Camera_2";

    private static final Transform3d FR_MAGNET_TO_CAMERA_0 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.267),
                Units.inchesToMeters(0.374),
                Units.inchesToMeters(-1.995)),
            new Rotation3d(
                Units.degreesToRadians(-5.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(35.310)));

    private static final Transform3d FL_MAGNET_TO_CAMERA_1 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-0.375),
                Units.inchesToMeters(0.415),
                Units.inchesToMeters(-1.995)),
            new Rotation3d(
                Units.degreesToRadians(-5.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-125.0)));

    private static final Transform3d BR_MAGNET_TO_CAMERA_2 =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-1.118),
                Units.inchesToMeters(0.200),
                Units.inchesToMeters(-1.995)),
            new Rotation3d(
                Units.degreesToRadians(-5.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-22.402)));

    private static final Transform3d ROBOT_TO_FR_MAGNET =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.375),
                Units.inchesToMeters(-11.375),
                Units.inchesToMeters(6.014)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));
    private static final Transform3d ROBOT_TO_FL_MAGNET =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.375),
                Units.inchesToMeters(11.375),
                Units.inchesToMeters(6.014)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(90.0)));
    private static final Transform3d ROBOT_TO_BR_MAGNET =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11.375),
                Units.inchesToMeters(-11.375),
                Units.inchesToMeters(6.014)),
            new Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(0.0)));

    public static final VisionIOConstants[] IO_CONSTANTS = {
      // new VisionIOConstantsPhotonVision(CAMERA_0_NAME,
      // ROBOT_TO_FR_MAGNET.plus(FR_MAGNET_TO_CAMERA_0)),
    };
  }

  public static final double MAX_AMBIGUITY = 0.3;
  public static final double LINEAR_STD_DEV_BASELINE = 0.06;
  public static final double ANGULAR_STD_DEV = 5.0;
  public static final double MAX_Z_ERROR = 3.0;
  public static final double MAX_AVERAGE_TAG_DISTANCE = 3.0; // Meters
  public static final int JITTER_TEST_ROLLING_AVG_LENGTH = 20;
}
