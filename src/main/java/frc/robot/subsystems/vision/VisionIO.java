package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  record VisionMeasurement(Pose3d estimatedPose, double timestamp, Matrix<N3, N1> stdDevs) {}

  @AutoLog
  public static class VisionIOInputs {
    boolean connected = false;
    VisionMeasurement[] measurements = new VisionMeasurement[0];
  }

  public void updateInputs(VisionIOInputs inputs);

  public String getName();
}
