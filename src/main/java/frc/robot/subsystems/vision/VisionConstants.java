package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(
          AprilTagFields.k2025ReefscapeWelded); // TODO: change this at comp

  public static List<Pose2d> blueReefTagPoses =
      List.of(
          aprilTagLayout.getTagPose(17).get().toPose2d(),
          aprilTagLayout.getTagPose(18).get().toPose2d(),
          aprilTagLayout.getTagPose(19).get().toPose2d(),
          aprilTagLayout.getTagPose(20).get().toPose2d(),
          aprilTagLayout.getTagPose(21).get().toPose2d(),
          aprilTagLayout.getTagPose(22).get().toPose2d());

  public static List<Pose2d> redReefTagPoses =
      List.of(
          aprilTagLayout.getTagPose(6).get().toPose2d(),
          aprilTagLayout.getTagPose(7).get().toPose2d(),
          aprilTagLayout.getTagPose(8).get().toPose2d(),
          aprilTagLayout.getTagPose(9).get().toPose2d(),
          aprilTagLayout.getTagPose(10).get().toPose2d(),
          aprilTagLayout.getTagPose(11).get().toPose2d());

  // Camera names, must match names configured on coprocessor
  public static String CAM_FL_NAME = "CamLeft";
  public static String CAM_FR_NAME = "CamRight";
  public static String CAM_BACK_NAME = "OV9281-3";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  // TODO: find out proper transforms
  public static Transform3d ROBOT_TO_CAM_FL_TRANSFORM =
      new Transform3d(
          0.3175, 0.2921, 0.1778, new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30)));
  public static Transform3d ROBOT_TO_CAM_FR_TRANSFORM =
      new Transform3d(
          0.3175, -0.2921, 0.1778, new Rotation3d(0.0, 0.0, Units.degreesToRadians(30)));
  public static Transform3d ROBOT_TO_CAM_BACK_TRANSFORM =
      new Transform3d(-0.05, 0, 0.3, new Rotation3d(0.0, 0.0, Math.PI / 2.0));
  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
  public static double maxZError = 0.1;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 1.0; // Meters
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera FL
        1.0, // Camera FR
        1.0 // Camera back
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
