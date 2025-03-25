package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(
          //   AprilTagFields.k2025ReefscapeAndyMark
          AprilTagFields.k2025ReefscapeWelded); // TODO: change the field type at comp

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

  // Robot to camera transforms, measured empirically
  public static Transform3d ROBOT_TO_CAM_FL_TRANSFORM =
      new Transform3d(
          Inches.of(11.75).in(Meters),
          Inches.of(11.089).in(Meters),
          Inches.of(8 + 15.0 / 32.0).in(Meters),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(-30)));
  public static Transform3d ROBOT_TO_CAM_FR_TRANSFORM =
      new Transform3d(
          Inches.of(11.75).in(Meters),
          Inches.of(-11.571).in(Meters),
          Inches.of(8 + 15.0 / 32.0).in(Meters),
          new Rotation3d(0.0, 0.0, Units.degreesToRadians(30)));
  // Basic filtering thresholds
  public static double maxSingleTagAmbiguity = 0.4;
  public static Distance maxZError = Meters.of(0.1);
  public static Distance maxSingleTagDistance = Meters.of(1.25);

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.35; // Meters
  public static double angularStdDevBaseline = Double.POSITIVE_INFINITY; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera FL
        1.0, // Camera FR
      };
}
