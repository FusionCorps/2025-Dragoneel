// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator singleTagPoseEstimator;
  protected final PhotonPoseEstimator multiTagPoseEstimator;
  protected final Supplier<Rotation2d> rotationSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> rotationSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.rotationSupplier = rotationSupplier;
    singleTagPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagLayout, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE, robotToCamera);
    multiTagPoseEstimator =
        new PhotonPoseEstimator(
            VisionConstants.aprilTagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();
    for (var result : camera.getAllUnreadResults()) {
      singleTagPoseEstimator.addHeadingData(result.getTimestampSeconds(), rotationSupplier.get());
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));

        int bestTagId = result.getBestTarget().getFiducialId();
        if (DriverStation.getAlliance().isPresent()) {
          if (DriverStation.getAlliance().get() == Alliance.Blue) {
            if (17 <= bestTagId && bestTagId <= 22) inputs.bestReefTagId = bestTagId;
          } else {
            if (6 <= bestTagId && bestTagId <= 11) inputs.bestReefTagId = bestTagId;
          }
        }
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }

      // Add pose observation
      if (result.getMultiTagResult().isPresent()) { // Multitag result
        multiTagPoseEstimator
            .update(result)
            .ifPresent(
                (robotPoseEst) -> {
                  // Calculate average tag distance
                  double totalTagDistance = 0;
                  for (var target : robotPoseEst.targetsUsed) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                  }

                  // Add tag IDs
                  tagIds.addAll(
                      robotPoseEst.targetsUsed.stream().map(t -> (short) t.fiducialId).toList());

                  // Add observation
                  poseObservations.add(
                      new PoseObservation(
                          result.getTimestampSeconds(), // Timestamp
                          robotPoseEst.estimatedPose, // 3D pose estimate
                          result.multitagResult.get().estimatedPose.ambiguity, // Ambiguity
                          robotPoseEst.targetsUsed.size(), // Tag count
                          totalTagDistance / result.targets.size(), // Average tag distance
                          PoseObservationType.PHOTONVISION)); // Observation type
                });
      } else if (!result.targets.isEmpty()) { // Single tag result
        singleTagPoseEstimator
            .update(result)
            .ifPresent(
                robotPoseEst -> {
                  var target = robotPoseEst.targetsUsed.get(0);

                  // Add tag ID
                  tagIds.add((short) robotPoseEst.targetsUsed.get(0).fiducialId);

                  // Add observation
                  poseObservations.add(
                      new PoseObservation(
                          robotPoseEst.timestampSeconds, // Timestamp
                          robotPoseEst.estimatedPose, // 3D pose estimate
                          target.poseAmbiguity, // Ambiguity
                          1, // Tag count
                          target
                              .getBestCameraToTarget()
                              .getTranslation()
                              .getNorm(), // Average tag distance
                          PoseObservationType.PHOTONVISION)); // Observation type
                });
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
