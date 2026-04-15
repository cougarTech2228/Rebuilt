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

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionConsumer consumer;
  private final Function<Double, Rotation2d> gyroRotationSupplier;
  private final Supplier<Pose2d> currentPoseSupplier;
  private final VisionIO[] io;
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;

  private static record ProcessedVisionUpdate(
      Pose3d pose,
      double timestamp,
      Matrix<N3, N1> stdDevs,
      int cameraIndex
  ) {}

  public Vision(
      VisionConsumer consumer,
      Function<Double, Rotation2d> gyroRotationSupplier,
      Supplier<Pose2d> currentPoseSupplier,
      VisionIO... io) {
    this.consumer = consumer;
    this.gyroRotationSupplier = gyroRotationSupplier;
    this.currentPoseSupplier = currentPoseSupplier;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Collect valid updates from all cameras
    List<ProcessedVisionUpdate> validUpdates = new ArrayList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = aprilTagLayout.getTagPose(tagId);
        if (tagPose.isPresent()) {
          tagPoses.add(tagPose.get());
        }
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        Pose3d bestPose = observation.pose();

        // 1. DISAMBIGUATION (Gyro-Assisted Single Tag Solve)
        if (observation.tagCount() == 1 && observation.altPose() != null) {
          Rotation2d gyroRotation = gyroRotationSupplier.apply(observation.timestamp());
          
          double bestRotationDiff = Math.abs(bestPose.toPose2d().getRotation().minus(gyroRotation).getDegrees());
          double altRotationDiff = Math.abs(observation.altPose().toPose2d().getRotation().minus(gyroRotation).getDegrees());

          // Select the pose that aligns best with our physical gyro heading
          if (altRotationDiff < bestRotationDiff) {
            bestPose = observation.altPose();
          }
        }

        // 2. BOUNDARY FILTERING (With Margins)
        boolean outOfBounds = 
            bestPose.getX() < -fieldBorderMargin
            || bestPose.getX() > aprilTagLayout.getFieldLength() + fieldBorderMargin
            || bestPose.getY() < -fieldBorderMargin
            || bestPose.getY() > aprilTagLayout.getFieldWidth() + fieldBorderMargin;

        // 3. DISTANCE FILTERING (Tiered Based on Tag Count)
        double maxDistance = observation.tagCount() == 1 ? maxSingleTagDistance : maxMultiTagDistance;

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity() > maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(bestPose.getZ())
                    > maxZError // Must have realistic Z coordinate
                || outOfBounds // Must be within the field boundaries
                || observation.averageTagDistance() > maxDistance; // Tiered distance rejection

        // Add pose to log
        robotPoses.add(bestPose);
        if (rejectPose) {
          robotPosesRejected.add(bestPose);
          continue; // Skip calculating/adding valid updates if rejected
        } else {
          robotPosesAccepted.add(bestPose);
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = linearStdDevBaseline * stdDevFactor;
        double angularStdDev = angularStdDevBaseline * stdDevFactor;
        if (observation.type() == PoseObservationType.MEGATAG_2) {
          linearStdDev *= linearStdDevMegatag2Factor;
          angularStdDev *= angularStdDevMegatag2Factor;
        }
        if (cameraIndex < cameraStdDevFactors.length) {
          linearStdDev *= cameraStdDevFactors[cameraIndex];
          angularStdDev *= cameraStdDevFactors[cameraIndex];
        }

        // Pose-deviation scaling: reduce trust in measurements that are far from
        // the current estimate. This prevents visible "jumping" while still allowing
        // gradual drift correction over many frames.
        double poseDeviation = bestPose.toPose2d().getTranslation()
            .getDistance(currentPoseSupplier.get().getTranslation());
        double deviationScale = 1.0 + poseDeviationScaleFactor
            * Math.max(0.0, poseDeviation - poseDeviationSoftThreshold);
        linearStdDev *= deviationScale;
        angularStdDev *= deviationScale;

        // Queue vision observation to be sorted and dispatched
        validUpdates.add(
            new ProcessedVisionUpdate(
                bestPose,
                observation.timestamp(),
                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev),
                cameraIndex));
      }

      // Log camera datadata
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/TagPoses",
          tagPoses.toArray(new Pose3d[tagPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPoses",
          robotPoses.toArray(new Pose3d[robotPoses.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
      Logger.recordOutput(
          "Vision/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // 4. CHRONOLOGICAL & QUALITY SORTING
    // Sort updates by timestamp (oldest first). If timestamps match, tiebreak by lowest standard deviation.
    validUpdates.sort(
        Comparator.comparingDouble(ProcessedVisionUpdate::timestamp)
            .thenComparingDouble(
                update -> update.stdDevs().get(0, 0) + update.stdDevs().get(1, 0) + update.stdDevs().get(2, 0)));

    // Send sorted vision observations to drive subsystem and log diagnostics
    for (int updateIdx = 0; updateIdx < validUpdates.size(); updateIdx++) {
      var update = validUpdates.get(updateIdx);
      consumer.accept(update.pose().toPose2d(), update.timestamp(), update.stdDevs());
    }
    // Log aggregate vision diagnostics for tuning in AdvantageScope
    Logger.recordOutput("Vision/Summary/AcceptedCount", validUpdates.size());
    if (!validUpdates.isEmpty()) {
      var best = validUpdates.get(0);
      Logger.recordOutput("Vision/Summary/BestLinearStdDev", best.stdDevs().get(0, 0));
      Logger.recordOutput("Vision/Summary/BestAngularStdDev", best.stdDevs().get(2, 0));
    }

    // Log summary data
    Logger.recordOutput(
        "Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesAccepted",
        allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
    Logger.recordOutput(
        "Vision/Summary/RobotPosesRejected",
        allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}