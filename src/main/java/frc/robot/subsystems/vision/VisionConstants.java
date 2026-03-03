// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String frontCameraName = "FrontCamera";
    public static String leftCameraName = "LeftCamera";
    public static String backCameraName = "BackCamera";
    public static String rightCameraName = "RightCamera";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToFrontCamera = new Transform3d(-0.13, -0.224, 0.706,
            new Rotation3d(0.0, Units.degreesToRadians(0), 0.0));
    public static Transform3d robotToLeftCamera = new Transform3d(-0.095, 0.345, 0.19,
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(90.0)));
    public static Transform3d robotToBackCamera = new Transform3d(-0.313, 0.213, 0.19,
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(180.0)));
    public static Transform3d robotToRightCamera = new Transform3d(-0.20, -0.325, 0.19,
            new Rotation3d(0.0, Units.degreesToRadians(-25), Units.degreesToRadians(-90.0)));
            
    // Basic filtering thresholds
    public static double maxAmbiguity = 0.15; // Tightened from 0.3 to kill ambiguity flipping
    public static double maxZError = 0.75;
    public static double maxSingleTagDistance = 4.0; // Meters - ignore single tags further than this

    // Standard deviation baselines
    public static double multiTagLinearStdDev = 0.05; // Highly stable, trust it more
    public static double multiTagAngularStdDev = 0.05;
    public static double singleTagLinearStdDev = 0.5; // 10x less trust to act as a rigid noise filter

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors = new double[] {
            1.0, // Front Camera
            1.0, // Left Camera
            1.0, // Back Camera
            1.0 // Right Camera
    };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available

    // Array of transforms for logging actual camera positions in 3D space
    public static Transform3d[] cameraTransforms = new Transform3d[] {
            robotToFrontCamera,
            robotToLeftCamera,
            robotToBackCamera,
            robotToRightCamera
    };
}