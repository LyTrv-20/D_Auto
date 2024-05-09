// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package CSP_Lib.utils.atlatl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import CSP_Lib.utils.atlatl.apriltags.AprilTagLayoutType;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Estimator class using ATLATL (Aiden's Trigonometric Localization Algorithm for Translational Localization) */
public class Estimator {
    
    private static final double ROOT_TWO_OVER_TWO = Math.sqrt(2.0) / 2.0;
    private static HashMap<Integer, Translation3d> readTranslations() {
        HashMap<Integer, Translation3d> result = new HashMap<>();
        AprilTagFieldLayout layout = AprilTagLayoutType.OFFICIAL.layout;

        for (AprilTag tag : layout.getTags()) {
            result.put(tag.ID, tag.pose.getTranslation());
        }

        return result;
    }
    private static final HashMap<Integer, Translation3d> tagTranslations = readTranslations();

    /**
     * Calculate an estimated position of the robot using a single {@link VisionResult}
     * @param camera The {@link CameraLocation} containing the data characterizing the orientation of the camera.
     * @param vision The {@link VisionResult} containing the data for a single tag.
     * @return The {@link LocationResult} based solely on the input data. Returns null if the tag viewed isn't in the tag set.
     */
    public static LocationResult localize(CameraLocation camera, VisionResult vision) {

        // Correct the angles of the vision reading with the camera mounting information.
        Translation2d visionAngles = new Translation2d(
            vision.tx + Math.toDegrees(camera.cameraMounting.getZ()),
            vision.ty + Math.toDegrees(camera.cameraMounting.getRotation().getY()))
            .rotateBy(Rotation2d.fromRadians(camera.cameraMounting.getRotation().getX()));

        // Find the total angle from the positive X axis on the field to the target.
        Rotation2d totalAngle = camera.gyroAngle
            .plus(Rotation2d.fromDegrees(visionAngles.getX()));

        // Get the translation corresponding to the viewed tag.
        Translation3d tagTranslation = tagTranslations.get(vision.id);

        // Stop here and return zero if the tag isn't in the tag set.
        if (tagTranslation == null) return null;

        // Calculate distance from the robot to the target.
        // d = z / tan(ty)
        double distance = (tagTranslation.getZ() - camera.cameraMounting.getZ()) / Math.tan(Math.toRadians(vision.ty));

        // Calculate the position of the robot relative to the goal.
        Translation2d position = new Translation2d(distance, totalAngle)
            .plus(tagTranslation.toTranslation2d())
            .minus(camera.cameraMounting.getTranslation().toTranslation2d().rotateBy(camera.gyroAngle.times(-1.0)));

        // Store the sine of ty to avoid repeated operation.
        double sin = Math.sin(vision.ty);

        // Calculate the component specific vision standard deviation
        // (assuming the standard deviation for tx and ty are equal).
        double visionStdDev = vision.stdDev * ROOT_TWO_OVER_TWO;

        // Calculate standard deviations of the position estimate.
        Translation2d stdDevs = new Translation2d(
            // Find the derivative of the distance with respect to whatever the vision standard deviation is with respect to.
            // -s*z/(sin(ty)^2)
            -visionStdDev * tagTranslation.getZ() / (sin * sin),
            // Find the derivative of the rotation from the target with respect to whatever the vision standard deviation is with respect to.
            // d*pi*s/180.0
            distance * Math.PI * visionStdDev / 180.0
        ).rotateBy(totalAngle);

        // Make sure both components of the standard deviation are positive.
        stdDevs = new Translation2d(Math.abs(stdDevs.getX()), Math.abs(stdDevs.getY()));

        return new LocationResult(position, stdDevs);
    }

    /**
     * Calculate an estimated position of the robot using a set of {@link VisionResult}s
     * @param camera The {@link CameraLocation} containing the data characterizing the orientation of the camera.
     * @param visionResults The set of {@link VisionResult}s containing the data for a set of tags.
     * @return The set of {@link LocationResult}s based solely on the input data. Doesn't include tags which aren't part of the tag set
     */
    public static List<LocationResult> localize(CameraLocation camera, VisionResult... visionResults) {

        // Pre-allocate an ArrayList the size of visionResults.
        ArrayList<LocationResult> result = new ArrayList<>(visionResults.length);

        for (VisionResult measurement : visionResults) {

            // If the localize method doesn't return null then include the result in the result array.
            LocationResult location = localize(camera, measurement);
            if (location != null)
                result.add(location);
        }

        return result;
    }
}
