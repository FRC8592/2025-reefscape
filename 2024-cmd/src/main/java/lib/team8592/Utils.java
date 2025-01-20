package lib.team8592;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Utils {
    private static final double RED_WALL_X = 16.542;

    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double range) {
        return Math.max(Math.min(value, range), -range);
    }

    /**
     * Rounds the number to a certain number of decimal places
     */
    public static double roundTo(double value, double decimalPlaces) {
        return Math.round(value*Math.pow(10, decimalPlaces))/Math.pow(10, decimalPlaces);
    }

    /**
     * Whether the particlar value is within a specified tolerance of the target
     */
    public static boolean isWithin(double value, double target, double tolerance) {
        return Math.abs(target - value) <= tolerance;
    }

    public static Pose2d mirrorPose(Pose2d pose, boolean flip) {
        Pose2d newPose = pose;
        if (flip) {
            newPose = new Pose2d(
                new Translation2d(RED_WALL_X - pose.getX(), pose.getY()),
                Rotation2d.fromDegrees(180).minus(pose.getRotation())
            );
        }

        return newPose;
    }

    public static double getMOIForRoller(double massKG, double radiusMeters) {
        return (massKG * Math.pow(radiusMeters, 2d)) / 2d;
    }
}