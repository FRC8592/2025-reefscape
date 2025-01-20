package lib.team8592.math;

public class Conversions {
    /**
     * 4096 ticks per revolution
     * 360 degrees per revolution
     */
    public static double falconTicksToDegrees(double ticks) {
        return ticks / 4096 * 360;
    }

    /**
     * 4096 ticks per revolution
     * 360 degrees per revolution
     */
    public static double degreesToFalconTicks(double degrees) {
        return degrees / 360 * 4096;
    }

    /**
     * 0.0254 meters per inch
     */
    public static double inchesToMeters(double inches) {
        return inches / 0.0254;
    }

    /**
     * 0.0254 meters per inch
     */
    public static double metersToInches(double meters) {
        return meters * 0.0254;
    }

    /**
     * 360 degrees per revolution
     */
    public static double degreesToRotations(double degrees) {
        return degrees / 360.0;
    }

    /**
     * 360 degrees per revolution
     */
    public static double rotationsToDegrees(double rotations) {
        return rotations / 360.0;
    }
 }