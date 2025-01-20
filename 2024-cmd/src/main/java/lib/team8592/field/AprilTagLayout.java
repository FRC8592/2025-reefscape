package lib.team8592.field;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

public class AprilTagLayout extends AprilTagFieldLayout {
    private static final double FIELD_LENGTH_METERS = 16.541;
    private static final double FIELD_WIDTH_METERS = 8.211;

    public AprilTagLayout(List<AprilTag> apriltags) {
        super(apriltags, FIELD_LENGTH_METERS, FIELD_WIDTH_METERS);
    }
}