package lib.team8592.pathing;

import choreo.Choreo;
import edu.wpi.first.math.geometry.Pose2d;

public final class ChoreoUtils {
    /**
     * The initial pose for the given Choreo path
     */
    public static Pose2d getStartPoseFromTrajectory(String file, boolean flip) {
        return Choreo.loadTrajectory(file).get().getInitialPose(flip).get();
    }
}
