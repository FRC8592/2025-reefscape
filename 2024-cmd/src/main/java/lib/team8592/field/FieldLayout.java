package lib.team8592.field;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import lib.team8592.logging.LogUtils;

public abstract class FieldLayout {
    private AprilTagLayout aprilTagLayout;
    private Field2d field = new Field2d();

    public enum Alliance {
        kRed,
        kBlue;
    }

    public FieldLayout(AprilTagLayout layout) {
        this.aprilTagLayout = layout;
    }

    public FieldLayout(List<AprilTag> tags) {
        this.aprilTagLayout = new AprilTagLayout(tags);
    }

    public AprilTagFieldLayout getAprilTagLayout() {
        return this.aprilTagLayout;
    }

    public Pose2d getSimulatedRobotPose() {
        return this.field.getRobotPose();
    }

    public void setSimulatedRobotPose(Pose2d robotPose) {
        this.field.setRobotPose(robotPose);
    }

    public Field2d getField() {
        return this.field;
    }

    public void logToShuffleboard(boolean log) {
        if (log) LogUtils.addSendable(field);
    }

    public static FieldLayout none() {
        return new FieldLayout(new AprilTagLayout(List.of())) {};
    }

    public static AprilTagFieldLayout createLayout(String path) {
        try {
            return AprilTagFieldLayout.loadFromResource(path);
        } catch (IOException e) {
            System.out.println("April Tag Layout Does Not Exist!!");
            return null;
        }
    }
}