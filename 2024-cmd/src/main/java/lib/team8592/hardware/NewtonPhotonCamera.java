package lib.team8592.hardware;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.*;
import org.photonvision.simulation.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import frc.robot.Constants.VISION;
import lib.team8592.field.FieldLayout;

public class NewtonPhotonCamera {
    private PhotonCamera camera;
    private PhotonCameraSim simCamera;
    private VisionSystemSim simVisionSystem;
    private SimCameraProperties simProperties;

    private boolean isSimulation = false;

    private Transform3d cameraPoseToRobotPose = new Transform3d();

    public NewtonPhotonCamera(String name, Transform3d cameraPoseToRobotPose, AprilTagFieldLayout fieldLayout, boolean isSimulation) {
        this.camera = new PhotonCamera(name);
        this.cameraPoseToRobotPose = cameraPoseToRobotPose;

        this.isSimulation = isSimulation;

        this.simProperties = new SimCameraProperties();
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        this.simProperties.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        this.simProperties.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        this.simProperties.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        this.simProperties.setAvgLatencyMs(35);
        this.simProperties.setLatencyStdDevMs(5);

        this.simCamera = new PhotonCameraSim(camera, simProperties);
        this.simVisionSystem = new VisionSystemSim(name);

        this.simVisionSystem.addAprilTags(fieldLayout);
        this.simVisionSystem.addCamera(simCamera, VISION.CAMERA_OFFSET);
    }

    public void updateSim(Pose3d robotPose) {
        if (isSimulation) {
            this.simCamera.enableDrawWireframe(true);
            this.simVisionSystem.update(robotPose);
        }
    }

    public VisionSystemSim getSimulation() {
        return this.simVisionSystem;
    }

    /*
     * The photonlib camera object
     */
    public PhotonCamera getCamera() {
        return this.camera;
    }

    /**
     * Grabs all visible targets
     */
    public List<PhotonTrackedTarget> getTargets() {
        this.camera.getAllUnreadResults().size();
        if (this.camera.getAllUnreadResults().size() == 0) return new ArrayList<>();
        return this.camera.getAllUnreadResults().get(0).targets;
    }

    /**
     * Grabs the target most matching the current pipeline
     */
    public PhotonTrackedTarget getBestTarget() {
        if (getTargets().size() == 0) return null;

        return getTargets().get(0);
    }

    /**
     * Sets the current pipeline index
     */
    public void setPipeline(int pipeline) {
        this.camera.setPipelineIndex(pipeline);
    }

    /**
     * Whether there is any target visible in the camera
     */
    public boolean isAnyTargetVisible() {
        return this.getBestTarget() != null;
    }

    /**
     * Whether a particular AprilTag is visible in the camera
     */
    public boolean isAprilTagVisible(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return true;
        }
        return false;
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToBestTarget() {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(this.getBestTarget().getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToTarget(PhotonTrackedTarget target) {
        if (!isAnyTargetVisible()) return null; // Return null if no target visible
        return Rotation2d.fromDegrees(target.getYaw());
    }

    /**
     * Gets the yaw offset from the best visible target
     */
    public Rotation2d getYawToAprilTag(int id) {
        for (PhotonTrackedTarget target : getTargets()) {
            if (target.getFiducialId() == id) return Rotation2d.fromDegrees(target.getYaw());
        }
        return null; // Return null if the particular april tag is not seen by the camera
    }

    /**
     * The calculated 2d field position of the robot based on april tag vision data
     */
    public Pose2d getEstimatedFieldPose(FieldLayout layout) {
        return PhotonUtils.estimateFieldToRobotAprilTag(
            getBestTarget().getBestCameraToTarget(), 
            layout.getAprilTagLayout().getTagPose(
                getBestTarget().getFiducialId()
            ).get(), 
            cameraPoseToRobotPose
        ).toPose2d();
    }
}