package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.Robot;
import frc.robot.Constants.VISION;
import lib.team8592.MatchMode;
import lib.team8592.hardware.NewtonPhotonCamera;

public class VisionSubsystem extends NewtonSubsystem {
    private NewtonPhotonCamera newtonCamera;
 
    public VisionSubsystem(boolean logToShuffleboard){
        super(logToShuffleboard);

        this.newtonCamera = new NewtonPhotonCamera(// TODO - Change to k2025Reefscape
            VISION.CAM_NAME, 
            VISION.CAMERA_OFFSET, 
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            Robot.isSimulation()
        );

        // this.logger.addSendable("Vision Simulated Field", this.newtonCamera.getSimulation().getDebugField());
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Target Visible", newtonCamera.isAnyTargetVisible());
        
        PhotonTrackedTarget bestTarget = newtonCamera.getBestTarget();

        if (newtonCamera.isAnyTargetVisible()) {
            this.logger.log("Best Target Yaw", bestTarget.yaw);
            this.logger.log("Best Target Pitch", bestTarget.pitch);
            this.logger.log("Best Target Skew", bestTarget.skew);
            this.logger.log("Best Target ID", bestTarget.getDetectedObjectClassID());
            this.logger.log("Best Target X", bestTarget.getBestCameraToTarget().getX());
            this.logger.log("Best Target Y", bestTarget.getBestCameraToTarget().getY());
            this.logger.log("Best Target Z", bestTarget.getBestCameraToTarget().getZ());
        }
    }

    @Override
    public void simulationPeriodic() {
        this.newtonCamera.updateSim(new Pose3d(Robot.FIELD.getRobotPose()));
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void stop() {}
}