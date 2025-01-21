package frc.robot.subsystems;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.Constants.CORAL_ALIGN;
import frc.robot.Constants.VISION;
import lib.team8592.MatchMode;
import lib.team8592.hardware.NewtonPhotonCamera;

public class VisionSubsystem extends NewtonSubsystem {
    private NewtonPhotonCamera camera;
 
    private PIDController xController = new PIDController(CORAL_ALIGN.X_KP, CORAL_ALIGN.X_KI, CORAL_ALIGN.X_KD);
    private PIDController yController = new PIDController(CORAL_ALIGN.Y_KP, CORAL_ALIGN.Y_KI, CORAL_ALIGN.Y_KD);
    private PIDController rotController = new PIDController(CORAL_ALIGN.ROT_KP, CORAL_ALIGN.ROT_KI, CORAL_ALIGN.ROT_KD);

    public VisionSubsystem(boolean logToShuffleboard){
        super(logToShuffleboard);

        this.camera = new NewtonPhotonCamera(// TODO - Change to k2025Reefscape
            VISION.CAM_NAME, 
            VISION.CAMERA_OFFSET, 
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            Robot.isSimulation()
        );

        // this.logger.addSendable("Vision Simulated Field", this.newtonCamera.getSimulation().getDebugField());
    }

    public boolean isAnyTargetVisible() {
        return camera.isAnyTargetVisible();
    }

    public ChassisSpeeds driveToReef(double offset) {
        PhotonTrackedTarget tag = camera.getBestTarget();
        if (tag == null) {
            return new ChassisSpeeds();
        }

        Transform3d cameraToTarget = tag.getBestCameraToTarget();

        double x = xController.calculate(cameraToTarget.getX(), offset);
        double y = yController.calculate(cameraToTarget.getY(), 0);
        double rot = rotController.calculate(cameraToTarget.getRotation().getAngle(), 0);
        return new ChassisSpeeds(x, y, rot);
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Target Visible", camera.isAnyTargetVisible());
        
        PhotonTrackedTarget bestTarget = camera.getBestTarget();

        if (camera.isAnyTargetVisible()) {
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
        this.camera.updateSim(new Pose3d(Robot.FIELD.getRobotPose()));
    }

    @Override
    public void onInit(MatchMode mode) {}

    @Override
    public void stop() {}
}