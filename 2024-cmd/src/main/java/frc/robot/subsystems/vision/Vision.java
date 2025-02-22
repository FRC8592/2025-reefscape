package frc.robot.subsystems.vision;    

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CORAL_ALIGN;
import frc.robot.Robot;

public class Vision extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("Arducam_OV9782_B");
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    PhotonPoseEstimator estimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, CORAL_ALIGN.CAMERA_OFFSETS);

    boolean targetVisible = false;
    double targetX = 0.0;
    double targetY = 0.0;
    double targetZ = 0.0;
    double targetPitch = 0.0;
    double targetArea = 0.0;
    double targetXRotation =0d;
    double targetYRotation =0d;
    double targetZRotation =0d;
    double targetAmbiguity = 0.0;
    double targetYawRotation = 0.0;
    double targetPitchRotation = 0.0;
    double targetRollRotation = 0.0;
    List<PhotonPipelineResult> results;

    VisionSystemSim visionSim;
    SimCameraProperties cameraBProperties;
    PhotonCameraSim cameraSim;

    public Vision(){
        visionSim = new VisionSystemSim("photonvision");

        visionSim.addAprilTags(aprilTagFieldLayout);

        cameraBProperties = new SimCameraProperties();

        // A 1280 x 800 camera with a 100 degree diagonal FOV.
        cameraBProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in pixels.
        cameraBProperties.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop rate).
        cameraBProperties.setFPS(90);
        // The average and standard deviation in milliseconds of image data latency.
        cameraBProperties.setAvgLatencyMs(35);
        cameraBProperties.setLatencyStdDevMs(5);

        cameraSim = new PhotonCameraSim(camera, cameraBProperties);

        visionSim.addCamera(cameraSim, CORAL_ALIGN.CAMERA_OFFSETS);

        visionSim.getDebugField();

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);
    }

    @Override
    public void periodic(){
        // Calculate drivetrain commands from Joystick values
        //  double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
        //  double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
        //  double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;
 
         // Read in relevant data from the Camera
         
         int targetId = 0;
         Transform3d bestCameraToTarget = new Transform3d();
         results = camera.getAllUnreadResults();
         SmartDashboard.putBoolean("results empty", results.isEmpty());
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
             targetVisible = result.hasTargets();
             if (targetVisible) {
                // At least one AprilTag was seen by the camera
                PhotonTrackedTarget target = result.getBestTarget();
                
                targetAmbiguity = target.getPoseAmbiguity();
                targetPitch = target.getPitch();
                targetArea = target.getArea();
                targetId = target.getFiducialId();
                bestCameraToTarget = target.getBestCameraToTarget();
                Rotation3d targetRotation = bestCameraToTarget.getRotation();
                targetXRotation = targetRotation.getX();
                targetYRotation = targetRotation.getY();
                targetZRotation = targetRotation.getZ();

                targetYawRotation = targetRotation.getMeasureZ().baseUnitMagnitude()*(180/Math.PI);
                targetPitchRotation = targetRotation.getMeasureY().baseUnitMagnitude()*(180/Math.PI);
                targetRollRotation = targetRotation.getMeasureX().baseUnitMagnitude()*(180/Math.PI);

                    if (targetYawRotation > 0){
                        targetYawRotation -= 180;
                    }
                    else{
                        targetYawRotation += 180;
                    }
                    
                    targetX = bestCameraToTarget.getX();
                    targetY = bestCameraToTarget.getY();
                    targetZ = bestCameraToTarget.getZ();
                    
                 }
             }
        
         SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
         SmartDashboard.putNumber("Target ID", targetId);
         SmartDashboard.putNumber("Target Yaw Rotation", targetYawRotation);
    }

    public void simulationPeriodic() {
        visionSim.update(Robot.FIELD.getRobotPose());
    }

    public double getTargetX(){
        return targetX;
    }

    public double getTargetY(){
        return targetY;
    }

    public double getTargetZ(){
        return targetZ;
    }

    public double getTargetYaw(){
        return targetYawRotation;
    }

    public boolean getTargetVisible(){
        return targetVisible;
    }

    public double getPoseAmbiguityRatio(){
        return targetAmbiguity;
    }

    public List<PhotonTrackedTarget> getTargets() {
        return camera.getLatestResult().getTargets();
    }

    //actually PhotonTrackedTarget
    public int getClosestTagID() {

        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            List<Double> distances = new ArrayList<Double>();
            

            targets.forEach(
                (target) -> {
                    distances.add(Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetX, 2)));
                }
            );


            return targets.get(distances.indexOf(Collections.min(distances))).getFiducialId();
        }
        else {
            return -1;
        }

    }

    public Optional<EstimatedRobotPose> getRobotPoseVision() {
       return estimator.update(camera.getLatestResult());

       
        
    }

    



}