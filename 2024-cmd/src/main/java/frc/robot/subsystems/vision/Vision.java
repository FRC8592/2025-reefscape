package frc.robot.subsystems.vision;    

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;


import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CORAL_ALIGN;
import frc.robot.subsystems.LEDs;
import frc.robot.Robot;

public class Vision extends SubsystemBase{
    PhotonCamera camera;
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    PhotonPoseEstimator estimator;

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

    List<AprilTag> al = new ArrayList<>();

    public Vision(String camName, Transform3d camOffsets){
        camera = new PhotonCamera(camName);
        estimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOffsets);
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

        visionSim.addCamera(cameraSim, camOffsets);

        visionSim.getDebugField();

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);

        cameraSim.enableDrawWireframe(true);
        
        al.add(new AprilTag(1, new Pose3d(657.37, 25.80, 58.5, new Rotation3d(0, 0, Math.toRadians(126)))));
        al.add(new AprilTag(2, new Pose3d(657.37, 291.2, 58.5, new Rotation3d(0, 0, Math.toRadians(234)))));
        al.add(new AprilTag(3, new Pose3d(455.15, 317.15, 51.25, new Rotation3d(0, 0, Math.toRadians(270)))));
        al.add(new AprilTag(4, new Pose3d(365.2, 241.64, 73.54, new Rotation3d(0, Math.toRadians(30), 0))));
        al.add(new AprilTag(5, new Pose3d(365.2, 75.39, 73.54, new Rotation3d(0, Math.toRadians(30), 0))));
        al.add(new AprilTag(6, new Pose3d(530.49, 130.17, 12.13, new Rotation3d(0, 0, Math.toRadians(300)))));
        al.add(new AprilTag(7, new Pose3d(546.87, 158.5, 12.13, new Rotation3d(0, 0, 0))));
        al.add(new AprilTag(8, new Pose3d(530.49, 186.83, 12.13, new Rotation3d(0, 0, Math.toRadians(60)))));
        al.add(new AprilTag(9, new Pose3d(497.77, 186.83, 12.13, new Rotation3d(0, 0, Math.toRadians(120)))));
        al.add(new AprilTag(10, new Pose3d(481.39, 158.5, 12.13, new Rotation3d(0, 0, Math.toRadians(180)))));
        al.add(new AprilTag(11, new Pose3d(467.77, 130.17, 12.13, new Rotation3d(0, 0, Math.toRadians(240)))));
        al.add(new AprilTag(12, new Pose3d(33.51, 25.8, 58.5, new Rotation3d(0, 0, Math.toRadians(54)))));
        al.add(new AprilTag(13, new Pose3d(33.51, 291.2, 58.5, new Rotation3d(0, 0, Math.toRadians(306)))));
        al.add(new AprilTag(14, new Pose3d(325.68, 241.64, 73.54, new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))));
        al.add(new AprilTag(15, new Pose3d(325.68, 75.39, 73.54, new Rotation3d(0, Math.toRadians(30), Math.toRadians(180)))));
        al.add(new AprilTag(16, new Pose3d(235.73, -0.15, 51.25, new Rotation3d(0, 0, Math.toRadians(90)))));
        al.add(new AprilTag(17, new Pose3d(160.39, 130.17, 12.13, new Rotation3d(0, 0, Math.toRadians(240)))));
        al.add(new AprilTag(18, new Pose3d(144, 158.5, 12.13, new Rotation3d(0, 0, Math.toRadians(180)))));
        al.add(new AprilTag(19, new Pose3d(160.39, 186.83, 12.13, new Rotation3d(0, 0, Math.toRadians(120)))));
        al.add(new AprilTag(20, new Pose3d(193.1, 186.83, 12.13, new Rotation3d(0, 0, Math.toRadians(60)))));
        al.add(new AprilTag(21, new Pose3d(209.49, 158.5, 12.13, new Rotation3d(0, 0, 0))));
        al.add(new AprilTag(22, new Pose3d(193.1, 130.17, 12.13, new Rotation3d(0, 0, Math.toRadians(300)))));

        aprilTagFieldLayout = new AprilTagFieldLayout(al,26.4, 57.6); //check units
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
        //  SmartDashboard.putBoolean("results empty", results.isEmpty());
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
            if(camera.isConnected()){
                LEDs.setHasTags(result.getTargets().size());
            }

            else{
                LEDs.setHasTags(-1);
            }
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
        
        //  SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        //  SmartDashboard.putNumber("Target ID", targetId);
        //  SmartDashboard.putNumber("Target Yaw Rotation", targetYawRotation);

        // Logs if the robot sees 1 or sees 2 tags
        SmartDashboard.putBoolean("Has one tag", getTargets().size() > 0);
        SmartDashboard.putBoolean("Has two tags", getTargets().size() > 1);
    }

    public void simulationPeriodic() {
        visionSim.update(Robot.FIELD.getRobotPose());
    }

    /**
     * Gets the target X of the camera.
     * @return Returns the target X of the camera.
     */
    public double getTargetX(){
        return targetX;
    }

    /**
     * Gets the target Y of the camera.
     * @return Returns the target Y of the camera.
     */
    public double getTargetY(){
        return targetY;
    }

    /**
     * Gets the target Z of the camera.
     * @return Returns the target Z of the camera.
     */
    public double getTargetZ(){
        return targetZ;
    }

    /**
     * Gets the target yaw of the camera.
     * @return Returns the target yaw of the camera.
     */
    public double getTargetYaw(){
        return targetYawRotation;
    }

    /**
     * Gets whether the cameras target is visible as a boolean.
     * @return Returns whether the targets camera is visible as a boolean.
     */
    public boolean getTargetVisible(){
        return targetVisible;
    }

    /**
     * Gets the pose ambiguity ratio.
     * @return Returns the pose ambiguity ratio.
     */
    public double getPoseAmbiguityRatio(){
        return targetAmbiguity;
    }

    /**
     * Lists the targets visible by the camera.
     * @return Returns a list of the targets visible by the camera.
     */
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

    /**
     * Gets the current vision pose.
     * @return Returns the current vision pose.
     */
    public Optional<EstimatedRobotPose> getRobotPoseVision() {
       return estimator.update(camera.getLatestResult());
    }
}