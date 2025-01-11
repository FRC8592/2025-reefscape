package frc.robot.subsystems.vision;

import org.photonvision.*;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Vision implements Subsystem{
    PhotonCamera camera = new PhotonCamera("Arducam_OV9782_D");

    public Vision(){
    }

    public void periodic(){
        // Read in relevant data from the Camera
         boolean targetVisible = false;
         double targetYaw = 0.0;
         double targetPitch = 0.0;
         double targetArea = 0.0;
         double targetSkew = 0.0;
         int targetId = 0;
         Transform3d bestCameraToTarget = new Transform3d();
         var results = camera.getAllUnreadResults();
         SmartDashboard.putBoolean("results empty", results.isEmpty());
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
             if (result.hasTargets()) {
                 // At least one AprilTag was seen by the camera
                 for (var target : result.getTargets()) {
                    targetYaw = target.getYaw();
                    targetPitch = target.getPitch();
                    targetArea = target.getArea();
                    targetSkew = target.getSkew();
                    targetId = target.getFiducialId();
                    bestCameraToTarget = target.getBestCameraToTarget();
                    targetVisible = true;
                 }
             }
         }
         SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
         SmartDashboard.putNumber("Target Yaw", targetYaw);
         SmartDashboard.putNumber("Target Pitch", targetPitch);
         SmartDashboard.putNumber("Target Area", targetArea);
         SmartDashboard.putNumber("Target Skew", targetSkew);
         SmartDashboard.putNumber("Target ID", targetId);
         SmartDashboard.putNumber("Target X", bestCameraToTarget.getX());
         SmartDashboard.putNumber("Target Y", bestCameraToTarget.getY());
         SmartDashboard.putNumber("Target Z", bestCameraToTarget.getZ());
    }

}
