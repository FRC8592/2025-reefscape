package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Rotation;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    PhotonCamera camera = new PhotonCamera("Arducam_OV9782_E");

    boolean targetVisible = false;
    double targetX = 0.0;
    double targetY = 0.0;
    double targetZ = 0.0;
    double targetPitch = 0.0;
    double targetArea = 0.0;
    double targetXRotation =0d;
    double targetYRotation =0d;
    double targetZRotation =0d;
    
    double targetYawRotation = 0.0;
    double targetPitchRotation = 0.0;
    double targetRollRotation = 0.0;
 
    public Vision(){
        SmartDashboard.putString("hi", "hi");
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
         var results = camera.getAllUnreadResults();
         SmartDashboard.putBoolean("results empty", results.isEmpty());
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
             targetVisible = result.hasTargets();
             if (targetVisible) {
                 // At least one AprilTag was seen by the camera
                 for (PhotonTrackedTarget target : result.getTargets()) {
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
                    targetVisible = true;
                    // target.
                 }
             }
         }
         SmartDashboard.putNumber("Target X Rotation ", targetXRotation);
         SmartDashboard.putNumber("Target Z Rotation ", targetZRotation);
         SmartDashboard.putNumber("Target Y Rotation ", targetYRotation);

         SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
         SmartDashboard.putNumber("Target Pitch", targetPitch);
         SmartDashboard.putNumber("Target Area", targetArea);
         SmartDashboard.putNumber("Target ID", targetId);
         SmartDashboard.putNumber("Target X", targetX);
         SmartDashboard.putNumber("Target Y", targetY);
         SmartDashboard.putNumber("Target Z", targetZ);

         SmartDashboard.putNumber("Target Yaw Rotation", targetYawRotation);
         SmartDashboard.putNumber("Target Pitch Rotation", targetPitchRotation);
         SmartDashboard.putNumber("Target Roll Rotation", targetRollRotation);
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

}
