// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionCam1;
import frc.robot.Constants;
import frc.robot.Constants.*;


public class OdometryUpdates extends SubsystemBase {

    private Swerve swerve;
    private VisionCam1 vision1;
    private VisionCam1 vision2;
    private Pose2d initialPose;
    private static boolean seedFromVision;

    public OdometryUpdates(Swerve swerve, VisionCam1 vision1, VisionCam1 vision2) {
        this.swerve = swerve;
        this.vision1 = vision1;
        this.vision2 = vision2;
    }


    public void periodic() {
        if (RobotBase.isReal()){
        Pose2d robotPosition1 = new Pose2d();
        double ambiguity1 = -1d;
        double timeStamp1 = 0.0;
        Pose2d robotPosition2 = new Pose2d();
        double ambiguity2 = -1d;
        double timeStamp2 = 0.0;

        Optional<EstimatedRobotPose> robotPose1 = vision1.getRobotPoseVision();
        Optional<EstimatedRobotPose> robotPose2 = vision2.getRobotPoseVision();
        
        if (robotPose1.isPresent()) {
            robotPosition1 = robotPose1.get().estimatedPose.toPose2d();
            ambiguity1 = vision1.getPoseAmbiguityRatio();
            timeStamp1 = robotPose1.get().timestampSeconds;
            robotPosition2 = robotPose2.get().estimatedPose.toPose2d();
            ambiguity2 = vision2.getPoseAmbiguityRatio();
            timeStamp2 = robotPose2.get().timestampSeconds;


            Pose2d combinedPose;
            boolean hasBothcameras;
            boolean has2Tags;
            if(vision1.getTargets().size() > 1){
                if(vision2.getTargets().size() > 1){
                    combinedPose = robotPosition1.interpolate(robotPosition2, 0.5);
                    hasBothcameras = true;
                    has2Tags = true;
                }
                else{
                    combinedPose = robotPosition1;
                    hasBothcameras = false;
                    has2Tags = true;
                }
            }
            else{
                if(vision2.getTargets().size() > 1){
                    combinedPose = robotPosition2;
                    hasBothcameras = false;
                    has2Tags = true;
                }
                else{
                    combinedPose = robotPosition1.interpolate(robotPosition2, 0.5);
                    hasBothcameras = false;
                    has2Tags = false;
                }
            }

            //if(Math.abs(ambiguity) < 0.2 && vision.getTargets().size() > 1) {

            if (DriverStation.isDisabled() || has2Tags){
                swerve.resetPose(combinedPose);
            } else {
                swerve.addVisionMeasurement(robotPosition1, timeStamp1);
            }

        }

        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView", vision.getTargets().size());
        // Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose", robotPosition);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentPosition());
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio", ambiguity);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/InitialPose", initialPose);
        }
    }

    public static Command seedFromVision(){
        return Commands.runOnce(() -> {seedFromVision = true;});
    }
    
    public void simulationPeriodic() {

    }
    
}
