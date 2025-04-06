// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.*;


public class OdometryUpdates extends SubsystemBase {

    private Swerve swerve;
    private Vision vision1;
    // private Vision vision2;
    private Pose2d initialPose;
    private static boolean useVision;
    private static LeftOrRight leftOrRight;

    public OdometryUpdates(Swerve swerve, Vision vision1) {
        this.swerve = swerve;
        this.vision1 = vision1;
        // this.vision2 = vision2;
    }


    public void periodic() {
        // if (RobotBase.isReal()){
        // Pose2d robotPosition1 = new Pose2d();
        // double ambiguity1 = -1d;
        // double timeStamp1 = 0.0;

        // Optional<EstimatedRobotPose> robotPose1 = vision1.getRobotPoseVision();

        // Pose2d robotPosition2 = new Pose2d();
        // double ambiguity2 = -1d;
        // double timeStamp2 = 0.0;

        // Optional<EstimatedRobotPose> robotPose2 = vision1.getRobotPoseVision();
        
        // if (robotPose1.isPresent()) {
        //     robotPosition1 = robotPose1.get().estimatedPose.toPose2d();
        //     ambiguity1 = vision1.getPoseAmbiguityRatio();
        //     timeStamp1 = robotPose1.get().timestampSeconds;

        // }
        // if (robotPose2.isPresent()) {
        //     robotPosition2 = robotPose2.get().estimatedPose.toPose2d();
        //     ambiguity2 = vision2.getPoseAmbiguityRatio();
        //     timeStamp2 = robotPose2.get().timestampSeconds;

        // }
        // useVision = false;
        // if(useVision){
            
        //     switch (leftOrRight) {
        //         case Right:
        //             if(robotPose1.isPresent() && ambiguity1 < NAVIGATION.MAX_ACCEPTABLE_AMBIGUITY){
        //                 swerve.resetPose(robotPosition1);
        //             }
        //             else{
        //                 if(robotPose2.isPresent()){
        //                     swerve.addVisionMeasurement(robotPosition2, timeStamp2);
        //                 }
        //             }
        //             break;
            
        //         case Left:
        //             if(robotPose2.isPresent() && ambiguity2 < NAVIGATION.MAX_ACCEPTABLE_AMBIGUITY){
        //                 swerve.resetPose(robotPosition2);
        //             }
        //             else{
        //                 if(robotPose1.isPresent()){
        //                     swerve.addVisionMeasurement(robotPosition1, timeStamp1);
        //                 }
        //             }
        //             break;
        //     }
        // }
        // else{
        //     if(DriverStation.isDisabled()){
        //         if(robotPose1.isPresent()){
        //             swerve.resetPose(robotPosition1);
        //         }
        //     }
        //     else{
        //         if(robotPose1.isPresent()){
        //             swerve.addVisionMeasurement(robotPosition1, timeStamp1);
        //         }
        //         if(robotPose2.isPresent()){
        //             swerve.addVisionMeasurement(robotPosition2, timeStamp2);
        //         }
        //     }
        // }
        // if(
        //     vision1.getTargets().size() > 1 || (
        //         Math.abs(ambiguity1) < Constants.NAVIGATION.MAX_ACCEPTABLE_AMBIGUITY
        //         && vision1.getTargets().get(0).bestCameraToTarget.getX() < CORAL_ALIGN.REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE
        //     )
        // ) {
        //     if (DriverStation.isDisabled() || (useVision && Math.abs(ambiguity1) < 0.1)){
        //         useVision = false;
        //         initialPose = robotPosition1;
        //         swerve.resetPose(initialPose);
        //     } else {
                
        //     }
        // }
        // runVision(vision2);
        runVision(vision1);
    }

    
    public void simulationPeriodic() {

    }
    public static void setVision(ScoreCoral scoreCoral){
        useVision = true;
        leftOrRight = scoreCoral.getDirection();
    }

    public void runVision(Vision vision){
        if (RobotBase.isReal()){
            Pose2d robotPosition = new Pose2d();
            double ambiguity = -1d;
            double timeStamp = 0.0;
    
            Optional<EstimatedRobotPose> robotPose = vision.getRobotPoseVision();
            
            if (robotPose.isPresent()) {
                robotPosition = robotPose.get().estimatedPose.toPose2d();
                ambiguity = vision.getPoseAmbiguityRatio();
                timeStamp = robotPose.get().timestampSeconds;
    
                //if(Math.abs(ambiguity) < 0.2 && vision.getTargets().size() > 1) {
                if(
                    vision.getTargets().size() > 1 || (
                        Math.abs(ambiguity) < Constants.NAVIGATION.MAX_ACCEPTABLE_AMBIGUITY
                        && vision.getTargets().get(0).bestCameraToTarget.getX() < CORAL_ALIGN.REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE
                    )
                ) {
                    if (DriverStation.isDisabled()){
                        initialPose = robotPosition;
                        swerve.resetPose(initialPose);
                    } else {
                        swerve.addVisionMeasurement(robotPosition, timeStamp);
                    }
                }
    
            }

            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView1", vision1.getTargets().size());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose1", robotPosition);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentPosition());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio1", ambiguity);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/InitialPose", initialPose);
        }
    }
    
}