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
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.*;


public class OdometryUpdates extends SubsystemBase {

    private Swerve swerve;
    private Vision vision1;
    private Vision vision2;
    private Pose2d initialPose;
    private static boolean seedFromVision;

    public OdometryUpdates(Swerve swerve, Vision vision1, Vision vision2) {
        this.swerve = swerve;
        this.vision1 = vision1;
        this.vision2 = vision2;
    }


    public void periodic() {
        if (RobotBase.isReal()){
            Pose2d robotPosition1 = null;
            double ambiguity1 = -1d;
            double timeStamp1 = 0.0;
            Pose2d robotPosition2 = null;
            double ambiguity2 = -1d;
            double timeStamp2 = 0.0;

            Optional<EstimatedRobotPose> robotPose1 = vision1.getRobotPoseVision();
            Optional<EstimatedRobotPose> robotPose2 = vision2.getRobotPoseVision();
        


            Pose2d combinedPose;
            double combinedTimestamp;
            boolean hasCam1 = false;
            boolean hasCam2 = false;
            boolean twoTagsCam1 = false;
            boolean twoTagsCam2 = false;
            if(robotPose1.isPresent()){
                robotPosition1 = robotPose1.get().estimatedPose.toPose2d();
                timeStamp1 = robotPose1.get().timestampSeconds;
                hasCam1 = true;
                if(vision1.getTargets().size() > 1){
                    twoTagsCam1 = true;
                }
            }
            if(robotPose2.isPresent()){
                robotPosition2 = robotPose2.get().estimatedPose.toPose2d();
                timeStamp2 = robotPose2.get().timestampSeconds;
                hasCam2 = true;
                if(vision2.getTargets().size() > 1){
                    twoTagsCam1 = true;
                }
            }
            if(hasCam1 && hasCam2){
                if(twoTagsCam1 == twoTagsCam2){
                    combinedPose = robotPosition1.interpolate(robotPosition2, 0.5);
                    combinedTimestamp = (timeStamp1 + timeStamp2) / 2;
                }
                else if(twoTagsCam1){ // Must mean one tag cam 2
                    combinedPose = robotPosition1;
                    combinedTimestamp = timeStamp1;
                }
                else{ // Must mean one tag cam 1
                    combinedPose = robotPosition2;
                    combinedTimestamp = timeStamp2;
                }
            }
            else if(hasCam1){ // We don't have cam2
                combinedPose = robotPosition1;
                combinedTimestamp = timeStamp1;
            }
            else{ // We don't have cam1
                combinedPose = robotPosition2;
                combinedTimestamp = timeStamp2;
            }
            boolean pose2PlusTags = twoTagsCam1 || twoTagsCam2;

            //if(Math.abs(ambiguity) < 0.2 && vision.getTargets().size() > 1) {

            if(combinedPose != null){
                if (DriverStation.isDisabled() || (pose2PlusTags)){
                    swerve.resetPose(combinedPose);
                } else {
                    swerve.addVisionMeasurement(combinedPose, combinedTimestamp);
                }
            }


            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView1", vision1.getTargets().size());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView2", vision2.getTargets().size());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose1", robotPosition1);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose2", robotPosition2);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPoseCombined", combinedPose);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentPosition());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio1", ambiguity1);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio2", ambiguity2);
        }
    }

    public static Command seedFromVision(){
        return Commands.runOnce(() -> {seedFromVision = true;});
    }
    
    public void simulationPeriodic() {

    }
    
}
