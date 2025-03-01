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
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants;
import frc.robot.Constants.*;


public class OdometryUpdates extends SubsystemBase {

    private Swerve swerve;
    private Vision vision;
    private Pose2d initialPose;

    public OdometryUpdates(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;
    }


    public void periodic() {
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
            if(Math.abs(ambiguity) < Constants.NAVIGATION.MAX_ACCEPTABLE_AMBIGUITY || vision.getTargets().size() > 1) {
            
                if (DriverStation.isDisabled()){
                    initialPose = robotPosition;
                    swerve.resetPose(initialPose);        
                } else {
                    swerve.addVisionMeasurement(robotPosition, timeStamp);
                }
            }

        }

        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView", vision.getTargets().size());
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose", robotPosition);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentPosition());
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio", ambiguity);
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/InitialPose", initialPose);
        }
    }

    
    public void simulationPeriodic() {

    }
    
}
