// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import javax.xml.xpath.XPathNodes;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.*;
import frc.robot.commands.largecommands.FollowPathCommand;

public class ScoreCoral extends SubsystemBase {

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    private Swerve swerve;
    private Vision vision;
    
    private PIDController xController = new PIDController(CORAL_ALIGN.X_KP, CORAL_ALIGN.X_KI, CORAL_ALIGN.X_KD);
    private PIDController yController = new PIDController(CORAL_ALIGN.Y_KP, CORAL_ALIGN.Y_KI, CORAL_ALIGN.Y_KD);
    private PIDController rotController = new PIDController(CORAL_ALIGN.ROT_KP, CORAL_ALIGN.ROT_KI, CORAL_ALIGN.ROT_KD);

    //The AprilTag target taken from vision
    private Pose2d target;
    
    //These enums are for the setPosition() method that will set the coral scoring level and its respective direction

    public enum ScoreLevels{
        Level1,
        Level2,
        Level3,
        Level4,
    
    };

    public enum LeftOrRight{
        Left,
        Right
    };

    private LeftOrRight direction = LeftOrRight.Left;
    private ScoreLevels level = ScoreLevels.Level1;
    private int heartbeat = 0;

    public ScoreCoral(Swerve swerve, Vision vision) {
        this.swerve = swerve;
        this.vision = vision;

    }

    public void initialize() {  
        
        Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
    
        if (robot_pose.isPresent()) {
            Pose2d robotPosition = robot_pose.get().estimatedPose.toPose2d();
            swerve.resetPose(robotPosition);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/InitialPose", robotPosition);

        }

    }

    public void periodic() {
        if (DriverStation.isDisabled()){
            Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
        
            if (robot_pose.isPresent()) {
                Pose2d robotPosition = robot_pose.get().estimatedPose.toPose2d();
                double ambiguity = vision.getPoseAmbiguityRatio();
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/AmbiguityRatio", ambiguity);
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/TagsInView", vision.getTargets().size());
                if(Math.abs(ambiguity) < 0.2 && vision.getTargets().size() > 1) {
                    swerve.resetPose(robotPosition);
                    Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/InitialPose", robotPosition);
                }

            }
        } else {
            Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
            
            if (robot_pose.isPresent()) {
                EstimatedRobotPose robotPosition = robot_pose.get();

                if (vision.getTargets().size() > 1) {
                    swerve.addVisionMeasurement(robotPosition.estimatedPose.toPose2d(), robotPosition.timestampSeconds);
                }

                heartbeat++;
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/TagsInView", vision.getTargets().size());
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/VisionPose", robotPosition.estimatedPose.toPose2d());
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/Hearbeat", heartbeat);
            }
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/SwervePosition", swerve.getCurrentPosition());
        }

        
    }

    public void simulationPeriodic() {

    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    public void stop(){
        drive(new ChassisSpeeds());
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds){
        // TODO: implement something that allows the commented code to work
        swerve.drive(speeds);
    } 

    public void setTarget(Pose2d target){
        this.target = target;
    }

    public Pose2d getTarget(){
        return target;
    }

    //TODO complete writing the method to compute distance
    public double getDistance(){
        return 0.0;
    }

    public void driveToReef(Pose2d target) {
        // Setting the x speed, y speed,rotating speed
        double xSpeed = 0d, ySpeed = 0d, rotSpeed = 0d;

        Pose2d currentPosition = swerve.getCurrentPosition();
        double xDistance = target.getX() - currentPosition.getX();
        double yDistance = target.getY() - currentPosition.getY();
        //TODO ensure if correct
        double rotDistance = target.getRotation().getDegrees() - currentPosition.getRotation().getDegrees();

        Logger.recordOutput("CustomLog/Scorecoral/Target", target);
        
        xSpeed = xController.calculate(xDistance);
        xSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, xSpeed);
        xSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, xSpeed);

        xSpeed = xSpeed * CORAL_ALIGN.SPEED_SCALE;

        ySpeed = yController.calculate(yDistance);
        ySpeed = Math.min(CORAL_ALIGN.SPEED_MAX, ySpeed);
        ySpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, ySpeed);

        ySpeed = ySpeed * CORAL_ALIGN.SPEED_SCALE;

        rotSpeed = rotController.calculate(rotDistance);
        rotSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, rotSpeed);
        rotSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, rotSpeed);

        rotSpeed = -rotSpeed * CORAL_ALIGN.SPEED_SCALE;

        ChassisSpeeds speed = swerve.processJoystickInputs(xSpeed, ySpeed, rotSpeed);
        SmartDashboard.putString("ChassisSpeedJoystick", speed.toString());
        Logger.recordOutput("speed", speed);
        swerve.drive(speed, DriveModes.FIELD_RELATIVE);

        SmartDashboard.putNumber("Provided XSpeed", xSpeed);
        SmartDashboard.putNumber("Provided YSpeed", ySpeed);
        SmartDashboard.putNumber("Provided RotSpeed", rotSpeed);

    }

    public Command driveToReefVision() {


        Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
        List<Pose2d> waypoints = new ArrayList<Pose2d>();

        if (robot_pose.isPresent()) {
            Pose2d robotPosition = robot_pose.get().estimatedPose.toPose2d();
            Pose2d tag18Position = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTagPose(18).get().toPose2d();

            waypoints.add(robotPosition);
            waypoints.add(tag18Position);
        }
        return new FollowPathCommand(TrajectoryGenerator.generateTrajectory(waypoints, SWERVE.PATH_FOLLOW_TRAJECTORY_CONFIG), () -> false);

    }

    public void setPosition(LeftOrRight leftOrRight, ScoreLevels scoreLevel){
       direction = leftOrRight;
       level = scoreLevel;
       SmartDashboard.putString("direction", direction.name());
       SmartDashboard.putString("Level", level.name());


    }

    
}
