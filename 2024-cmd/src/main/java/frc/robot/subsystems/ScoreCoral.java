// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

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

    private int ticks_stopped;
    
    private PIDController xController = new PIDController(CORAL_ALIGN.X_KP, CORAL_ALIGN.X_KI, CORAL_ALIGN.X_KD);
    private PIDController yController = new PIDController(CORAL_ALIGN.Y_KP, CORAL_ALIGN.Y_KI, CORAL_ALIGN.Y_KD);
    private PIDController rotController = new PIDController(CORAL_ALIGN.ROT_KP, CORAL_ALIGN.ROT_KI, CORAL_ALIGN.ROT_KD);

    
    
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
        

    }

    public void periodic() {
        if (DriverStation.isDisabled()){
            Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
            //TrajectoryConfig config = new TrajectoryConfig(SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND, SWERVE.MAX_TRANSLATIONAL_ACCELERATION);
        
            if (robot_pose.isPresent()) {
                Pose2d robotPosition = robot_pose.get().estimatedPose.toPose2d();
                //Trajectory traj = TrajectoryGenerator.generateTrajectory(robotPosition, null, robotPosition, config);
                swerve.resetPose(robotPosition);
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/InitialPose", robotPosition);

            }
        } else {
            Optional<EstimatedRobotPose> robot_pose = vision.getRobotPoseVision();
            //TrajectoryConfig config = new TrajectoryConfig(SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND, SWERVE.MAX_TRANSLATIONAL_ACCELERATION);
            
            if (robot_pose.isPresent()) {
                EstimatedRobotPose robotPosition = robot_pose.get();
                //Trajectory traj = TrajectoryGenerator.generateTrajectory(robotPosition, null, robotPosition, config);
                //var estStdDevs = vision.getEstimationStdDevs();
                swerve.addVisionMeasurement(robotPosition.estimatedPose.toPose2d(), robotPosition.timestampSeconds);
                Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/VisionPose", robotPosition.estimatedPose.toPose2d());
                
                heartbeat++;
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


    public void driveToReef() {
        // Setting the x speed, y speed,rotating speed
        double xSpeed = 0d, ySpeed = 0d, rotSpeed = 0d;
        double yOffset = 0d;


        Logger.recordOutput("CustomLog/Scorecoral/Target Visible", vision.getTargetVisible());

        if (vision.getTargetVisible() == true){
            
            if (direction == LeftOrRight.Left) {
                yOffset = -CORAL_ALIGN.Y_OFFSET_LEFT;
            }
            else {
                yOffset = CORAL_ALIGN.Y_OFFSET_RIGHT;
            }
            
            ySpeed = xController.calculate(vision.getTargetX(), CORAL_ALIGN.X_OFFSET);
            ySpeed = Math.min(CORAL_ALIGN.SPEED_MAX, ySpeed);
            ySpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, ySpeed);

            ySpeed = ySpeed * CORAL_ALIGN.SPEED_SCALE;

            xSpeed = yController.calculate(vision.getTargetY(), yOffset);
            xSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, xSpeed);
            xSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, xSpeed);
    
            xSpeed = xSpeed * CORAL_ALIGN.SPEED_SCALE;
    
            rotSpeed = rotController.calculate(vision.getTargetYaw(), CORAL_ALIGN.ROT_OFFSET);
            rotSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, rotSpeed);
            rotSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, rotSpeed);
    
            rotSpeed = -rotSpeed * CORAL_ALIGN.SPEED_SCALE;
            
            //only horizontal movement while moving to the apriltag
            //if xSpeed greater than ySpeed  
            if (Math.abs(xSpeed) > Math.abs(ySpeed)) {
                xSpeed = 0; 
            } 

            ChassisSpeeds speed = swerve.processJoystickInputs(xSpeed, ySpeed, rotSpeed);
            SmartDashboard.putString("ChassisSpeedJoystick", speed.toString());
            Logger.recordOutput("speed", speed);
            swerve.drive(speed, DriveModes.ROBOT_RELATIVE);
            ticks_stopped = 0;

        } else {
            if (ticks_stopped >= CORAL_ALIGN.MAX_LOCK_LOSS_TICKS) {
                swerve.drive(Swerve.speedZero);
            }
            ticks_stopped += 1;
        }

        SmartDashboard.putNumber("Provided XSpeed", xSpeed);
        SmartDashboard.putNumber("Provided YSpeed", ySpeed);

        heartbeat++;
        Logger.recordOutput("CustomLogs/Scorecoral/heartbeat", heartbeat);
        Logger.recordOutput("CustomLogs/Scorecoral/direction", direction);
            
    }

    public void setPosition(LeftOrRight leftOrRight, ScoreLevels scoreLevel){
       direction = leftOrRight;
       level = scoreLevel;
       SmartDashboard.putString("direction", direction.name());
       SmartDashboard.putString("Level", level.name());


    }

    
}
