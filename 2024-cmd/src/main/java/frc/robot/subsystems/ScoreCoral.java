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
import java.util.Queue;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Suppliers;
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

    public enum ReefPositions{
        //The positions goes from the side facing the driver being south and going clockwise on a compass
        South(CORAL_ALIGN.SOUTH_BLUE_POSE, CORAL_ALIGN.SOUTH_RED_POSE),
        SouthWest(CORAL_ALIGN.SOUTH_WEST_BLUE_POSE, CORAL_ALIGN.SOUTH_WEST_RED_POSE),
        NorthWest(CORAL_ALIGN.NORTH_WEST_BLUE_POSE, CORAL_ALIGN.NORTH_WEST_RED_POSE),
        North(CORAL_ALIGN.NORTH_BLUE_POSE, CORAL_ALIGN.NORTH_RED_POSE),
        NorthEast(CORAL_ALIGN.NORTH_EAST_BLUE_POSE, CORAL_ALIGN.NORTH_EAST_RED_POSE),
        SouthEast(CORAL_ALIGN.SOUTH_EAST_BLUE_POSE, CORAL_ALIGN.SOUTH_EAST_RED_POSE);

        //Data fields for the which side the robot is on
        public Pose2d bluePosition;
        public Pose2d redPosition;
        private ReefPositions(Pose2d bluePosition, Pose2d redPosition){
            this.bluePosition = bluePosition;
            this.redPosition = redPosition;
        }

        /**
         * @param none
         * Uses robotRunningOnRed, if it is true it returns the redPosition
         * else it returns bluePosition
         * @return redPosition or bluePosition
         */
        public Pose2d getReefPosition(){
            if (Suppliers.robotRunningOnRed.getAsBoolean()){
                return redPosition;
            }
            else{
                return bluePosition;
            }
        }
    }

    private LeftOrRight direction = LeftOrRight.Left;
    private ScoreLevels level = ScoreLevels.Level1;
    private ReefPositions position = ReefPositions.South;
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

    public Command driveToReefOdometry() {

        Pose2d robotPose = swerve.getCurrentPosition();
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d targetReefPosition = position.getReefPosition();//AprilTagFields.k2025Reefscape.loadAprilTagLayoutField().getTagPose(18).get().toPose2d();
        Pose2d targetReefPositionOffset = new Pose2d(new Translation2d(targetReefPosition.getX()-0.5, targetReefPosition.getY()), targetReefPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        waypoints.add(robotPose);
        waypoints.add(targetReefPositionOffset);
        final Trajectory traj = TrajectoryGenerator.generateTrajectory(waypoints, SWERVE.PATH_FOLLOW_TRAJECTORY_CONFIG);

        // State start = new State(0, 0, 1, robotPose, 0);
        // State end = new State(traj.getTotalTimeSeconds()-0.25,0, -1, targetReefPositionOffset.transformBy(new Transform2d(new Translation2d(-0.75, 0), new Rotation2d())), 0);

        //traj = new Trajectory(List.of(start, end));

        List<Pose2d> path = new ArrayList<Pose2d>();
        List<State> wp = new ArrayList<State>();


        
        
        traj.getStates().forEach((state) -> {wp.add(new State(state.timeSeconds, state.velocityMetersPerSecond, state.accelerationMetersPerSecondSq, new Pose2d(state.poseMeters.getTranslation(), robotPose.getRotation().interpolate(targetReefPositionOffset.getRotation(), state.timeSeconds/traj.getTotalTimeSeconds())), 2));});
        Trajectory upTraj = new Trajectory(wp);
        upTraj.getStates().forEach((state) -> {path.add(state.poseMeters);});

        Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/GeneratedPath", path.toArray(new Pose2d[0]));

        return new FollowPathCommand(upTraj, () -> false);

    }

    public double Lerp(double time, double start, double end) {


        return ((start*(end-start)) * time);

    }

    public void setPosition(LeftOrRight leftOrRight, ScoreLevels scoreLevel, ReefPositions reefPosition){
       direction = leftOrRight;
       level = scoreLevel;
       position = reefPosition;
       SmartDashboard.putString("direction", direction.name());
       SmartDashboard.putString("Level", level.name());
       SmartDashboard.putString("Position", position.name());
    }

    
}
