// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Suppliers;
import frc.robot.Constants.*;
import frc.robot.commands.largecommands.FollowPathCommand;

public class ScoreCoral extends SubsystemBase {

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    private Swerve swerve;

    //The AprilTag target taken from vision
    private Pose2d target;
    int heartbeat = 0;
    
    //These enums are for the setPosition() method that will set the coral scoring level and its respective direction

    public enum LeftOrRight{
        Left,
        Right,;

        public LeftOrRight swap(){
            if(this == LeftOrRight.Left){
                return LeftOrRight.Right;
            }
            else{
                return LeftOrRight.Left;
            }
        }
    };

    private LeftOrRight direction = LeftOrRight.Left;

    public ScoreCoral(Swerve swerve) {
        this.swerve = swerve;
    }

    public void periodic() {
        Pose2d closestTagPos = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(getClosestTag(CORAL_ALIGN.RED_REEF_TAGS)).get().toPose2d();
        Pose2d robotPose = swerve.getCurrentPosition();
        Logger.recordOutput(CORAL_ALIGN.LOG_PATH+"DistanceToNearestTag", Math.sqrt(Math.pow(closestTagPos.getX()-robotPose.getX(), 2) + Math.pow(closestTagPos.getY()-robotPose.getY(), 2)));
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
 
    public Command driveToClosestReefTag(){
        if (Suppliers.isRedAlliance.getAsBoolean()){
            return driveToTag(getClosestTag(CORAL_ALIGN.RED_REEF_TAGS));
        }
        else {
            return driveToTag(getClosestTag(CORAL_ALIGN.BLUE_REEF_TAGS));
        }
    }

    public Command driveToClosestHumanPlayerStation(){
        if (Suppliers.isRedAlliance.getAsBoolean()){
            return driveToTag(getClosestTag(CORAL_ALIGN.RED_HPLAYER_TAGS));
        }
        else {
            return driveToTag(getClosestTag(CORAL_ALIGN.BLUE_HPLAYER_TAGS));
        }
    }


    /**
     * This command takes in a tag number and uses an 
     * odometry-generated trajectory to drive to it
     * @param tag
     * @return a new FollowPathCommand to follow the generated trajectory 
     */
    public Command driveToTag(int tag) {

        Pose2d robotPose = swerve.getCurrentPosition();
        List<Pose2d> waypoints = new ArrayList<Pose2d>();
        Pose2d targetTagPosition = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(tag).get().toPose2d();

        targetTagPosition = generateScoringPose(targetTagPosition, direction);

        Logger.recordOutput("CustomLogs/ScoreCoral/TargetedTagRotation", targetTagPosition.getRotation().getDegrees());
        
        Pose2d targetTagPositionOffset = new Pose2d(new Translation2d(targetTagPosition.getX(), targetTagPosition.getY()), targetTagPosition.getRotation().plus(Rotation2d.fromDegrees(180)));
        double deltaPosition[] = {targetTagPositionOffset.getX()-robotPose.getX(), targetTagPositionOffset.getY()-robotPose.getY()};
        
        //trick the path generator into thinking the robot is always pointing at the tag
        // Pose2d robotPose2 = (
        //     swerve.getCurrentSpeeds().equals(new ChassisSpeeds())
        //     ? new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(Math.atan2(deltaPosition[1],deltaPosition[0])))
        //     : new Pose2d(
        //         robotPose.getTranslation(),
        //         Rotation2d.fromRadians(Math.atan2(swerve.getCurrentSpeeds().vyMetersPerSecond, swerve.getCurrentSpeeds().vxMetersPerSecond))
        //     )
        // );
        Pose2d robotPose2 = new Pose2d(robotPose.getTranslation(), Rotation2d.fromRadians(Math.atan2(deltaPosition[1],deltaPosition[0])));
       
        //create basic tank drive trajectoryswerve
        waypoints.add(robotPose2);
        waypoints.add(targetTagPositionOffset);
        TrajectoryConfig config = SWERVE.PATH_FOLLOW_TRAJECTORY_CONFIG.setStartVelocity(
            Math.sqrt(
                Math.pow(swerve.getCurrentSpeeds().vxMetersPerSecond, 2)
                +Math.pow(swerve.getCurrentSpeeds().vyMetersPerSecond, 2)
            )
        );
        final Trajectory traj = TrajectoryGenerator.generateTrajectory(waypoints, config);

        List<Pose2d> path = new ArrayList<Pose2d>();
        List<State> wp = new ArrayList<State>();

        //swervify trajectory by replacing built in tank drive rotation with custom holonomic lerp-based holonomic rotation system
        traj.getStates().forEach((state) -> {
            
            wp.add(

                new State(
                    state.timeSeconds, 
                    state.velocityMetersPerSecond, 
                    state.accelerationMetersPerSecondSq, 
                    new Pose2d(
                        state.poseMeters.getTranslation(), 
                        robotPose.getRotation().interpolate(
                            targetTagPositionOffset.getRotation(), 
                            (state.timeSeconds)/(traj.getTotalTimeSeconds()/2)
                        )
                    ), 
                    2
                )

            );

        });

        //compile swervified states into an actual trajectory
        Trajectory upTraj = new Trajectory(wp);

        //Make a renderable path for PathPlanner logging
        upTraj.getStates().forEach((state) -> {path.add(state.poseMeters);});

        //Render the path to PathPlanner
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/GeneratedPath", path.toArray(new Pose2d[0]));
                

        //Run path
        return new FollowPathCommand(upTraj, () -> false, "DriveToReef", 0.5, false, true);



    }

    public int getClosestTag(int[] tags) {
        Pose2d robotPose = swerve.getCurrentPosition();
        //this calculates only for each alliance, reduces all the iterative compute for the other 6 tags

        List<Double> distances = new ArrayList<Double>();
        for (int i = 0; i < tags.length; i++) {
            
            Pose2d tagpos = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(tags[i]).get().toPose2d();
            distances.add(Math.sqrt(Math.pow(robotPose.getX()-tagpos.getX(), 2) + Math.pow(robotPose.getY()-tagpos.getY(), 2)));

        }

        int tagID = tags[distances.indexOf(Collections.min(distances))];
        Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/SelectedTag", AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField().getTagPose(tagID).get().toPose2d());
        return tagID;
    }


    public void setPosition(LeftOrRight leftOrRight){
        heartbeat++;
        Logger.recordOutput("CustomLogs/ScoreCoral/LeftOrRight", leftOrRight.name());
        Logger.recordOutput("CustomLogs/ScoreCoral/LeftOrRightHeartbeat", heartbeat);
       direction = leftOrRight;
    //    SmartDashboard.putString("direction", direction.name());
    }

    /**
     * Accepts a pose of a reef tag, calculates the offset to align with the coral branches,
     * and returns a new pose based on the offset
     * @param tagPose The pose of a tag on the reef
     * @param reefScoringSide The side either left or right that we want to score on
     * @return new pose with correct translation and rotation
     */
    
    public Pose2d generateScoringPose(Pose2d tagPose, LeftOrRight reefScoringSide){

        Pose2d newPose;

        double offset = 0;

        if(reefScoringSide == LeftOrRight.Left){
            offset = CORAL_ALIGN.OFFSET_LEFT_METERS;
        } else {
            offset = CORAL_ALIGN.OFFSET_RIGHT_METERS;
        }

        //Using a plus function on a Transform2D will transform the pose or transform it has been called on RELATIVELY.
        // Pose2d.plus(Transform2d) transforms relatively
        // both of these descriptions work, as long as we remember it next time! ツ
        newPose = tagPose.plus(new Transform2d(CORAL_ALIGN.OFFSET_DEPTH, offset, new Rotation2d()));

        Logger.recordOutput("CustomLogs/ScoreCoral/ChosenPose", newPose);

        return newPose;

        
    }
    public LeftOrRight getDirection(){
        return this.direction;
    }
}