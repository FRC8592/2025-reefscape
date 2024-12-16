// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public static Field2d FIELD = new Field2d();

    //Creating cameras
    PhotonCamera forwardCamera = new PhotonCamera(VISION.FORWARD_CAMERA);
    PhotonCamera backCamera = new PhotonCamera(VISION.BACK_CAMERA);

    // The field from AprilTagFields will be different depending on the game.
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    // Construct PhotonPoseEstimators
    PhotonPoseEstimator forwardPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, forwardCamera, VISION.forwardTransform);
    PhotonPoseEstimator backPhotonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, VISION.backTransform);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Logger.recordMetadata("Game", "Crescendo");
        Logger.recordMetadata("Year", "2024");
        Logger.recordMetadata("Robot", "Zenith");
        Logger.recordMetadata("Team", "8592");

        if (isReal()) { // If running on a real robot
            String time = DateTimeFormatter.ofPattern("yy-MM-dd_HH-mm-ss").format(LocalDateTime.now());
            String path = "/U/"+time+".wpilog";
            Logger.addDataReceiver(new WPILOGWriter(path)); // Log to a USB stick
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            LoggedPowerDistribution.getInstance(1, ModuleType.kRev);// Enables power distribution logging
            Logger.start();
        }
        else { // If simulated
            SmartDashboard.putData(FIELD);
        }
        robotContainer = new RobotContainer();

        //setting camera pipelines for PhotonVision
        forwardCamera.setPipelineIndex(VISION.APRILTAG_PIPELINE);
        backCamera.setPipelineIndex(VISION.APRILTAG_PIPELINE);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        PhotonPipelineResult forwardResult = forwardCamera.getLatestResult();
        boolean forwardHasTargets = forwardResult.hasTargets();
        SmartDashboard.putBoolean("Forward Camera Has Target: ", forwardHasTargets);

        PhotonPipelineResult backResult = backCamera.getLatestResult();
        boolean backHasTargets = backResult.hasTargets();
        SmartDashboard.putBoolean("Back Camera Has Target: ", backHasTargets);

        Optional<EstimatedRobotPose> forwardPose = forwardPhotonPoseEstimator.update();
        Optional<EstimatedRobotPose> backPose = backPhotonPoseEstimator.update();
        if(forwardPose.isPresent()){
            Logger.recordOutput("CustomLogs/forwardPose", forwardPose.get().estimatedPose);
        }
        if(backPose.isPresent()){
            Logger.recordOutput("CutomLogs/backPose", backPose.get().estimatedPose);
        }
            
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
