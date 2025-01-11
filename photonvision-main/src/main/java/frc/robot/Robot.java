// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.photonvision.*;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

    PhotonCamera camera = new PhotonCamera("Arducam_OV9782_D");

    public static Field2d FIELD = new Field2d();

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
         // Calculate drivetrain commands from Joystick values
        //  double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
        //  double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
        //  double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;
 
         // Read in relevant data from the Camera
         boolean targetVisible = false;
         double targetYaw = 0.0;
         double targetPitch = 0.0;
         double targetArea = 0.0;
         double targetSkew = 0.0;
         int targetId = 0;
         Transform3d bestCameraToTarget = new Transform3d();
         var results = camera.getAllUnreadResults();
         SmartDashboard.putBoolean("results empty", results.isEmpty());
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
             if (result.hasTargets()) {
                 // At least one AprilTag was seen by the camera
                 for (var target : result.getTargets()) {
                    targetYaw = target.getYaw();
                    targetPitch = target.getPitch();
                    targetArea = target.getArea();
                    targetSkew = target.getSkew();
                    targetId = target.getFiducialId();
                    bestCameraToTarget = target.getBestCameraToTarget();
                    targetVisible = true;
                 }
             }
         }
         SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
         SmartDashboard.putNumber("Target Yaw", targetYaw);
         SmartDashboard.putNumber("Target Pitch", targetPitch);
         SmartDashboard.putNumber("Target Area", targetArea);
         SmartDashboard.putNumber("Target Skew", targetSkew);
         SmartDashboard.putNumber("Target ID", targetId);
         SmartDashboard.putNumber("Target X", bestCameraToTarget.getX());
         SmartDashboard.putNumber("Target Y", bestCameraToTarget.getY());
         SmartDashboard.putNumber("Target Z", bestCameraToTarget.getZ());
 
         // Auto-align when requested
        //  if (controller.getAButton() && targetVisible) {
        //      // Driver wants auto-alignment to tag 7
        //      // And, tag 7 is in sight, so we can turn toward it.
        //      // Override the driver's turn command with an automatic one that turns toward the tag.
        //      turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
        //  }
 
        //  // Command drivetrain motors based on target speeds
        //  drivetrain.drive(forward, strafe, turn);
 
         // Put debug information to the dashboard
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
