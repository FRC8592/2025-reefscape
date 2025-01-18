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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
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

    private PIDController xController = new PIDController(CORAL_ALIGN.X_KP, CORAL_ALIGN.X_KI, CORAL_ALIGN.X_KD);

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

        //Button 1 is Y/Δ
        //Button 2 is B/O
        //Button 3 is A/X
        //Button 4 is X/□
        //Button 5 is LB/L1
        //Button 6 is RB/R1
        //Button 7 is LT/L2
        //Button 8 is RT/R2
        //Button 9 is SELECT
        //Button 10 is START
        //Button 13 is PS/HOME
        //There isn't a button 11 or button 12
        Logger.recordOutput("HID_B1", robotContainer.coralController.getRawButton(1));
        Logger.recordOutput("HID_B2", robotContainer.coralController.getRawButton(2));
        Logger.recordOutput("HID_B3", robotContainer.coralController.getRawButton(3));
        Logger.recordOutput("HID_B4", robotContainer.coralController.getRawButton(4));
        Logger.recordOutput("HID_B5", robotContainer.coralController.getRawButton(5));
        Logger.recordOutput("HID_B6", robotContainer.coralController.getRawButton(6));
        Logger.recordOutput("HID_B7", robotContainer.coralController.getRawButton(7));
        Logger.recordOutput("HID_B8", robotContainer.coralController.getRawButton(8));
        Logger.recordOutput("HID_B9", robotContainer.coralController.getRawButton(9));
        Logger.recordOutput("HID_B10", robotContainer.coralController.getRawButton(10));
        Logger.recordOutput("HID_B13", robotContainer.coralController.getRawButton(13));

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
        double xSpeed = 0d, ySpeed = 0d, rotSpeed = 0d;

        if(robotContainer.vision.getTargetVisible() == true){
            ySpeed = xController.calculate(robotContainer.vision.getTargetX(), CORAL_ALIGN.X_OFFSET);
            ySpeed = Math.min(0.6, ySpeed);
            ySpeed = Math.max(-0.6, ySpeed);

            //robot goes side-to-side
            if(robotContainer.vision.getTargetY() > 0.1){
                xSpeed = -0.15;
            }
            else if(robotContainer.vision.getTargetY() < -0.1){
                xSpeed = 0.15;
            }
            else {
                xSpeed = 0;
            }

            if(robotContainer.vision.getTargetYaw() > 10){
                rotSpeed = -0.15;
            }
            else if(robotContainer.vision.getTargetYaw() < -10){
                rotSpeed = 0.15;
            }
            else {
                rotSpeed = 0;
            }

        //     ChassisSpeeds speed = robotContainer.swerve.processJoystickInputs(0, ySpeed, 0);
        //     SmartDashboard.putString("ChassisSpeedJoystick", speed.toString());
        //     robotContainer.swerve.drive(speed);

        // } else {
        //     robotContainer.swerve.drive(Swerve.speedZero);
        // }

        SmartDashboard.putNumber("Provided XSpeed", xSpeed);
        SmartDashboard.putNumber("Provided YSpeed", ySpeed);
        }
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
