// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import frc.robot.*;
import frc.robot.helpers.*;
import frc.robot.Constants.*;

public class Swerve extends SubsystemBase {
    /**
     * Small enum to control whether to drive robot- or field-
     * relative for {@link Swerve#drive(ChassisSpeeds, DriveModes)}
     */
    public enum DriveModes{
        /** Drive robot-relative */
        ROBOT_RELATIVE,
        /** Switch between robot- and field-relative depending on driver input */
        AUTOMATIC,
        /** Drive field-relative */
        FIELD_RELATIVE
    }

    private PIDController snapToController;

    private boolean isSlowMode;
    private boolean robotRelative;

    private SmoothingFilter smoothingFilter;
    
    private CTRESwerveWrapper swerve;

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    public Swerve() {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD);

        // TODO: Any initialization code needed for the new swerve stuff
        swerve = new CTRESwerveWrapper();
    }

    public void periodic() {
        // TODO: Periodic logging
        swerve.periodic();
    }

    public void simulationPeriodic() {
        Pose2d pose = getCurrentPosition();
        Robot.FIELD.setRobotPose(pose==null?new Pose2d():pose);
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    public void stop(){
        drive(new ChassisSpeeds());
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, field-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds){
        // TODO: implement something that allows the commented code to work
        swerve.drive(speeds, false);
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds, DriveModes mode){
        // TODO: implement something that allows the commented code to work'
         swerve.drive(
             speeds,
             switch(mode){
                 case FIELD_RELATIVE:
                     yield true;
                 case AUTOMATIC:
                     yield !robotRelative;
                 case ROBOT_RELATIVE:
                     yield false;
             }
         );
    }

    /**
     * Set whether human-input-processed joystick input should be slowed
     *
     * @param slowMode whether to slow the drivetrain
     */
    public void setSlowMode(boolean slowMode){
        this.isSlowMode = slowMode;
    }

    /**
     * Set whether human-input-processed joystick input should be robot-relative
     * (as opposed to field-relative)
     *
     * @param robotRelative whether to run the drivetrain robot-relative
     */
    public void setRobotRelative(boolean robotRelative){
        this.robotRelative = robotRelative;
    }

    /**
     * Define whatever direction the robot is facing as forward
     */
    public void resetHeading(){
        // TODO: implement something that allows the commented code to work
        swerve.resetHeading();
    }

    /**
     * Get the current robot yaw as a Rotation2d
     */
    public Rotation2d getYaw() {
        // TODO: implement something that allows the commented code to work
        return swerve.getYaw();
    }

    /**
     * Get the current position of the swerve as judged by odometry.
     */
    public Pose2d getCurrentPosition() {
        // TODO: implement something that allows the commented code to work
        return swerve.getCurrentOdometryPosition();
    }

   

    /**
     * Reset the robot's known position.
     *
     * @param pose the pose to set the robot's known position to.
     */
    public void resetPose(Pose2d pose) {
        // TODO: implement something that allows the commented code to work
        swerve.setKnownOdometryPose(pose);
        Logger.recordOutput(
            SWERVE.LOG_PATH+"Console", (
                "Current pose reset to X: "+
                pose.getX()+
                "; Y: "+
                pose.getY()+
                "; Rotation: "+
                pose.getRotation().getDegrees()+
                "°."
            )
        );
    }

    public void resetPose(Pose2d pose, boolean flip) {
        // TODO: implement something that allows the commented code to work
        if(flip){
            Pose2d flipped = new Pose2d(
                new Translation2d(
                    MEASUREMENTS.FIELD_LENGTH_METERS-pose.getX(),
                    pose.getY()
                ),
                Rotation2d.fromDegrees(180).minus(pose.getRotation())
            );
            swerve.setKnownOdometryPose(flipped);
            return;
        }
        swerve.setKnownOdometryPose(pose);
    }

    /**
     * Use PID to snap the robot to a rotational setpoint
     *
     * @param setpoint the setpoint to snap to
     * @return the rotational velocity setpoint as a Rotation2d
     */
    public double snapToAngle(Rotation2d setpoint) {
        double currYaw = Math.toRadians(getYaw().getDegrees()%360);
        double errorAngle = setpoint.getRadians() - currYaw;

        if(errorAngle > Math.PI){
            errorAngle -= 2*Math.PI;
        }
        else if(errorAngle <= -Math.PI){
            errorAngle += 2*Math.PI;
        }

        double out = snapToController.calculate(0, errorAngle);

        return out;
    }

    /**
     * Process joystick inputs for human control
     *
     * @param rawX the raw X input from a joystick. Should be -1 to 1
     * @param rawY the raw Y input from a joystick. Should be -1 to 1
     * @param rawRot the raw rotation input from a joystick. Should be -1 to 1
     * @param fieldRelativeAllowed if this is true, switch between field- and
     * robot-relative based on {@link Swerve#robotRelative}. Otherwise, force
     * robot-relative.
     *
     * @return a ChassisSpeeds ready to be sent to the swerve.
     */
    public ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
        double driveTranslateY = (
            rawY >= 0
            ? (Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
        );

        double driveTranslateX = (
            rawX >= 0
            ? (Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
        );

        double driveRotate = (
            rawRot >= 0
            ? (Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
        );

        ChassisSpeeds currentSpeeds;

        if (isSlowMode) {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_SLOW * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_FAST * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }

        return currentSpeeds;
    }
}
