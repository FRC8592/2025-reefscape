// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.Constants.*;
import frc.robot.commands.largecommands.FollowPathCommand;

public class ScoreCoral extends SubsystemBase {

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    private Swerve swerve;
    
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

    public ScoreCoral(Swerve swerve) {
        this.swerve = swerve;
    }

    public void periodic() {
        
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

    public void driveToReef() {
        // Setting the x speed, y speed,rotating speed
        double xSpeed = 0d, ySpeed = 0d, rotSpeed = 0d;

        Pose2d currentPosition = swerve.getCurrentPosition();
        double xDistance = target.getX() - currentPosition.getX();
        double yDistance = target.getY() - currentPosition.getY();
        //TODO: ensure if correct
        double rotDistance = target.getRotation().getDegrees() - currentPosition.getRotation().getDegrees();
        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/xDistance", xDistance);
        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/yDistance", yDistance);
        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/rotDistance", rotDistance);

        Logger.recordOutput(SHARED.LOG_FOLDER+"/Scorecoral/Target", target);
        
        xSpeed = xController.calculate(xDistance);
        xSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, xSpeed);
        xSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, xSpeed);

        xSpeed = -xSpeed * CORAL_ALIGN.SPEED_SCALE;

        ySpeed = yController.calculate(yDistance);
        ySpeed = Math.min(CORAL_ALIGN.SPEED_MAX, ySpeed);
        ySpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, ySpeed);

        ySpeed = -ySpeed * CORAL_ALIGN.SPEED_SCALE;

        rotSpeed = rotController.calculate(rotDistance);
        rotSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, rotSpeed);
        rotSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, rotSpeed);

        rotSpeed = -rotSpeed * CORAL_ALIGN.SPEED_SCALE;

        //Field-relative x, y axis and joystick x, y axis are flipped (i.g. field x is joystick y)
        ChassisSpeeds speed = swerve.processJoystickInputs(ySpeed, xSpeed, rotSpeed);
        SmartDashboard.putString("ChassisSpeedJoystick", speed.toString());
        Logger.recordOutput("speed", speed);
        swerve.drive(speed, DriveModes.FIELD_RELATIVE);

        SmartDashboard.putNumber("Provided XSpeed", xSpeed);
        SmartDashboard.putNumber("Provided YSpeed", ySpeed);
        SmartDashboard.putNumber("Provided RotSpeed", rotSpeed);

        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/Final XSpeed", speed.vxMetersPerSecond);
        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/Final YSpeed", speed.vyMetersPerSecond);
        SmartDashboard.putNumber(SHARED.LOG_FOLDER+"/Scorecoral/Final RotSpeed", speed.omegaRadiansPerSecond);

    }

    public void setPosition(LeftOrRight leftOrRight, ScoreLevels scoreLevel){
       direction = leftOrRight;
       level = scoreLevel;
       SmartDashboard.putString("direction", direction.name());
       SmartDashboard.putString("Level", level.name());
    }    
}
