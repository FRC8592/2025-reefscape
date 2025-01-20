// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.largecommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.Constants.*;

public class ScoreCoralCommand extends LargeCommand {

    public static ChassisSpeeds speedZero = new ChassisSpeeds();
    
    private PIDController xController;
    private PIDController yController;
    private PIDController rotController;

    public ScoreCoralCommand() {
        super(swerve);
    }

    public void initialize(){
        xController = new PIDController(CORAL_ALIGN.X_KP, CORAL_ALIGN.X_KI, CORAL_ALIGN.X_KD);
        yController = new PIDController(CORAL_ALIGN.Y_KP, CORAL_ALIGN.Y_KI, CORAL_ALIGN.Y_KD);
        rotController = new PIDController(CORAL_ALIGN.ROT_KP, CORAL_ALIGN.ROT_KI, CORAL_ALIGN.ROT_KD);
    }

    public void execute() {
        // Setting the x speed, y speed,rotating speed
        double xSpeed = 0d, ySpeed = 0d, rotSpeed = 0d;


        if(vision.getTargetVisible() == true){
            ySpeed = xController.calculate(vision.getTargetX(), CORAL_ALIGN.X_OFFSET);
            ySpeed = Math.min(CORAL_ALIGN.SPEED_MAX, ySpeed);
            ySpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, ySpeed);

            ySpeed = -ySpeed * CORAL_ALIGN.SPEED_SCALE;

            xSpeed = yController.calculate(vision.getTargetY(), CORAL_ALIGN.Y_OFFSET);
            xSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, xSpeed);
            xSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, xSpeed);
    
            xSpeed = -xSpeed * CORAL_ALIGN.SPEED_SCALE;
    
            rotSpeed = rotController.calculate(vision.getTargetYaw(), CORAL_ALIGN.ROT_OFFSET);
            rotSpeed = Math.min(CORAL_ALIGN.SPEED_MAX, rotSpeed);
            rotSpeed = Math.max(-CORAL_ALIGN.SPEED_MAX, rotSpeed);
    
            rotSpeed = rotSpeed * CORAL_ALIGN.SPEED_SCALE;

            ChassisSpeeds speed = swerve.processJoystickInputs(xSpeed, ySpeed, rotSpeed);
            SmartDashboard.putString("ChassisSpeedJoystick", speed.toString());
            swerve.drive(speed, DriveModes.ROBOT_RELATIVE);

        } else {
            swerve.drive(new ChassisSpeeds());
        }

        SmartDashboard.putNumber("Provided XSpeed", xSpeed);
        SmartDashboard.putNumber("Provided YSpeed", ySpeed);
    }

    public void end(boolean interrupted){
        swerve.drive(new ChassisSpeeds());
    }
}
