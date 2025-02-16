package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Wrist extends SubsystemBase{
    private KrakenX60Motor wristMotor;
    private double targetWristDegrees;

    public Wrist(){
    
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(WRIST.WRIST_P, WRIST.WRIST_I, WRIST.WRIST_D);
    
      
    
        wristMotor = new KrakenX60Motor(CAN.WRIST_CAN_ID, true);
        targetWristDegrees = 0.0;
    
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setPositionSoftLimit(wristDegreesToMotorRotations(WRIST.WRIST_ANGLE_DEGREES_MIN), wristDegreesToMotorRotations(WRIST.WRIST_ANGLE_DEGREES_MAX));
        wristMotor.setCurrentLimit(WRIST.WRIST_CURRENT_LIMIT);
    
        wristMotor.withGains(positionPid);

        wristMotor.configureMotionMagic(WRIST.WRIST_MAX_ACCELERATION, WRIST.WRIST_MAX_VELOCITY);
        
    }

    public void setWristDegrees(double degrees){
        targetWristDegrees = degrees;
    }

    public double getWristDegrees(){
        return motorRotationsToWristDegrees(wristMotor.getRotations());
    }

    public void setPercentOutput(double percent){
        wristMotor.setPercentOutput(percent);
    }

    public boolean atPosition(){
        return Utils.isWithin(getWristDegrees(), targetWristDegrees, WRIST.WRIST_POSITION_TOLERANCE);
    }

    public double motorRotationsToWristDegrees(double rotations){
        return (rotations*WRIST.WRIST_GEAR_RATIO*360);
    }

    public double wristDegreesToMotorRotations(double degrees){
        return ((degrees/360.0)/WRIST.WRIST_GEAR_RATIO);
    }

    public Command setWristPercentOutputCommand(double percent){
        return this.run(() -> setPercentOutput(percent));
    }

    public Command stopWristCommand(){
        return setWristPercentOutputCommand(0);
    }

    public Command setWristPositionCommand(DoubleSupplier degrees){
        return this.run(()-> setWristDegrees(degrees.getAsDouble())).until(() -> atPosition());
    }

    @Override
    public void periodic(){
        wristMotor.setPosition(wristDegreesToMotorRotations(targetWristDegrees));
        SmartDashboard.putNumber("Wrist|CurrentDegrees", getWristDegrees());
        SmartDashboard.putNumber("Wrist|TargetDegrees", targetWristDegrees);
        SmartDashboard.putBoolean("Wrist|AtPosition", atPosition());
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|CurrentDegrees ", getWristDegrees());
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|TargetDegrees ", targetWristDegrees);
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|AtPosition", atPosition());
    }
}