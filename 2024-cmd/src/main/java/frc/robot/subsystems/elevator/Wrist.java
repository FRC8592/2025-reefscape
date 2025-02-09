package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Wrist extends SubsystemBase{
    private KrakenX60Motor wristMotor;
    private double targetDegree;

    public Wrist(){
    
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(3, 0, 0);
    
      
    
        wristMotor = new KrakenX60Motor(CAN.WRIST_CAN_ID);
        targetDegree = 0.0;
    
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setPositionSoftLimit(wristDegreesToMotorRotations(0), wristDegreesToMotorRotations(180));
        wristMotor.setCurrentLimit(80);
    
        wristMotor.withGains(positionPid);

        wristMotor.configureMotionMagic(300, 100);
        
    }

    public void setWristDegrees(double degrees){
        targetDegree = degrees;
        wristMotor.setPosition(wristDegreesToMotorRotations(degrees));
    }

    public double getWristDegrees(){
        return motorRotationsToWristDegrees(wristMotor.getRotations());
    }

    public void setPercentOutput(double percent){
        wristMotor.setPercentOutput(percent);
    }

    public boolean atPosition(){
        return Utils.isWithin(getWristDegrees(), targetDegree, ELEVATOR.WRIST_POSITION_TOLERANCE);
    }

    public double motorRotationsToWristDegrees(double rotations){
        return (rotations*ELEVATOR.WRIST_GEAR_RATIO*360);
    }

    public double wristDegreesToMotorRotations(double degrees){
        return ((degrees/360.0)/ELEVATOR.WRIST_GEAR_RATIO);
    }

    public Command setWristPercentOutput(double percent){
        return this.run(() -> setPercentOutput(percent));
    }

    public Command stopWrist(){
        return this.run(() -> setPercentOutput(0));
    }

    public Command setWristCommand(double degrees){
        return this.run(()-> setWristDegrees(degrees)).until(() -> atPosition());
    }

    @Override
    public void periodic(){
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"current wrist degrees ", getWristDegrees());
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"target degree ", targetDegree);
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"at position", atPosition());
    }
}