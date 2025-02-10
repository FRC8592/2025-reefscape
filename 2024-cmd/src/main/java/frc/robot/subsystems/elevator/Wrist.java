package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

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
    
      
    
        wristMotor = new KrakenX60Motor(CAN.WRIST_CAN_ID, true);
        targetDegree = 0.0;
    
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setPositionSoftLimit(wristDegreesToMotorRotations(-45), wristDegreesToMotorRotations(135));
        wristMotor.setCurrentLimit(80);
    
        wristMotor.withGains(positionPid);

        wristMotor.configureMotionMagic(200, 80);
        
    }

    public void setWristDegrees(double degrees){
        targetDegree = degrees;
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

    public Command setWristCommand(DoubleSupplier degrees){
        return this.run(()-> setWristDegrees(degrees.getAsDouble()));
    }

    @Override
    public void periodic(){
        wristMotor.setPosition(wristDegreesToMotorRotations(targetDegree));
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"current wrist degrees ", getWristDegrees());
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"target degree ", targetDegree);
        Logger.recordOutput(ELEVATOR.WRIST_LOG_PATH+"at position", atPosition());
    }
}