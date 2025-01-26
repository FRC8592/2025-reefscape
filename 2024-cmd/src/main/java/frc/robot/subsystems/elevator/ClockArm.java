package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class ClockArm extends SubsystemBase{
    private KrakenX60Motor clockArmMotor;
    private double targetDegree;

    public ClockArm(){
        clockArmMotor = new KrakenX60Motor(CAN.CLOCK_ARM_CAN_ID);
        targetDegree = 0.0;
    }

    public void setArmPositionDegrees(double degrees){
        targetDegree = degrees;
        clockArmMotor.setPosition(armDegreesToMotorRotations(degrees));
    }

    public double getArmPositionDegrees(){
        return motorRotationsToArmDegrees(clockArmMotor.getRotations());
    }

    public void setPercentOutput(double percent){
        clockArmMotor.setPercentOutput(percent);
    }

    public boolean atPosition(){
        return Utils.isWithin(getArmPositionDegrees(), targetDegree, ELEVATOR.CLOCK_ARM_POSITION_TOLERANCE);
    }

    public double motorRotationsToArmDegrees(double rotations){
        return (rotations*ELEVATOR.CLOCK_ARM_GEAR_RATIO*360);
    }

    public double armDegreesToMotorRotations(double degrees){
        return ((degrees/360.0)/ELEVATOR.CLOCK_ARM_GEAR_RATIO);
    }

    public Command setArmPositionCommand(double degrees){
        return this.run(()-> setArmPositionDegrees(degrees));
    }

    @Override
    public void periodic(){
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"current arm degrees ", getArmPositionDegrees());
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"target degree ", targetDegree);
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"at position", atPosition());
    }
}
