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

public class ClockArm extends SubsystemBase{
    private KrakenX60Motor clockArmMotor;
    private double targetDegree;

    public ClockArm(){
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(3, 0, 0);

  

        clockArmMotor = new KrakenX60Motor(CAN.CLOCK_ARM_CAN_ID);
        targetDegree = 0.0;

        clockArmMotor.setIdleMode(IdleMode.kCoast);
        clockArmMotor.setPositionSoftLimit(armDegreesToMotorRotations(0), armDegreesToMotorRotations(180));
        clockArmMotor.setCurrentLimit(80);

        clockArmMotor.withGains(positionPid);

        clockArmMotor.configureMotionMagic(200, 80);
    }

    public void setArmPositionDegrees(double degrees){
        targetDegree = degrees;
    }

    public double getArmPositionDegrees(){
        return motorRotationsToArmDegrees(clockArmMotor.getRotations());
    }

    public double getTargetArmPositionDegrees(){
        return targetDegree;
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

    public Command setArmPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }
    
    public Command stopArmCommand() {
        return this.runOnce(() -> setPercentOutput(0));
    }

    public Command setArmPositionCommand(DoubleSupplier degrees){
        return this.run(()-> setArmPositionDegrees(degrees.getAsDouble()));
    }

    @Override
    public void periodic(){
        clockArmMotor.setPosition(armDegreesToMotorRotations(targetDegree));
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"current arm degrees ", getArmPositionDegrees());
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"target degree ", targetDegree);
        Logger.recordOutput(ELEVATOR.CLOCK_ARM_LOG_PATH+"at position", atPosition());
    }
}