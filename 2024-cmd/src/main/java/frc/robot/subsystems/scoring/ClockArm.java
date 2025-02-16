package frc.robot.subsystems.scoring;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class ClockArm extends SubsystemBase{
    private KrakenX60Motor clockArmMotor;
    private double targetArmDegrees;

    public ClockArm(){
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(ARM.ARM_P, ARM.ARM_I, ARM.ARM_D);

  

        clockArmMotor = new KrakenX60Motor(CAN.CLOCK_ARM_CAN_ID);
        targetArmDegrees = 0.0;

        clockArmMotor.setIdleMode(IdleMode.kCoast);
        clockArmMotor.setPositionSoftLimit(degreesToMotorRotations(ARM.ARM_ANGLE_DEGREES_MIN), degreesToMotorRotations(ARM.ARM_ANGLE_DEGREES_MAX));
        clockArmMotor.setCurrentLimit(ARM.ARM_CURRENT_LIMIT);

        clockArmMotor.withGains(positionPid);

        clockArmMotor.configureMotionMagic(ARM.ARM_MAX_ACCELERATION, ARM.ARM_MAX_VELOCITY);
    }

    public void setDegrees(double degrees){
        targetArmDegrees = degrees;
    }

    public double getDegrees(){
        return motorRotationsToDegrees(clockArmMotor.getRotations());
    }

    public double getTargetPositionDegrees(){
        return targetArmDegrees;
    }

    public void setPercentOutput(double percent){
        clockArmMotor.setPercentOutput(percent);
    }

    public boolean atPosition(){
        return Utils.isWithin(getDegrees(), targetArmDegrees, ARM.CLOCK_ARM_POSITION_TOLERANCE);
    }

    public double motorRotationsToDegrees(double rotations){
        return (rotations*ARM.CLOCK_ARM_GEAR_RATIO*360);
    }

    public double degreesToMotorRotations(double degrees){
        return ((degrees/360.0)/ARM.CLOCK_ARM_GEAR_RATIO);
    }

    public Command setPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }
    
    public Command stopCommand() {
        return setPercentOutputCommand(0);
    }

    public Command setDegreesCommand(DoubleSupplier degrees){
        return this.run(()-> setDegrees(degrees.getAsDouble()));
    }

    @Override
    public void periodic(){
        clockArmMotor.setPosition(degreesToMotorRotations(targetArmDegrees));
        SmartDashboard.putNumber("ClockArm|CurrentDegrees", getDegrees());
        SmartDashboard.putNumber("ClockArm|TargetDegrees", targetArmDegrees);
        SmartDashboard.putBoolean("ClockArm|AtPosition", atPosition());
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|CurrentDegrees ", getDegrees());
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|TargetDegrees", targetArmDegrees);
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|AtPosition", atPosition());
    }
}