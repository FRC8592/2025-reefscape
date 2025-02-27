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
    
      
    
        wristMotor = new KrakenX60Motor(CAN.WRIST_CAN_ID, SHARED.IS_RIPTIDE);
        targetWristDegrees = 0.0;
        
        wristMotor.setIdleMode(IdleMode.kCoast);
        wristMotor.setPositionSoftLimit(degreesToMotorRotations(WRIST.WRIST_ANGLE_DEGREES_MIN), degreesToMotorRotations(WRIST.WRIST_ANGLE_DEGREES_MAX));
        wristMotor.setCurrentLimit(WRIST.WRIST_CURRENT_LIMIT);
        
    
        wristMotor.withGains(positionPid);

        wristMotor.configureMotionMagic(WRIST.WRIST_MAX_ACCELERATION, WRIST.WRIST_MAX_VELOCITY);
        
    }

    /**
     * Accepts desired arm degrees and sets that target degrees for the wrist to the desired degrees.
     * @param degrees The degrees desired by the user.
     */
    public void setDegrees(double degrees){
        targetWristDegrees = degrees;
    }

    /**
     * Gets the current wrist degrees by taking the motor rotations and coverting them to degrees.
     * @return Returns the current wrist degrees.
     */
    public double getDegrees(){
        return motorRotationsToDegrees(wristMotor.getRotations());
    }

    /**
     * Accepts a percentage and sets the motor power to that given percentage.
     * @param percent The percentage of power that we want the wrist to be moving at.
     */
    public void setPercentOutput(double percent){
        wristMotor.setPercentOutput(percent);
    }

    /**
     * Outputs whether the wrist is at it's desired position or not as a boolean.
     * @return Returns if the wriat is at position as a boolean.
     */
    public boolean atPosition(){
        return Utils.isWithin(getDegrees(), targetWristDegrees, WRIST.WRIST_POSITION_TOLERANCE);
    }

    /**
     * Converts an amount of motor rotations to wrist degrees.
     * @param rotations The amount of wrist motor rotations we want to convert to degrees.
     * @return Returns the amount of rotations we wanted to convert as degrees.
     */
    public double motorRotationsToDegrees(double rotations){
        return (rotations*WRIST.WRIST_GEAR_RATIO*360);
    }

    /**
     * Converts an amount of wrist degrees to motor rotations.
     * @param degrees The amount of wrist degrees we want to convent to motor rotations.
     * @return Returns the amount of degrees we wanted to convert as rotations.
     */
    public double degreesToMotorRotations(double degrees){
        return ((degrees/360.0)/WRIST.WRIST_GEAR_RATIO);
    }

    /**
     * Accepts a percentage and sets the motor power to that percentage.
     * @param power The percentage of we want the motor to be moving at.
     * @return Returns a command to set the motors speed as a percentage.
     */
    public Command setPercentOutputCommand(double percent){
        return this.run(() -> setPercentOutput(percent));
    }

    /**
     * Stops the wrist by setting the target degrees to the current degrees.
     * @return Returns a command to stop the wrist motor.
     */
    public Command stopCommand(){
        return this.runOnce(() -> {
            setDegrees(getDegrees());
        });
    }

    /**
     * Accepts an amount of degrees and sets the target wrist degrees to that amount of degrees.
     * @param degrees The amount of degrees we want the wrist to go to.
     * @return Returns a command to set the target degrees to the desired degrees.
     */
    public Command setDegreesCommand(DoubleSupplier degrees){
        return this.run(()-> setDegrees(degrees.getAsDouble()));
    }

    @Override
    public void periodic(){
        //Moves the wrist to the target degrees.
        wristMotor.setPosition(degreesToMotorRotations(targetWristDegrees));
        //Logs the wrist target degrees, current degrees, and whether the wrist is at its desired position.
        // SmartDashboard.putNumber("Wrist|CurrentDegrees", getDegrees());
        // SmartDashboard.putNumber("Wrist|TargetDegrees", targetWristDegrees);
        // SmartDashboard.putBoolean("Wrist|AtPosition", atPosition());
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|CurrentDegrees ", getDegrees());
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|TargetDegrees ", targetWristDegrees);
        Logger.recordOutput(WRIST.WRIST_LOG_PATH+"Wrist|AtPosition", atPosition());
    }
}