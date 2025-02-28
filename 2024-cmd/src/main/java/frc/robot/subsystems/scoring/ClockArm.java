package frc.robot.subsystems.scoring;

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

    /**
     * Accepts desired arm degrees and sets that target degrees for the arm to the desired degrees.
     * @param degrees The degrees desired by the user.
     */
    public void setDegrees(double degrees){
        targetArmDegrees = degrees;
    }

    /**
     * Gets the current arm degrees by taking the motor rotations and coverting them to degrees.
     * @return Returns the current arm degrees.
     */
    public double getDegrees(){
        return motorRotationsToDegrees(clockArmMotor.getRotations());
    }

    /**
     * Gets the position that the arm is currently trying to go to.
     * @return Returns the target arm degrees.
     */
    public double getTargetPositionDegrees(){
        return targetArmDegrees;
    }

    /**
     * Accepts a percentage and sets the motor power to that given percentage.
     * @param percent The percentage of power that we want the arm to be moving at.
     */
    public void setPercentOutput(double percent){
        clockArmMotor.setPercentOutput(percent);
    }

    /**
     * Outputs whether the arm is at it's desired position or not as a boolean.
     * @return Returns if the arm is at position as a boolean.
     */
    public boolean atPosition(){
        return Utils.isWithin(getDegrees(), targetArmDegrees, ARM.CLOCK_ARM_POSITION_TOLERANCE);
    }

    /**
     * Outputs whether the arm is at it's desired position within the passed-in tolerance.
     * @param tolerence the number of degrees the arm must be within to count as "at position."
     * @return Returns if the arm is at position as a boolean.
     */
    public boolean atPosition(double tolerance){
        return Utils.isWithin(getDegrees(), targetArmDegrees, tolerance);
    }

    /**
     * Converts an amount of motor rotations to arm degrees.
     * @param rotations The amount of arm motor rotations we want to convert to degrees.
     * @return Returns the amount of rotations we wanted to convert as degrees.
     */
    public double motorRotationsToDegrees(double rotations){
        return (rotations*ARM.CLOCK_ARM_GEAR_RATIO*360);
    }

    /**
     * Converts an amount of arm degrees to motor rotations.
     * @param degrees The amount of arm degrees we want to convent to motor rotations.
     * @return Returns the amount of degrees we wanted to convert as rotations.
     */
    public double degreesToMotorRotations(double degrees){
        return ((degrees/360.0)/ARM.CLOCK_ARM_GEAR_RATIO);
    }

    /**
     * Accepts a percentage and sets the motor power to that percentage.
     * @param power The percentage of we want the motor to be moving at.
     * @return Returns a command to set the motors speed as a percentage.
     */
    public Command setPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }
    
    /**
     * Stops the arm by setting the target degrees to the current degrees.
     * @return Returns a command to stop the arm motor.
     */
    public Command stopCommand() {
        return this.runOnce(() -> {
            setDegrees(getDegrees());
        });
    }

    /**
     * Accepts an amount of degrees and sets the target arm degrees to that amount of degrees.
     * @param degrees The amount of degrees we want the arm to go to.
     * @return Returns a command to set the target degrees to the desired degrees.
     */
    public Command setDegreesCommand(DoubleSupplier degrees){
        return this.run(()-> setDegrees(degrees.getAsDouble()));
    }

    @Override
    public void periodic(){
        //Moves the arm to the target degrees.
        clockArmMotor.setPosition(degreesToMotorRotations(targetArmDegrees));
        //Logs the arms target degrees, current degrees, and whether the arm is at position.
        // SmartDashboard.putNumber("ClockArm|CurrentDegrees", getDegrees());
        // SmartDashboard.putNumber("ClockArm|TargetDegrees", targetArmDegrees);
        // SmartDashboard.putBoolean("ClockArm|AtPosition", atPosition());
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|CurrentDegrees ", getDegrees());
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|TargetDegrees", targetArmDegrees);
        Logger.recordOutput(ARM.CLOCK_ARM_LOG_PATH+"ClockArm|AtPosition", atPosition());
    }
}