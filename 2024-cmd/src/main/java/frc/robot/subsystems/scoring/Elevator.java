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

public class Elevator extends SubsystemBase{
    private KrakenX60Motor leftExtensionMotor;
    private KrakenX60Motor rightExtensionMotor;
    private double targetExtension;

    public Elevator(){
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(ELEVATOR.ELEVATOR_POSITION_P, ELEVATOR.ELEVATOR_POSITION_I, ELEVATOR.ELEVATOR_POSITION_D);
        
        PIDProfile velocityPid = new PIDProfile();

        velocityPid.setPID(ELEVATOR.ELEVATOR_VELOCITY_P, ELEVATOR.ELEVATOR_VELOCITY_I, ELEVATOR.ELEVATOR_VELOCITY_D);
        velocityPid.setS(ELEVATOR.ELEVATOR_VELOCITY_S);

        leftExtensionMotor = new KrakenX60Motor(CAN.BACK_EXTENSION_MOTOR_CAN_ID, true);
        rightExtensionMotor = new KrakenX60Motor(CAN.FORWARD_EXTENSION_MOTOR_CAN_ID);

        leftExtensionMotor.setIdleMode(IdleMode.kCoast);
        rightExtensionMotor.setIdleMode(IdleMode.kCoast);


        //configure right motor to be inverted and to follow L motor
        rightExtensionMotor.setFollowerTo(leftExtensionMotor, true);

        leftExtensionMotor.setPositionSoftLimit(inchesToRotations(ELEVATOR.EXTENSION_INCHES_MIN), inchesToRotations(ELEVATOR.EXTENSION_INCHES_MAX));
        leftExtensionMotor.setCurrentLimit(ELEVATOR.ELEVATOR_CURRENT_LIMIT);
        rightExtensionMotor.setCurrentLimit(ELEVATOR.ELEVATOR_CURRENT_LIMIT);

        leftExtensionMotor.withGains(positionPid, 0);
        leftExtensionMotor.withGains(velocityPid, 1);

        // leftExtensionMotor.configureMotionMagic(600, 100);
        leftExtensionMotor.configureMotionMagic(ELEVATOR.ELEVATOR_MAX_ACCELERATION, ELEVATOR.ELEVATOR_MAX_VELOCITY);


        //SmartDashboard.putData("Elevator PID", positionPid);

        setPercentOutput(0);
        targetExtension = 0;
    }


    /**
    * Gets the current inches of the extention by converting left motor rotations to inches. 
    * @return Returns the current height of the elevator in inches.
    */
    public double getInches(){
        return rotationsToInches(leftExtensionMotor.getRotations());
    }

    /**
     * Accepts the target inches and sets the target extention to the target inches.
     * @param targetInches The desired position of the elevator in inches.
     */
    public void setInches(double targetInches){
        targetExtension = targetInches;
    }

    /**
     * Accepts a percentage and sets the motor power to that given percentage.
     * @param percent The percentage of power that we want the elevator to be moving at.
     */
    public void setPercentOutput(double percent){
        leftExtensionMotor.setPercentOutput(percent);
    }

    /**
     * Converts an amount of motor rotations to extention inches.
     * @param rotations The amount of motor rotations we want to convert to inches.
     * @return Returns an amount of motor rotations as inches.
     */
    private double rotationsToInches(double rotations){
        return (rotations*(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))*ELEVATOR.EXTENSION_GEAR_RATIO;
    }

    /**
     * Converts an amount of extention inches to motor rotations.
     * @param inches The amount of inches we want to convert to rotations of the motor.
     * @return Returns an amount of inches as motor rotations.
     */
    private double inchesToRotations(double inches){
        return ((inches/(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))/ELEVATOR.EXTENSION_GEAR_RATIO);
    }
    
    /**
     * Detects whether the elevator is at its desired position and outputs it as a boolean.
     * @return Returns whether the elevator is at its target position as a boolean.
     */
    public boolean atPosition(){
        return Utils.isWithin(getInches(), targetExtension, ELEVATOR.EXTENSION_POSITION_TOLERANCE);
    }

    /**
     * Sets the target extention of the elevator to the target inches.
     * @param targetExtension The desired position of the elevator in inches.
     * @return Returns a command to set the target extention to the target inches.
     */
    public Command setInchesCommand(DoubleSupplier targetExtension){
        return this.run(()-> setInches(targetExtension.getAsDouble()));
    }

    /**
     * Accepts a percentage and then sets the motor power to that percentage.
     * @param power The percentage we want to set the motors power to.
     * @return Returns a command to set the speed at which the motor moves.
     */
    public Command setPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }

    /**
     * Stops the extention by setting the where it's moving to where it currently is.
     * @return Returns a command to stop the elevator.
     */
    public Command stopCommand() {
        return this.runOnce(() -> {
            setInches(getInches());
        });
    }

    @Override
    public void periodic(){
        //Moves the elevator to the target extention.
        leftExtensionMotor.setPosition(inchesToRotations(targetExtension));
        //Logs the current extention, target extention, and whether the elevator is at position.
        // SmartDashboard.putNumber("Extension|CurrentInches", getInches());
        // SmartDashboard.putNumber("Extension|TargetInches ", targetExtension);
        // SmartDashboard.putNumber("Extension|TargetExtentionInRot", inchesToRotations(targetExtension));
        // SmartDashboard.putBoolean("Extension|AtPosition", atPosition());
        // SmartDashboard.putNumber("Extension|AppliedVoltage", leftExtensionMotor.getVoltage());
        // SmartDashboard.putNumber("Extension|CurrentVelocity", leftExtensionMotor.getVelocityRPM());

        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|CurrentInches", getInches());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|TargetInches ", targetExtension);
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|TargetExtentionInRot", inchesToRotations(targetExtension));
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|AtPosition", atPosition());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|AppliedVoltage", leftExtensionMotor.getVoltage());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|CurrentVelocity", leftExtensionMotor.getVelocityRPM());
    }

}
