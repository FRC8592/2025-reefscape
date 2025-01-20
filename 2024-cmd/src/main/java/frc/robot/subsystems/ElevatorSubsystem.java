package frc.robot.subsystems;

import frc.robot.Ports;
import lib.team8592.MatchMode;
import lib.team8592.hardware.motor.talonfx.KrakenX60Motor;

import static frc.robot.Constants.ELEVATOR.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends NewtonSubsystem {
    private KrakenX60Motor leftElevatorMotor;
    private KrakenX60Motor rightElevatorMotor;
    private double targetElevatorHeightInches = 0d;

    /***
     * Instantiate elevator and pivot motors
     */
    protected ElevatorSubsystem(boolean logToShuffleboard){
        super(logToShuffleboard);

        leftElevatorMotor = new KrakenX60Motor(Ports.LEFT_ELEVATOR_MOTOR_CAN_ID);
        rightElevatorMotor = new KrakenX60Motor(Ports.RIGHT_ELEVATOR_MOTOR_CAN_ID);
    }

    /***
     * Sets target elevator position 
     * @param position in inches
     */
    public void setElevatorHeight(double desiredInches){
        this.targetElevatorHeightInches = desiredInches;

        double rotations = fromElevatorInchesToMotorRotations(desiredInches);
        this.leftElevatorMotor.setPositionSmartMotion(rotations);
        this.rightElevatorMotor.setPositionSmartMotion(rotations);
    }

    private double getMotorRotations() {
        return this.leftElevatorMotor.getRotations();
    }

    /***
     * Gets the elevator position
     * @return elevator position in inches
     */
    public double getElevatorHeightInches(){
        return fromMotorRotationsToElevatorInches(getMotorRotations());
    }

    private double fromElevatorInchesToMotorRotations(double inches){
        return (inches / (DRIVEN_SPROCKET_DIAMETER_INCHES * Math.PI)) * OVERALL_GEAR_RATIO;
    }

    private double fromMotorRotationsToElevatorInches(double rotations){
        return (rotations / OVERALL_GEAR_RATIO) * (DRIVEN_SPROCKET_DIAMETER_INCHES * Math.PI);
    }

    public boolean atTargetHeight(){
        return Math.abs(targetElevatorHeightInches - getElevatorHeightInches()) < ELEVATOR_TOLERANCE_ROTATIONS;
    }

    public Command setElevatorHeightCommand(double degrees) {
        return this.run(() -> {
            this.setElevatorHeight(degrees);
        }).until(() -> atTargetHeight());
    }

    @Override
    public void onInit(MatchMode mode) {
        this.stop();
    }

    @Override
    public void periodicTelemetry() {
        this.logger.log("Desired Elevator Height Inches", targetElevatorHeightInches);
        this.logger.log("Desired Motor Rotations", fromElevatorInchesToMotorRotations(targetElevatorHeightInches));
        this.logger.log("Current Elevator Height Inches", getElevatorHeightInches());
        this.logger.log("Current Elevator Rotations", getMotorRotations());
    }

    @Override
    public void stop() {
        leftElevatorMotor.setPercentOutput(0d);
        rightElevatorMotor.setPercentOutput(0d);
    }
}