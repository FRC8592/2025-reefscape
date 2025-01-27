package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Elevator extends SubsystemBase{
    private KrakenX60Motor leftExtensionMotor;
    private KrakenX60Motor rightExtensionMotor;
    private double targetExtension;

    public Elevator(){
        leftExtensionMotor = new KrakenX60Motor(CAN.LEFT_EXTENSION_MOTOR_CAN_ID);
        rightExtensionMotor = new KrakenX60Motor(CAN.RIGHT_EXTENSION_MOTOR_CAN_ID);
        rightExtensionMotor.setInverted(true);
        targetExtension = 0;
    }

    public double getExtensionPositionInches(){
        return rotationsToInches(leftExtensionMotor.getRotations());
    }

    public void setExtensionPositionInches(double targetInches){
        targetExtension = targetInches;
        leftExtensionMotor.setPosition(inchesToRotations(targetInches));
        rightExtensionMotor.setPosition(inchesToRotations(targetInches));
    }

    public void setPercentOutput(double percent){
        leftExtensionMotor.setPercentOutput(percent);
        rightExtensionMotor.setPercentOutput(percent);
    }

    private double rotationsToInches(double rotations){
        return (rotations*(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))*ELEVATOR.EXTENSION_GEAR_RATIO;
    }

    private double inchesToRotations(double inches){
        return ((inches/(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))/ELEVATOR.EXTENSION_GEAR_RATIO);
    }
    
    public boolean atPosition(){
        return Utils.isWithin(getExtensionPositionInches(), targetExtension, ELEVATOR.EXTENSION_POSITION_TOLERANCE);
    }

    public Command setExtensionCommand(double targetExtension){
        return this.run(()-> setExtensionPositionInches(targetExtension));
    }

    @Override
    public void periodic(){
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"current extension inches ", getExtensionPositionInches());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"target inches ", targetExtension);
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"at position", atPosition());
    }
}
