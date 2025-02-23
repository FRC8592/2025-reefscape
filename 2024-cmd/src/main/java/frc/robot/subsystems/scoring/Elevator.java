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



    public double getInches(){
        return rotationsToInches(leftExtensionMotor.getRotations());
    }

    public void setInches(double targetInches){
        targetExtension = targetInches;
    }

    public void setPercentOutput(double percent){
        leftExtensionMotor.setPercentOutput(percent);
    }

    private double rotationsToInches(double rotations){
        return (rotations*(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))*ELEVATOR.EXTENSION_GEAR_RATIO;
    }

    private double inchesToRotations(double inches){
        return ((inches/(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))/ELEVATOR.EXTENSION_GEAR_RATIO);
    }
    
    public boolean atPosition(){
        return Utils.isWithin(getInches(), targetExtension, ELEVATOR.EXTENSION_POSITION_TOLERANCE);
    }

    public Command setInchesCommand(DoubleSupplier targetExtension){
        return this.run(()-> setInches(targetExtension.getAsDouble()));
    }

    public Command setPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }

    public Command stopCommand() {
        return this.runOnce(() -> {
            setInches(getInches());
        });
    }

    @Override
    public void periodic(){
        leftExtensionMotor.setPosition(inchesToRotations(targetExtension));
        SmartDashboard.putNumber("Extension|CurrentInches", getInches());
        SmartDashboard.putNumber("Extension|TargetInches ", targetExtension);
        SmartDashboard.putNumber("Extension|TargetExtentionInRot", inchesToRotations(targetExtension));
        SmartDashboard.putBoolean("Extension|AtPosition", atPosition());
        SmartDashboard.putNumber("Extension|AppliedVoltage", leftExtensionMotor.getVoltage());
        SmartDashboard.putNumber("Extension|CurrentVelocity", leftExtensionMotor.getVelocityRPM());

        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|CurrentInches", getInches());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|TargetInches ", targetExtension);
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|TargetExtentionInRot", inchesToRotations(targetExtension));
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|AtPosition", atPosition());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|AppliedVoltage", leftExtensionMotor.getVoltage());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"Extension|CurrentVelocity", leftExtensionMotor.getVelocityRPM());
    }

}
