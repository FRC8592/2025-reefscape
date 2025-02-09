package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Arm {
    private KrakenX60Motor armControlMotor;
    private double targetExtension;

    public Arm(){
        PIDProfile positionPid = new PIDProfile();
        positionPid.setPID(ELEVATOR.ARM_P, ELEVATOR.ARM_I, ELEVATOR.ARM_D);
        
        // PIDProfile velocityPid = new PIDProfile();

        //TODO: SET THESE TO ACTUAL CONSTANTS
        // velocityPid.setPID(0.001, 0, 0);
        // velocityPid.setS(0.1);

        armControlMotor = new KrakenX60Motor(CAN.CLOCK_ARM_CAN_ID, false);

        armControlMotor.setIdleMode(IdleMode.kBrake);


  

        // armControlMotor.setPositionSoftLimit(inchesToRotations(ELEVATOR.ARM_ANGLE_DEGREES_MIN), inchesToRotations(ELEVATOR.ARM_ANGLE_DEGREES_MAX));
        // armControlMotor.setCurrentLimit(80);
        // armControlMotor.setCurrentLimit(80);

        // armControlMotor.withGains(positionPid, 0);
        // armControlMotor.withGains(velocityPid, 1);

        armControlMotor.configureMotionMagic(600, 100);

        SmartDashboard.putData("Elevator PID", positionPid);

        setPercentOutput(0);
        targetExtension = 0;
    }

    public void periodic(){
        // Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"current extension inches ", getExtensionPositionInches());
        // Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"current extension inches ", getExtensionPositionInches());
        // Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"target inches ", targetExtension);
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"arm position ", armControlMotor.getRotations());
        
        // Logger.recordOutput("Target extension in rotations", inchesToRotations(targetExtension));
        // Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"at position", atPosition());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"arm applied voltage", armControlMotor.getVoltage());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"arm current velocity", armControlMotor.getVelocityRPM());

        // setExtensionPositionInches(targetExtension);
    }
    
    public void setPercentOutput(double percent){
        armControlMotor.setPercentOutput(percent);
    }

    // public double getExtensionPositionInches(){
    //     return rotationsToInches(armControlMotor.getRotations());
    // }

    // public void setExtensionPositionInches(double targetInches){
    //     targetExtension = targetInches;
    //     armControlMotor.setPosition(inchesToRotations(targetInches));
    // }



    // private double rotationsToInches(double rotations){
    //     return (rotations*(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))*ELEVATOR.EXTENSION_GEAR_RATIO;
    // }

    // private double inchesToRotations(double inches){
    //     return ((inches/(ELEVATOR.EXTENSION_DRUM_DIAMTER_INCHES*Math.PI))/ELEVATOR.EXTENSION_GEAR_RATIO);
    // }
    
    // public boolean atPosition(){
    //     return Utils.isWithin(getExtensionPositionInches(), targetExtension, ELEVATOR.EXTENSION_POSITION_TOLERANCE);
    // }



}
