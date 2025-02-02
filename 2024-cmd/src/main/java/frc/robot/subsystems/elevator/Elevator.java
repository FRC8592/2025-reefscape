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

public class Elevator extends SubsystemBase{
    private KrakenX60Motor leftExtensionMotor;
    private KrakenX60Motor rightExtensionMotor;
    private double targetExtension;

    public enum ElevatorPositions {

        L1(0),
        L2(0),
        L3(0),
        L4(0),
        GROUND_ALGAE(0),
        HP_INTAKE(0),
        STOW(0),
        L2_ALGAE(0),
        L3_ALGAE(0),
        PROCESSOR(0),
        NET(0);

        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private  ElevatorPositions(double elevator) {

            elevatorPos = elevator;


        }

    }

    public Elevator(){
        PIDProfile pid = new PIDProfile();
        pid.setPID(ELEVATOR.ELEVATOR_P, ELEVATOR.ELEVATOR_I, ELEVATOR.ELEVATOR_D);
        
    
        leftExtensionMotor = new KrakenX60Motor(CAN.BACK_EXTENSION_MOTOR_CAN_ID, true);
        rightExtensionMotor = new KrakenX60Motor(CAN.FORWARD_EXTENSION_MOTOR_CAN_ID);

        leftExtensionMotor.setIdleMode(IdleMode.kBrake);
        rightExtensionMotor.setIdleMode(IdleMode.kBrake);


        //configure right motor to be inverted and to follow L motor
        rightExtensionMotor.setFollowerTo(leftExtensionMotor, true);

        leftExtensionMotor.setPositionSoftLimit(inchesToRotations(ELEVATOR.EXTENSION_INCHES_MIN), inchesToRotations(ELEVATOR.EXTENSION_INCHES_MAX));
        leftExtensionMotor.setCurrentLimit(80);
        rightExtensionMotor.setCurrentLimit(80);

        leftExtensionMotor.withGains(pid);

        SmartDashboard.putData("Elevator PID", pid);
        setPercentOutput(0);
        targetExtension = 0;
    }

    @Override
    public void periodic(){
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"current extension inches ", getExtensionPositionInches());
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"target inches ", targetExtension);
        Logger.recordOutput("Target extension in rotations", inchesToRotations(targetExtension));
        Logger.recordOutput(ELEVATOR.EXTENSION_LOG_PATH+"at position", atPosition());

        setExtensionPositionInches(targetExtension);
    }

    public double getExtensionPositionInches(){
        return rotationsToInches(leftExtensionMotor.getRotations());
    }

    public void setExtensionPositionInches(double targetInches){
        targetExtension = targetInches;
        leftExtensionMotor.setPosition(inchesToRotations(targetInches));
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
        return Utils.isWithin(getExtensionPositionInches(), targetExtension, ELEVATOR.EXTENSION_POSITION_TOLERANCE);
    }

    public Command setExtensionCommand(double targetExtension){
        return this.run(()-> setExtensionPositionInches(targetExtension));
    }

    public Command setExtensionPercentOutputCommand(double power) {
        return this.run(() -> setPercentOutput(power));
    }
    
    public Command stopCommand() {
        return this.run(() -> setPercentOutput(0));
    }

    //debug commands
    public Command gotoPosition(double position) {
        return this.run(() -> {targetExtension = position;});
    }


    

}
