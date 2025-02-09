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

public class Scoring extends SubsystemBase{

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;

    // public enum ScoringPositions {

    //     L1(0),
    //     L2(0),
    //     L3(0),
    //     L4(0),
    //     GROUND_ALGAE(0),
    //     HP_INTAKE(0),
    //     STOW(0),
    //     L2_ALGAE(0),
    //     L3_ALGAE(0),
    //     PROCESSOR(0),
    //     NET(0);

    //     public double elevatorPos = 0;
    //     public double wristPos = 0;
    //     public double clockArmPos = 0;

        
    //     private ElevatorPositions(double elevator) {

    //         elevatorPos = elevator;


    //     }

    // }

    public Scoring(){
        
        this.elevator = new Elevator();
        this.clockArm = new ClockArm();
        this.wrist = new Wrist();

    }

    @Override
    public void periodic(){
        
        elevator.periodic();
        clockArm.periodic();
        wrist.periodic();
    }

   
    public Command setExtensionCommand(double targetExtension){
        return this.run(()-> elevator.setExtensionPositionInches(targetExtension)).until(() -> elevator.atPosition());
    }

    public Command setExtensionPercentOutputCommand(double power) {
        return this.run(() -> elevator.setPercentOutput(power));
    }

    public Command setArmPercentOutputCommand(double power) {
        return this.run(() -> clockArm.setPercentOutput(power));
    }
    
    public Command stopArmCommand() {
        return this.runOnce(() -> clockArm.setPercentOutput(0));
    }

    public Command setArmPositionCommand(double degrees){
        return this.run(()-> clockArm.setArmPositionDegrees(degrees)).until(() -> clockArm.atPosition());
    }

    public Command setWristPercentOutput(double percent){
        return this.run(() -> wrist.setPercentOutput(percent));
    }

    public Command stopWrist(){
        return this.run(() -> wrist.setPercentOutput(0));
    }

    public Command setWristCommand(double degrees){
        return this.run(()-> wrist.setWristDegrees(degrees)).until(() -> wrist.atPosition());
    }

    public Command stopElevatorCommand() {
        return this.runOnce(() -> elevator.setPercentOutput(0));
    }

    public Command goToL4Command(){
        return setWristCommand(90).andThen(setExtensionCommand(18), setArmPositionCommand(135));
    }

    public Command stow(){
        return setWristCommand(0).andThen(setExtensionCommand(0), setArmPositionCommand(0));
    }

    public Command stopAll(){
        return stopElevatorCommand().andThen(stopWrist(), stopArmCommand());
    }

}
