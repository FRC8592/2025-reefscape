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
    private Arm arm;

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
        this.arm = new Arm();

    }

    @Override
    public void periodic(){
        
        elevator.periodic();
        arm.periodic();

    }

   
    public Command setExtensionCommand(double targetExtension){
        return this.run(()-> elevator.setExtensionPositionInches(targetExtension));
    }

    public Command setExtensionPercentOutputCommand(double power) {
        return this.run(() -> elevator.setPercentOutput(power));
    }

    public Command setArmPercentOutputCommand(double power) {
        return this.run(() -> arm.setPercentOutput(power));
    }
    
    public Command stopArmCommand() {
        return this.runOnce(() -> arm.setPercentOutput(0));
    }

    public Command stopElevatorCommand() {
        return this.runOnce(() -> elevator.setPercentOutput(0));
    }



}
