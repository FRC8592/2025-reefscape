package frc.robot.subsystems.elevator;

import java.lang.annotation.Target;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.commands.proxies.WaitUntilCommand;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Scoring extends SubsystemBase {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;

    private static ElevatorPositions targetPosition;

    public static enum ElevatorPositions {
        L1(15, 30, 120),
        L2(13.5, 30, 120),
        L3(2.5, 150, 120),
        L4(17.5, 150, 135),
        GROUND_ALGAE(0, 0, 0),
        STARTING(0, 0, 0),
        STOW(0, 10, -45),
        L2_ALGAE(14, 30, 120),
        L3_ALGAE(3, 150, 120),
        PROCESSOR(0, 0, 0),
        NET(0, 0, 0);
        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private  ElevatorPositions(double elevator, double clockArm, double wrist) {
          elevatorPos = elevator;
          wristPos = wrist;
          clockArmPos =  clockArm;
        }
    }

    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;

        targetPosition = ElevatorPositions.STARTING;

    }

    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
            targetPosition = eposition;
        }, new Subsystem[0]);
    }

    public Command goToL4Command(){
        return clockArm.setArmPositionCommand(()->135)
        .alongWith(elevator.setExtensionCommand(()->18), new WaitUntilCommand(wrist.setWristCommand(()->90), ()->clockArm.getArmPositionDegrees()>30));
    }

    public Command goToPosition(){
        return new ConditionalCommand(clockArm.setArmPositionCommand(()->45).until(()->clockArm.atPosition()), Commands.none(), ()->targetPosition.equals(ElevatorPositions.STOW)).andThen(
            elevator.setExtensionCommand(()->targetPosition.elevatorPos)
        .alongWith(wrist.setWristCommand(()->targetPosition.wristPos))
        .alongWith(clockArm.setArmPositionCommand(()->targetPosition.clockArmPos)));
    }

    // public Command starting(){
    //     return clockArm.setArmPositionCommand(30)
    //     .alongWith(elevator.setExtensionCommand(0)).andThen(wrist.setWristCommand(0), clockArm.setArmPositionCommand(0));
    // }

    public Command stopAll(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWrist(), clockArm.stopArmCommand());
    }

}
