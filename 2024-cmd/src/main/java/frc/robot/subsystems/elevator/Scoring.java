package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.proxies.WaitUntilCommand;
import frc.robot.subsystems.Intake;

public class Scoring extends SubsystemBase {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;

    private static ElevatorPositions targetPosition;

    public static enum ElevatorPositions {
        L1(15, 30, 120),
        L2(13, 30, 120),
        L3(2, 180, 120),
        L4(10., 180, 135),
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

    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;
        this.intake = intake;

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
        .alongWith(elevator.setExtensionPositionCommand(()->18), new WaitUntilCommand(wrist.setWristPositionCommand(()->90), ()->clockArm.getArmPositionDegrees()>30));
    }

    public Command goToPosition(){
        return new ConditionalCommand(
            clockArm.setArmPositionCommand(()->45).until(()->clockArm.atPosition()), 
            Commands.none(), 
            ()->targetPosition.equals(ElevatorPositions.STOW) && clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW.clockArmPos).andThen(
            elevator.setExtensionPositionCommand(()->targetPosition.elevatorPos)
        .alongWith(wrist.setWristPositionCommand(()->targetPosition.wristPos))
        .alongWith(clockArm.setArmPositionCommand(()->targetPosition.clockArmPos)));
    }

    public Command stowCommand(){
        return new ConditionalCommand(
            clockArm.setArmPositionCommand(()->45).until(()->clockArm.atPosition()), 
            Commands.none(), 
            ()-> clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW.clockArmPos).andThen(
            elevator.setExtensionPositionCommand(()->ElevatorPositions.STOW.elevatorPos)
        .alongWith(wrist.setWristPositionCommand(()->ElevatorPositions.STOW.wristPos))
        .alongWith(clockArm.setArmPositionCommand(()->ElevatorPositions.STOW.elevatorPos)));
    }

    
    public Command intakeCommand(){
        return stowCommand().alongWith(intake.setIntakeCommand(0.5)).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stopAllCommand(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWristCommand(), clockArm.stopArmCommand());
    }

}
