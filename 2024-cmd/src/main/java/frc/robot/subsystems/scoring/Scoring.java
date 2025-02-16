package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Scoring extends SubsystemBase {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;
    private static double speed;

    private static ElevatorPositions targetPosition;

    public static enum ElevatorPositions {
        L1(11.8, 0, 180, -0.3),
        // elevator front: , back: , arm: , wrist: 
        L2(14.4, 5, 180),
        // front: , back: , arm: , wrist: 
        L3(0, 160, 195),
        // front: , back: , arm: , wrist: 
        L4(19.5, 160, 200),
        // front: , back: , arm: , wrist: 
        GROUND_ALGAE(0, 0, 0),
        STARTING(0, 0, 0),
        STOW(0, 0, 0), //0, 10, -45
        // elevator front: 0.117, back: -0.165, arm: -0.4277, wrist: -0.2622
        L2_ALGAE(14, 50, 120),
        L3_ALGAE(0, 160, 160),
        PROCESSOR(0, 0, 0),
        INTERMEDIATE(0, 45, 0),
        NET(0, 0, 0);
        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private  ElevatorPositions(double elevator, double clockArm, double wrist, double speedAmt) {
          elevatorPos = elevator;
          wristPos = wrist;
          clockArmPos =  clockArm;
          speed = speedAmt;
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

    public Command goToPosition(){
        return clockArm.setArmPositionCommand(()->45)
        .until(() -> clockArm.atPosition())
        .unless(()-> targetPosition.equals(ElevatorPositions.STOW) || clockArm.getArmPositionDegrees() >= 45)
        .andThen(
            clockArm.setArmPositionCommand(()->targetPosition.clockArmPos).alongWith(
                wrist.setWristPositionCommand(()->targetPosition.wristPos),
                elevator.setExtensionPositionCommand(()->targetPosition.elevatorPos)
            )
        ).until(() -> false);
    }

    public Command goToSpecifiedPosition(ElevatorPositions eposition){
        return setPosition(targetPosition).andThen(goToPosition());
    }

    public Command stowCommand(){
        return clockArm.setArmPositionCommand(()->45).onlyIf(()-> clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW.clockArmPos).until(()->clockArm.atPosition()) 
        .andThen(
            wrist.setWristPositionCommand(()->ElevatorPositions.STOW.wristPos).alongWith(
                elevator.setExtensionPositionCommand(()->ElevatorPositions.STOW.elevatorPos),
                clockArm.setArmPositionCommand(()->20)
            ).until(() -> elevator.atPosition() && wrist.atPosition()),
            clockArm.setArmPositionCommand(()->ElevatorPositions.STOW.clockArmPos)
        );
    }
    
    public Command intakeCommand(){
        return intake.setIntakeCommand(0.5).alongWith(stowCommand());
    }

    public Command outtakeCommand(){
        return intake.setIntakeCommand(-0.5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stopAllCommand(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWristCommand(), clockArm.stopArmCommand());
    }

}
