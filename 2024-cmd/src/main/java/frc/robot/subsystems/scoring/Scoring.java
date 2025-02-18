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

    private static ElevatorPositions targetPosition;

    public static enum ElevatorPositions {
        L1(11.8, 0, 180, -0.2),
        L2(14.4, 5, 180, -0.5),
        L3(0, 160, 195, -0.5),
        L4(19.5, 160, 200, -0.5),
       GROUND_ALGAE(0, 0, 0, 0.5),
        STOW(0, 0, 0, 0.5),
        STOW_WITH_CORAL(0, 0, 20, 0.5),
        L2_ALGAE(14, 50, 120, -0.5),
        L3_ALGAE(0, 160, 160, -0.5),
        PROCESSOR(0, 0, 0, -0.3),
        INTERMEDIATE(0, 45, 0, 0),
        NET(0, 0, 0, -0.5);
        
        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        public double outtakeSpeed = 0;
        
        private  ElevatorPositions(double elevator, double clockArm, double wrist, double speed) {
          elevatorPos = elevator;
          wristPos = wrist;
          clockArmPos =  clockArm;
          outtakeSpeed = speed;
        }
    }

    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;
        this.intake = intake;

        targetPosition = ElevatorPositions.STOW;

    }

    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
            targetPosition = eposition;
        }, new Subsystem[0]);
    }

    public Command goToPosition(){
        return clockArm.setArmPositionCommand(()->45).onlyIf(()->clockArm.getArmPositionDegrees() < 45).andThen(
            elevator.setExtensionPositionCommand(()->targetPosition.elevatorPos)
            .alongWith(wrist.setWristPositionCommand(()->targetPosition.wristPos))
            .alongWith(clockArm.setArmPositionCommand(()->targetPosition.clockArmPos))
        );
    }

    public Command goToSpecifiedPosition(ElevatorPositions eposition){
        return setPosition(eposition).andThen(goToPosition());
    }

    public Command stowCommand(){
        return new ConditionalCommand(clockArm.setArmPositionCommand(()->45).onlyIf(()-> clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW_WITH_CORAL.clockArmPos).until(()->clockArm.atPosition()) 
        .andThen(goToSpecifiedPosition(ElevatorPositions.STOW_WITH_CORAL)), clockArm.setArmPositionCommand(()->45).onlyIf(()-> clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW.clockArmPos).until(()->clockArm.atPosition()) 
        .andThen(goToSpecifiedPosition(ElevatorPositions.STOW)), ()->intake.robotHasCoral());
        
    }
    

    public Command intakeCommand(){
        return intake.setIntakeCommand(0.5).alongWith(stowCommand()).until(()->intake.robotHasCoral());
    }

    public Command outtakeCommand(){
        return intake.setIntakeCommand(targetPosition.outtakeSpeed).until(()->intake.robotHasCoral() == false).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stopAllCommand(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWristCommand(), clockArm.stopArmCommand());
    }

}
