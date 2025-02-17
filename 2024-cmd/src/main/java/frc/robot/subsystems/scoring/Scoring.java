package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.commands.proxies.WaitUntilCommand;

public class Scoring extends SubsystemBase {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;

    private static ElevatorPositions scoringTargetPosition;
    private static ElevatorPositions userSelectedPosition;

    public static enum ElevatorPositions {
        L1(12.7, -19.4, 195.0), 
        // elevator front: , back: , arm: , wrist:
        L2(11.0, -20, 168.0),
        // front: , back: , arm: , wrist: 
        L3(0, 139.0, 201.5),
        // front: , back: , arm: , wrist: 
        L4(19.5, 147.9, 195.7),
        // front: , back: , arm: , wrist: 
        GROUND_ALGAE(0, 0, 0),
        STARTING(0, 0, 0),
        STOW(0, 0, 0), //0, 10, -45
        // elevator front: 0.117, back: -0.165, arm: -0.4277, wrist: -0.2622
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

        scoringTargetPosition = ElevatorPositions.STARTING;
        userSelectedPosition = ElevatorPositions.STARTING;

    }

    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            Logger.recordOutput(SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
            userSelectedPosition = eposition;
        }, new Subsystem[0]);
    }

    public Command goToPosition(){
        Logger.recordOutput(SCORING.LOG_PATH + "goToPosition", true);
        return this.run(() -> {
            scoringTargetPosition = userSelectedPosition;
        }).until(() -> atPosition());
        
    }

    public Command goToSpecifiedPosition(ElevatorPositions eposition){
        return setPosition(eposition).andThen(goToPosition());
    }

    public boolean atPosition(){
        return elevator.atPosition()&& wrist.atPosition() && clockArm.atPosition();
    }
    

    public Command intakeCommand(){
        return intake.setIntakeCommand(0.5)
        .until(() -> intake.robotHasCoral())
        .andThen(intake.setIntakeCommand(0.25).withTimeout(0.25))
        .andThen(intake.stopIntakeCommand());
    }

    public Command outtakeCommand(){
        return intake.setIntakeCommand(-0.5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stopAllCommand(){
        Logger.recordOutput(SCORING.LOG_PATH + "stopAllCmd", true);
        return elevator.stopCommand().alongWith(wrist.stopCommand(), clockArm.stopCommand());
    }

    public void periodic () {
        double targetElevatorPosition = scoringTargetPosition.elevatorPos;
        double targetWristPosition = scoringTargetPosition.wristPos;
        double targetArmPosition = scoringTargetPosition.clockArmPos;

        double currentElevatorPosition = elevator.getInches();
        double currentWristPosition = wrist.getDegrees();
        double currentArmPosition = clockArm.getDegrees();

        // The arm can move as far back as -20 degrees when the wrist is in a scoring position and is commanded to be in one
        if (currentWristPosition > 150 && scoringTargetPosition.wristPos > 150) {
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, -20);
        } else {
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 30);
        }

        // When the elevator is stowed, the arm can move to the stowed position
        if((elevator.atPosition() && scoringTargetPosition.elevatorPos == 0)){
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 0);
        }
        // Possible conflict - elevator arm wrist: (0, 0, 0) (19.5, 147.9, 195.7); Set to thirty until elevator reaches stow. once elevator reaches stow, the wrist cant move.
        // Wrist and arm dont move until arm is in a safe position for wrist movement
        if (currentArmPosition < 20) {
            targetWristPosition = currentWristPosition;
            targetElevatorPosition = currentElevatorPosition;
        }


        // Logging our original target positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalElevatorTarget", scoringTargetPosition.elevatorPos);
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalWristTarget", scoringTargetPosition.wristPos);
        Logger.recordOutput(SCORING.LOG_PATH+"originalArmTarget", scoringTargetPosition.clockArmPos);

        // Logging target positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"TargetArmPostion", targetArmPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"TargetWristPostion", targetWristPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"TargetElevatorPostion", targetElevatorPosition);

        // Logging current positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentArmPostion", currentArmPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentWristPostion", currentWristPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentElevatorPostion", currentElevatorPosition);


        elevator.setInches(targetElevatorPosition);
        wrist.setDegrees(targetWristPosition);
        clockArm.setDegrees(targetArmPosition);

        
    }
}
