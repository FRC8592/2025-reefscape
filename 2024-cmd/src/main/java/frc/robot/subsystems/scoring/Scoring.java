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

    // Scoring mechanism objects
    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;

    // Define scoring mechanism positions for various activities
    private static ElevatorPositions scoringTargetPosition;
    private static ElevatorPositions userSelectedPosition;

    public static enum ElevatorPositions {
        STARTING(0, 0, 0),
        STOW(0, 0, 0),
        L1(12.7, -19.4, 195.0), 
        L2(11.0, -20, 168.0),
        L3(0, 139.0, 201.5),
        L4(19.5, 147.9, 195.7),

        GROUND_ALGAE(0, 0, 0),  // TODO: set algae positions
        L2_ALGAE(14, 30, 120),  // TODO: set algae positions
        L3_ALGAE(3, 150, 120),  // TODO: set algae positions
        PROCESSOR(0, 0, 0),     // TODO: set processor position
        NET(0, 0, 0);           // TODO: set net position

        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private ElevatorPositions(double elevator, double clockArm, double wrist) {
            elevatorPos = elevator;
            clockArmPos =  clockArm;
            wristPos = wrist;
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


    /** 
     * Set a future position for the scoring mechanism that will be triggered when {@Link #goToPosition()} is called.
     * @param eposition A position for the elevator, arm, and wrist as defined in {@Code ElevatorPositions}
     */
    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            userSelectedPosition = eposition;

            Logger.recordOutput(SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
        });
    }

    /**
     * Command the scoring mechanisms to move to a position previously set with {@Code #setPosition()}.
     * @return
     */
    public Command goToPosition(){
        Logger.recordOutput(SCORING.LOG_PATH + "goToPosition", true);

        return this.run(() -> {
            scoringTargetPosition = userSelectedPosition;
        }).until(() -> atPosition());
    }


    /**
     * Immediately begin moving the scoring mechanisms to the specified position.
     * @param eposition A position for the elevator, arm, and wrist as defined in {@Code ElevatorPositions}
     * @return
     */
    public Command goToSpecifiedPosition(ElevatorPositions eposition){
        return setPosition(eposition).andThen(goToPosition());
    }


    /**
     * Check to see if all scoring mechanisms are in position within tolerances specified by constants.
     * @return
     */
    public boolean atPosition(){
        return elevator.atPosition()&& wrist.atPosition() && clockArm.atPosition();
    }
    

    /**
     * Start the intake and run continuously until the coral sensor is tripped.
     * @return
     */
    public Command intakeCommand(){
        return intake.setIntakeCommand(0.5)
        .until(() -> intake.robotHasCoral())
        .andThen(intake.setIntakeCommand(0.25).withTimeout(0.25))
        .andThen(intake.stopIntakeCommand());
    }


    /**
     * Outtake coral from the collector.
     * <p>
     * TODO: Block this command when the scoring mechanism is stowed
     * @return
     */
    public Command outtakeCommand(){
        return intake.setIntakeCommand(-0.5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    /**
     * Stop the motion of the scoring mechanism.
     * <p>
     * TODO: The stop commands should lock in the current postion, as opposed to disabling motor power.
     * @return
     */
    public Command stopAllCommand(){
        Logger.recordOutput(SCORING.LOG_PATH + "stopAllCmd", true);
        return elevator.stopCommand().alongWith(wrist.stopCommand(), clockArm.stopCommand());
    }


    //
    // Update the commanded position of the Elevator, Arm, and Wrist
    // Implment safety features to preven these mechanisms from doing damage to the robot
    //
    public void periodic () {

        // Get the commanded position for each mechanism
        double targetElevatorPosition = scoringTargetPosition.elevatorPos;
        double targetWristPosition = scoringTargetPosition.wristPos;
        double targetArmPosition = scoringTargetPosition.clockArmPos;

        // Get the current position for each mechanism
        double currentElevatorPosition = elevator.getInches();
        double currentWristPosition = wrist.getDegrees();
        double currentArmPosition = clockArm.getDegrees();

        //
        // Allow the arm to move as far back as -20 degrees when the wrist is rotated outward to the scoring position.
        //
        // Check to see if the wrist scoring position is the currently commanded position.  If not, push the arm out
        // to 30 degrees to allow the wrist to rotate (probably inward) without damaging anything.
        //
        if (currentWristPosition > 150 && scoringTargetPosition.wristPos > 150) {
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, -20);
        } else {
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 30);
        }

        // The arm can move into the stowed (0) position ONLY when the elevator is at the stowed (0) position AND
        // The wrist is at the stowed (0) position AND neither the elevator nor wrist are commanded to move elsewhere.
        //
        // This logic overrides the targetArmPosition set in previous statements to allow the arm to stow.  The order
        // of these statements is important
        if(elevator.atPosition() && scoringTargetPosition.elevatorPos == 0 &&
           wrist.atPosition()    && scoringTargetPosition.wristPos == 0) {
            targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 0);
        }

        // Freeze the movement of the Elevator and Wrist to prevent damage if the arm is in too far.  This will normally
        // happen when the mechanism is leaving the stowed state, but may happen in other, unforeseen circumstances.
        if (currentArmPosition < 20) {
            targetWristPosition = currentWristPosition;
            targetElevatorPosition = currentElevatorPosition;
        }

        // Logging the original target positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalElevatorTarget", scoringTargetPosition.elevatorPos);
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalWristTarget", scoringTargetPosition.wristPos);
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalArmTarget", scoringTargetPosition.clockArmPos);

        // Logging the potentially modified target positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"TargetArmPostion", targetArmPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"TargetWristPostion", targetWristPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"TargetElevatorPostion", targetElevatorPosition);

        // Logging the current positions of scoring mechanisms
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentArmPostion", currentArmPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentWristPostion", currentWristPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"CurrentElevatorPostion", currentElevatorPosition);

        // Command the position of the Elevator, Arm, and Wrist mechanisms
        elevator.setInches(targetElevatorPosition);
        wrist.setDegrees(targetWristPosition);
        clockArm.setDegrees(targetArmPosition);        
    }
}
