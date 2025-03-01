package frc.robot.subsystems.scoring;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Scoring extends SubsystemBase {

    // Scoring mechanism objects
    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;
    // private DeepClimb deepclimb;

    // Define scoring mechanism positions for various activities
    private static ElevatorPositions scoringTargetPosition;
    private static ElevatorPositions userSelectedPosition;

    public static enum ElevatorPositions {
        
        L1_RIPTIDE(14.4, 5, 175, -0.43, 0.75),
        L2_RIPTIDE(11.8, 0, 180, -0.2, 0.75),
        L3_RIPTIDE(0, 165, 195, -0.43, 0.75),
        L4_RIPTIDE(19.5, 160, 200, -0.43, 0.75),
        GROUND_ALGAE_RIPTIDE(0, 0, 0, 0.5, -0.75),
        STOW_RIPTIDE(0, 0, 0, 0.5, 0.75),
        STOW_WITH_CORAL_RIPTIDE(0, 0, 20, 0.5, 0.75),
        L2_ALGAE_RIPTIDE(0, 50, 120, 0.5, -0.75),
        L3_ALGAE_RIPTIDE(3, 120, 160, 0.5, -0.75),
        PROCESSOR_RIPTIDE(0, 0, 0, -0.3, 0.75),
        NET_RIPTIDE(19.4, 150, 120, 1, -0.75),

        // L1_PERRY(14.4, 5, 175, -0.43, 0.75),
        L1_PERRY(0, 0, 0, 0, 0),
        L2_PERRY(6.8, 43, 185, -0.2, 0.75),
        L3_PERRY(0, 151, 210, -0.35, 0.75), //adjust wrist down from 200
        L4_PERRY(20, 155, 207, -0.43, 0.75), //arm adjusted from 165
        GROUND_ALGAE_PERRY(0, 0, 0, 0.5, -0.75),
        STOW_PERRY(0, 0, 0, 0.5, 0.75),
        // STOW_WITH_CORAL_PERRY(0, 0, 20, 0.5, 0.75),
        L2_ALGAE_PERRY(0, 50, 120, 0.5, -0.75),
        L3_ALGAE_PERRY(1.5, 120, 160, 0.5, -0.75),
        PROCESSOR_PERRY(0, 0, 0, 0.3, 0.75),
        NET_PERRY(20, 150, 95, 1, -0.75),
        DEEP_CLIMB_PERRY(0, 45, 0, 0, 0),

        STOP(0,0,0,-0.75,0.5);

        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        public double outtakeSpeed = 0;
        public double intakeSpeed = 0;
        
        private  ElevatorPositions(double elevator, double clockArm, double wrist, double outtakeSpeed, double intakeSpeed) {
            this.elevatorPos = elevator;
            this.wristPos = wrist;
            this.clockArmPos =  clockArm;
            this.outtakeSpeed = outtakeSpeed;
            this.intakeSpeed = intakeSpeed;
        }
        public static ElevatorPositions getL1(){return SHARED.IS_RIPTIDE?ElevatorPositions.L1_RIPTIDE:ElevatorPositions.L1_PERRY;}
        public static ElevatorPositions getL2(){return SHARED.IS_RIPTIDE?ElevatorPositions.L2_RIPTIDE:ElevatorPositions.L2_PERRY;}
        public static ElevatorPositions getL3(){return SHARED.IS_RIPTIDE?ElevatorPositions.L3_RIPTIDE:ElevatorPositions.L3_PERRY;}
        public static ElevatorPositions getL4(){return SHARED.IS_RIPTIDE?ElevatorPositions.L4_RIPTIDE:ElevatorPositions.L4_PERRY;}
        public static ElevatorPositions getGroundAlgae(){return SHARED.IS_RIPTIDE?ElevatorPositions.GROUND_ALGAE_RIPTIDE:ElevatorPositions.GROUND_ALGAE_PERRY;}
        public static ElevatorPositions getStow(){return SHARED.IS_RIPTIDE?ElevatorPositions.STOW_RIPTIDE:ElevatorPositions.STOW_PERRY;}
        public static ElevatorPositions getL2Algae(){return SHARED.IS_RIPTIDE?ElevatorPositions.L2_ALGAE_RIPTIDE:ElevatorPositions.L2_ALGAE_PERRY;}
        public static ElevatorPositions getL3Algae(){return SHARED.IS_RIPTIDE?ElevatorPositions.L3_ALGAE_RIPTIDE:ElevatorPositions.L3_ALGAE_PERRY;}
        public static ElevatorPositions getProcessor(){return SHARED.IS_RIPTIDE?ElevatorPositions.PROCESSOR_RIPTIDE:ElevatorPositions.PROCESSOR_PERRY;}
        public static ElevatorPositions getNet(){return SHARED.IS_RIPTIDE?ElevatorPositions.NET_RIPTIDE:ElevatorPositions.NET_PERRY;}
        public static ElevatorPositions stopped(){return ElevatorPositions.STOP;}
        public static ElevatorPositions getDeepClimb(){return ElevatorPositions.DEEP_CLIMB_PERRY;}
    }


    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;
        this.intake = intake;

        scoringTargetPosition = SHARED.IS_RIPTIDE?ElevatorPositions.STOW_RIPTIDE:ElevatorPositions.STOW_PERRY;
        userSelectedPosition = SHARED.IS_RIPTIDE?ElevatorPositions.STOW_RIPTIDE:ElevatorPositions.STOW_PERRY;
    }
    /**
     * Accepts a scoring position and sets user selected position to the position given.
     * @param position Position is a parameter that represents the desired position of the user.
     * @return Command that sets the user selected position to the position.
     */
    public Command setUserPosition(ElevatorPositions position){
        return this.runOnce(() -> {
            userSelectedPosition = position;
        });
    }

    /**
     * Sets the target position of the scoring subsystem to the position selected by the user.
     * @return Returns a command that sets the target position to the user selected position.
    */
    public Command applyUserPosition(){
        return this.runOnce(() -> {
            scoringTargetPosition = userSelectedPosition;
        });
    }
    /**
    * Accepts a scoring position and sets the target position to the given position.
    * @param position Position is a parameter that represents the desired position of the user.
    * @return Returns a command that sets the target position to the current position.
    */
    public Command goToPosition(ElevatorPositions position){
        return this.runOnce(() -> {
            scoringTargetPosition = position;
        });
    }

    public Command goToSpecifiedPositionCommand(ElevatorPositions position){
        return setUserPosition(position).andThen(applyUserPosition());
    }


    /**
     * Check to see if all scoring mechanisms are in position within tolerances specified by constants.
     * @return
     */
    public boolean atPosition(){
        return elevator.atPosition()&& wrist.atPosition() && clockArm.atPosition();
    }
    

    /**
     * Start the intake and run continuously until stopped.
     * @return
     */
    public Command intakeCommand(){
        return new DeferredCommand(() -> intake.setIntakeCommand(scoringTargetPosition.intakeSpeed).finallyDo(() -> {intake.stop();}), Set.of(this));
    }

    /**
     *  Runs the intake until the robot detects that it has a coral.
     * @return Returns a command to run the intake untill the beam brake is tripped.
     */
    public Command intakeUntilHasCoralCommand(){
        return new DeferredCommand(() -> 
            intake.setIntakeCommand(scoringTargetPosition.intakeSpeed)
            .until(() -> intake.robotHasCoral())
            .finallyDo(() -> {intake.stop();}),

            Set.of(this)
        );
    }


    /**
     * Outtake coral from the collector.
     * <p>
     * TODO: Block this command when the scoring mechanism is stowed
     * @return
     */
    public Command outtakeCoralCommand(){
        return new DeferredCommand(() -> intake.setIntakeCommand(scoringTargetPosition.outtakeSpeed).finallyDo(() -> {intake.stop();}), Set.of(this));
    }

    // public Command outtakeAlgaeCommand(){
    //     return new DeferredCommand(()-> intake.setIntakeCommand(1).finallyDo(() -> {intake.stop();}), Set.of(this));
    // }

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

    public boolean isAtPosition(ElevatorPositions position){
        return scoringTargetPosition == position && atPosition();
    }


    //
    // Update the commanded position of the Elevator, Arm, and Wrist
    // Implment safety features to preven these mechanisms from doing damage to the robot
    //
    public void periodic () {
        if(scoringTargetPosition != ElevatorPositions.STOP){
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

                //Aims to make the arm move before the elevator moves while going from L2 to L3
                if(!clockArm.atPosition() && !wrist.atPosition()){
                    targetElevatorPosition = currentElevatorPosition;
                }

            } else {
                if (currentWristPosition > 150 && currentElevatorPosition < 5) {
                    // if (currentArmPosition < 20) {
                    //     targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 0);
                    //     targetElevatorPosition = currentElevatorPosition;
                    // } else {
                    //     targetElevatorPosition = 10;
                    //     targetArmPosition = currentArmPosition;
                    // }
                } else {
                    targetArmPosition = Math.max(scoringTargetPosition.clockArmPos, 30);
                }

                if (currentArmPosition < 20 && scoringTargetPosition.clockArmPos != 0) {
                    targetWristPosition = currentWristPosition;
                    targetElevatorPosition = currentElevatorPosition;
                }
            }   
                // Moving to L3/L3: Ensure the arm reaches position before moving the wrist
                if (scoringTargetPosition == ElevatorPositions.getL3() || scoringTargetPosition == ElevatorPositions.getL4()){
                    if (clockArm.atPosition(40)) { // Only move wrist when arm is in position
                        targetWristPosition = scoringTargetPosition.wristPos;
                    } else {
                        targetWristPosition = currentWristPosition; // Hold wrist position until arm is ready
                    }
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

            if(scoringTargetPosition == ElevatorPositions.getStow()){
                if(currentWristPosition > 30){
                    targetArmPosition = currentArmPosition;
                }
            }
            // Freeze the movement of the Elevator and Wrist to prevent damage if the arm is in too far.  This will normally
            // happen when the mechanism is leaving the stowed state, but may happen in other, unforeseen circumstances.

            // // Deep climb safety code where the winch cannot operate inward unless the arm and such are not in the deep climb position
            // if(scoringTargetPosition == ElevatorPositions.getDeepClimb()){
            //     deepclimb.setDeepClimbPercentOutput(0);
            // }
            

            // Logging the target position of scoring mechanisms.
            Logger.recordOutput(SCORING.LOG_PATH+"TargetArmPostion", targetArmPosition);
            Logger.recordOutput(SCORING.LOG_PATH+"TargetWristPostion", targetWristPosition);
            Logger.recordOutput(SCORING.LOG_PATH+"TargetElevatorPostion", targetElevatorPosition);
    
            // Logging the current positions of scoring mechanisms.
            Logger.recordOutput(SCORING.LOG_PATH+"CurrentArmPostion", currentArmPosition);
            Logger.recordOutput(SCORING.LOG_PATH+"CurrentWristPostion", currentWristPosition);
            Logger.recordOutput(SCORING.LOG_PATH+"CurrentElevatorPostion", currentElevatorPosition);
            
            // Command the position of the Elevator, Arm, and Wrist mechanisms.
            elevator.setInches(targetElevatorPosition);
            wrist.setDegrees(targetWristPosition);
            clockArm.setDegrees(targetArmPosition);
        }
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalElevatorTarget", scoringTargetPosition.elevatorPos);
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalWristTarget", scoringTargetPosition.wristPos);
        Logger.recordOutput(SCORING.LOG_PATH+"OriginalArmTarget", scoringTargetPosition.clockArmPos);

        // Logging the potentially modified target positions of scoring mechanisms

        Logger.recordOutput(SCORING.LOG_PATH+"UserSelectedPosition", userSelectedPosition);
        Logger.recordOutput(SCORING.LOG_PATH+"TargetPosition", scoringTargetPosition);

        // These will log which position we are in for scoring
        SmartDashboard.putBoolean("L1", userSelectedPosition == ElevatorPositions.getL1());
        SmartDashboard.putBoolean("L2", userSelectedPosition == ElevatorPositions.getL2() || ElevatorPositions.getL2Algae() == userSelectedPosition);
        SmartDashboard.putBoolean("L3", userSelectedPosition == ElevatorPositions.getL3() || ElevatorPositions.getL3Algae() == userSelectedPosition);
        SmartDashboard.putBoolean("L4", userSelectedPosition == ElevatorPositions.getL4());

        

    }
}
