package frc.robot.subsystems.scoring;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.TargetModel;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.LEDs;

public class Scoring extends SubsystemBase {

    // Scoring mechanism objects
    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;
    // private DeepClimb deepclimb;

    // Define scoring mechanism positions for various activities
    public static ElevatorPositions scoringTargetPosition;
    public static ElevatorPositions userSelectedPosition;

    private boolean isCoralMode = true;

    private Timer timer = new Timer();

    public static enum ElevatorPositions {
        
        // RIPTIDE POSITIONS 
        L1_RIPTIDE(14.4, 5, 175, -0.12, 0.75),
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


        
        // PERRY POSITIONS
        START_POSITION_PERRY(0, 0, 5, 0, 0),

        L1_PERRY(3, 9.5, 95, -0.09, 0.75),
        // L1_PERRY(13.3, 0, 107.8, 0, 0),
        // L2_PERRY(13.3, ARM.SAFE_ARM_TO_ROTATE_WRIST, 93.6, -0.15, 0.75),
        // L2_PERRY(13.3, 0, 93.6, -0.15, 0.75),

        L2_PERRY(18.7, 9.5, 139, -0.25, 0.75), //might be a good L2!

        L3_PERRY(0, 171, -214,-0.25, 0.75), //adjust wrist down from 200

        L4_PERRY(19.3, 171, -211.5, -0.33, 0.75), //arm adjusted from 165


        GROUND_ALGAE_PERRY(0, 35, 128, 0.5, -0.75),
        STOW_ALGAE_PERRY(0,11.7, 0, 0.5, -0.75),
        STOW_PERRY(0, 13.3, -54, 0.5, 0.75),
        // STOW_WITH_CORAL_PERRY(0, 0, 20, 0.5, 0.75),
        L2_ALGAE_PERRY(16.8, 30, 99, 0.5, -0.75),
        L3_ALGAE_PERRY(0, 141, 85, 0.5, -0.75),
        PROCESSOR_PERRY(0, 32, 92.3, 0.3, 0.75),
        NET_PERRY(19.3, 165, 15, 1, -0.72),
        DEEP_CLIMB_PERRY(0, 45, -73, 15.50, 0),
        NET_CATAPULT(19.3, 165, -55, 1, -0.75),

        STOP(0,0,0, 240, 0);

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
        public static ElevatorPositions getStowAlgae(){return ElevatorPositions.STOW_ALGAE_PERRY;}
        public static ElevatorPositions getNetCatapult(){return ElevatorPositions.NET_CATAPULT;}
    }


    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;
        this.intake = intake;

        scoringTargetPosition = SHARED.IS_RIPTIDE?ElevatorPositions.STOW_RIPTIDE:ElevatorPositions.STOW_PERRY;
        userSelectedPosition = SHARED.IS_RIPTIDE?ElevatorPositions.STOW_RIPTIDE:ElevatorPositions.STOW_PERRY;

        timer.start();
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
        return Commands.runOnce(() -> {
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

    public boolean atPosition(ElevatorPositions ep){
        return elevator.atPosition(ep.elevatorPos)&& wrist.atPosition(ep.wristPos) && clockArm.atPosition(ep.clockArmPos);
    }
    

    /**
     * Start the intake and run continuously until stopped.
     * @return
     */
    public Command intakeCommand(){
        return new DeferredCommand(
            () -> intake.setIntakeCommand(scoringTargetPosition.intakeSpeed).finallyDo(() -> {intake.stop();}),
            Set.of(this, intake)
        );
    }

    /**
     *  Runs the intake until the robot detects that it has a coral.
     * @return Returns a command to run the intake untill the beam brake is tripped.
     */
    public Command intakeUntilHasCoralCommand(){
        if(!Robot.isReal()){
            return new WaitCommand(2);
        }
        
        return new DeferredCommand(() -> 
            intake.setIntakeCommand(scoringTargetPosition.intakeSpeed)
            .until(() -> intake.robotHasCoral())
            .finallyDo(() -> {intake.stop();}),

            Set.of(this, intake)
        );
    }


    /**
     * Outtake coral from the collector.
     * <p>
     * TODO: Block this command when the scoring mechanism is stowed
     * @return
     */
    public Command outtakeCoralCommand(){
        return Robot.isReal()?new DeferredCommand(
            () -> intake.setIntakeCommand(scoringTargetPosition.outtakeSpeed).finallyDo(() -> {intake.stop();}),
            Set.of(this, intake)
        ):Commands.none();
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

    public Command setCoralMode(){
        return Commands.runOnce(() -> {this.isCoralMode = true;});
    }

    public Command setAlgaeMode(){
        return Commands.runOnce(() -> {this.isCoralMode = false;});
    }

    public boolean isCoralMode(){
        return this.isCoralMode;
    }

    public boolean isAlgaeMode(){
        return !isCoralMode();
    }

    public boolean isAtPosition(ElevatorPositions position){
        return atPosition();
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

            if(scoringTargetPosition == ElevatorPositions.getNet() && !elevator.atPosition()){
                targetWristPosition = -20;
            }

            if(scoringTargetPosition == ElevatorPositions.getL3Algae() && !clockArm.atPosition()){
                targetWristPosition = currentWristPosition;
            }

            //anytime we're moving the wrist, the arm should be out past the wrist rotate safe constant.
            if ( !wrist.atPosition(scoringTargetPosition.wristPos) ) {

                targetArmPosition = Math.max(ARM.SAFE_ARM_TO_ROTATE_WRIST, scoringTargetPosition.clockArmPos);

            }
            
            //if the arm is not extended, then don't move the elevator until it reaches the safe position
            if (
               
                (currentArmPosition < ARM.SAFE_ARM_TO_ROTATE_WRIST-10)

            ) {

                // this is a good way to tell a system not to move.
                // this doesn't work with the elevator as it drifts down.
                
                targetWristPosition = currentWristPosition;

                // this is the bad way to do it
                targetElevatorPosition = Math.round(currentElevatorPosition*5.0)/5.0;

            }

            //if the wrist is not in a safe position (anywhere but down), then don't let the arm move
            
            // if ( (currentWristPosition < -2 || currentWristPosition > 90)) {

            //     targetArmPosition = Math.min(ARM.SAFE_ARM_TO_ROTATE_WRIST, targetArmPosition);

            // }

            //if the wrist is not in a safe position, then don't move the elevator down.

            // if ( currentWristPosition < -2 || currentWristPosition > 90 ) {

            //     targetElevatorPosition = currentElevatorPosition;

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
        Logger.recordOutput(SCORING.LOG_PATH+"AtPosition", atPosition());

        // These will log which position we are in for scoring
        SmartDashboard.putBoolean("L1", userSelectedPosition == ElevatorPositions.getL1());
        SmartDashboard.putBoolean("L2", userSelectedPosition == ElevatorPositions.getL2() || ElevatorPositions.getL2Algae() == userSelectedPosition);
        SmartDashboard.putBoolean("L3", userSelectedPosition == ElevatorPositions.getL3() || ElevatorPositions.getL3Algae() == userSelectedPosition);
        SmartDashboard.putBoolean("L4", userSelectedPosition == ElevatorPositions.getL4());
        
        LEDs.setHasCoral(intake.robotHasCoral());
        LEDs.setCoralMode( isCoralMode );
        SmartDashboard.putBoolean("Coral mode", isCoralMode);
        
    }
}
