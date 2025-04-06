// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.DeepClimb;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.ScoreCoral;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.scoring.ClockArm;
import frc.robot.subsystems.scoring.Elevator;
import frc.robot.subsystems.scoring.Intake;
import frc.robot.subsystems.scoring.Scoring;
import frc.robot.subsystems.scoring.Wrist;
import frc.robot.subsystems.scoring.Scoring.ElevatorPositions;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );
    public static final CommandGenericHID coralController = new CommandGenericHID(
        CONTROLLERS.CORAL_SELECTOR_PORT
    );

    // The robot's subsystems
    private final Swerve swerve;
    private final Vision vision1;
    // private final Vision vision2;
    private Scoring scoring = null;
    private final DeepClimb deepclimb;

    private final ClockArm clockArm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Intake intake;
    private final LEDs leds;
    private ScoreCoral scoreCoral;
    private OdometryUpdates odometryUpdates;
    
    //TODO: Add all controls here
    //Driver controls
    

    private final Trigger ENABLED = new Trigger(() -> DriverStation.isEnabled()).and(()->DriverStation.isTeleop());

    private final Trigger INTAKE = driverController.leftTrigger();
    private final Trigger SCORE = driverController.rightTrigger();
    // private final Trigger SCORE_ALGAE = driverController.rightTrigger().and(()->scoring.isAlgaeMode());

    private final Trigger SLOW_MODE = driverController.rightBumper();
    private final Trigger RESET_HEADING = driverController.back();
    private final Trigger ROBOT_RELATIVE = driverController.y();
    // private final Trigger SNAP_NORTH = driverController.pov(0);
    // private final Trigger SNAP_SOUTH = driverController.pov(180);
    // private final Trigger SNAP_EAST = driverController.pov(90);
    // private final Trigger SNAP_WEST = driverController.pov(270);

    private final Trigger STOW = driverController.x();
    private final Trigger GO_TO_POSITION = driverController.a();
    private final Trigger ALIGN_TO_REEF = driverController.leftBumper();

    //private final Trigger LED_TEST = driverController.b();

    private final Trigger DEEP_CLIMB_INTAKE = driverController.b();
    private final Trigger WINCH_UP = driverController.pov(0).or(driverController.pov(45)).or(driverController.pov(315));
    private final Trigger WINCH_DOWN = driverController.pov(180).or(driverController.pov(225)).or(driverController.pov(135));
    private final Trigger DEEP_CLIMB_DEPLOY = driverController.pov(90);

    
    //Operator controls
    
    private final Trigger PRIME_L4 = (coralController.button(5).or(coralController.button(7))).and(()->scoring.isCoralMode());
    private final Trigger PRIME_L3 = (coralController.button(6).or(coralController.button(8))).and(()->scoring.isCoralMode());
    private final Trigger PRIME_L2 = (coralController.button(1).or(coralController.button(2))).and(()->scoring.isCoralMode());
    private final Trigger PRIME_L1 = (coralController.button(4).or(coralController.button(3))).and(()->scoring.isCoralMode());
    
    private final Trigger ALIGN_RIGHT = (coralController.button(2).or(coralController.button(3)).or(coralController.button(8)).or(coralController.button(7))).and(()->scoring.isCoralMode());
    private final Trigger ALIGN_LEFT = (coralController.button(1).or(coralController.button(4)).or(coralController.button(6)).or(coralController.button(5))).and(()->scoring.isCoralMode());
    
    // private final Trigger ALIGN_CENTER = (coralController.button(2).or(coralController.button(3)).or(coralController.button(8)).or(coralController.button(7))).and(()->scoring.isAlgaeMode());
    
    private final Trigger PRIME_PROCESSOR = coralController.button(4).and(()->scoring.isAlgaeMode());
    private final Trigger PRIME_L2_ALGAE = coralController.button(1).and(()->scoring.isAlgaeMode());
    private final Trigger PRIME_L3_ALGAE = coralController.button(6).and(()->scoring.isAlgaeMode());
    private final Trigger PRIME_NET = coralController.button(5).and(()->scoring.isAlgaeMode());
    
    private final Trigger GROUND_ALGAE_PERRY = coralController.button(3).and(()->scoring.isAlgaeMode());
    private final Trigger STOW_ALGAE_PERRY = coralController.button(2).and(()->scoring.isAlgaeMode());
    private final Trigger DEEP_CLIMB_POSITION = coralController.button(8).and(()->scoring.isAlgaeMode());
    private final Trigger CATAPULT_POSITION = coralController.button(7).and(()->scoring.isAlgaeMode());

    private final Trigger ALGAE_INTAKE = coralController.button(3).and(()->scoring.isAlgaeMode());
    // private final Trigger GROUND_INTAKE = coralController.button();
    private final Trigger MODE_SWITCH_ALGAE = coralController.button(10).or(coralController.axisGreaterThan(1, 0.1));
    private final Trigger MODE_SWITCH_CORAL = coralController.button(13).or(coralController.axisLessThan(1, -0.1));

    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        LEDs.init();
        swerve = new Swerve();
        vision1 = new Vision(CORAL_ALIGN.CAMERA_NAME, CORAL_ALIGN.CAMERA_OFFSETS);
        // vision2 = new Vision(CORAL_ALIGN.CAMERA_2_NAME, CORAL_ALIGN.CAMERA_2_OFFSETS);
        scoreCoral = new ScoreCoral(swerve);
        odometryUpdates = new OdometryUpdates(swerve, vision1);
        
        clockArm = new ClockArm();
        wrist = new Wrist();
        elevator = new Elevator();
        intake = new Intake();
        deepclimb = new DeepClimb();
        leds = new LEDs();
        
        scoring = new Scoring(elevator, clockArm, wrist, intake);
        
        passSubsystems();
        configureBindings();
        configureDefaults();
        
        AutoManager.prepare();
        LEDs.init();
    }

    /**
     * Pass subsystems everywhere they're needed
     */
    private void passSubsystems(){
        AutoManager.addSubsystems(swerve, scoring, leds);
        AutoCommand.addSubsystems(swerve, scoring, intake, leds, scoreCoral);
        LargeCommand.addSubsystems(swerve, scoring, leds, scoreCoral);
        NewtonCommands.addSubsystems(swerve, scoring, leds);
        Suppliers.addSubsystems(swerve, scoring, leds);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                -driverController.getLeftX(),
                -driverController.getLeftY(),
                -driverController.getRightX()
            ), DriveModes.AUTOMATIC);
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        setDefaultCommand(
            intake, intake.run(
                () -> {intake.setIntakePercentOutput(-0.2);}
            ).onlyIf(
                () -> (scoring.isAlgaeMode() && !(scoring.scoringTargetPosition == ElevatorPositions.getNet()) )
            ).repeatedly()
        );
        
        // setDefaultCommand(elevator, elevator.stopCommand());
        // setDefaultCommand(wrist, wrist.stopCommand());
        // setDefaultCommand(clockArm, clockArm.stopCommand());
        // setDefaultCommand(intake, intake.stopIntakeCommand());

    }

    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {

        ENABLED.onTrue(
            scoring.goToPosition(ElevatorPositions.stopped()).andThen(scoring.stopAllCommand()).andThen(deepclimb.setDeepClimbCommand(0))
        );

        //------------------------------ SWERVE COMMANDS ------------------------------//
        SLOW_MODE.onTrue(
            // The Commands.runOnce (instead of swerve.runOnce) is a special case here
            // to allow this to run while other swerve commands (the default driving
            // command, for example) run. This is usually a horrible idea and shouldn't
            // be used outside of special cases like this.

            // The .ignoringDisable makes sure slow mode won't get stuck on or off if
            // the robot is disabled.
            Commands.runOnce(() -> swerve.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setSlowMode(false)).ignoringDisable(true)
        );

        RESET_HEADING.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> swerve.resetHeading())
        );

        ROBOT_RELATIVE.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        // SNAP_NORTH.whileTrue(
        //     swerveSnapToCommand(
        //         Rotation2d.fromDegrees(0),
        //         () -> -driverController.getLeftX(),
        //         () -> -driverController.getLeftY()
        //     )
        // );

        // SNAP_SOUTH.whileTrue(
        //     swerveSnapToCommand(
        //         Rotation2d.fromDegrees(180),
        //         () -> -driverController.getLeftX(),
        //         () -> -driverController.getLeftY()
        //     )
        // );

        // SNAP_EAST.whileTrue(
        //     swerveSnapToCommand(
        //         Rotation2d.fromDegrees(270),
        //         () -> -driverController.getLeftX(),
        //         () -> -driverController.getLeftY()
        //     )
        // );

        // SNAP_WEST.whileTrue(
        //     swerveSnapToCommand(
        //         Rotation2d.fromDegrees(90),
        //         () -> -driverController.getLeftX(),
        //         () -> -driverController.getLeftY()
        //     ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        // );

        //------------------------------ OPERATOR POSITION COMMANDS ------------------------------//
        PRIME_L1.onTrue(scoring.setUserPosition(ElevatorPositions.getL1()).ignoringDisable(true));
        PRIME_L2.onTrue(scoring.setUserPosition(ElevatorPositions.getL2()).ignoringDisable(true));
        PRIME_L3.onTrue(scoring.setUserPosition(ElevatorPositions.getL3()).ignoringDisable(true));
        PRIME_L4.onTrue(scoring.setUserPosition(ElevatorPositions.getL4()).ignoringDisable(true));

        PRIME_PROCESSOR.onTrue(scoring.setUserPosition(ElevatorPositions.getProcessor()).ignoringDisable(true));
        PRIME_L2_ALGAE.onTrue(scoring.setUserPosition(ElevatorPositions.getL2Algae()).ignoringDisable(true));
        PRIME_L3_ALGAE.onTrue(scoring.setUserPosition(ElevatorPositions.getL3Algae()).ignoringDisable(true));
        PRIME_NET.onTrue(scoring.setUserPosition(ElevatorPositions.getNet()).ignoringDisable(true));
        ALGAE_INTAKE.onTrue(scoring.setUserPosition(ElevatorPositions.getGroundAlgae()).ignoringDisable(true));
        DEEP_CLIMB_POSITION.onTrue(scoring.setUserPosition(ElevatorPositions.getDeepClimb()).ignoringDisable(true));
        CATAPULT_POSITION.onTrue(scoring.setUserPosition(ElevatorPositions.getNetCatapult()).ignoringDisable(true));

        MODE_SWITCH_ALGAE.onTrue(scoring.setAlgaeMode());

        MODE_SWITCH_CORAL.onTrue(scoring.setCoralMode());

        ALIGN_LEFT.onTrue(Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left)));
        ALIGN_RIGHT.onTrue(Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right)));

        //------------------------------ DRIVER COMMANDS ------------------------------//

        STOW.whileTrue(scoring.goToPosition(ElevatorPositions.getStow()));

        GO_TO_POSITION.onTrue(new DeferredCommand(()->{
            if(Scoring.userSelectedPosition == ElevatorPositions.getNet() && Scoring.scoringTargetPosition != ElevatorPositions.getL3Algae()){
                return  scoring.goToSpecifiedPositionCommand(ElevatorPositions.getStowAlgae())
                .andThen(new WaitUntilCommand(()->scoring.isAtPosition(ElevatorPositions.getStowAlgae())))
                .andThen(scoring.goToSpecifiedPositionCommand(ElevatorPositions.getNet()));

            } else if(Scoring.userSelectedPosition == ElevatorPositions.getNetCatapult()){
                return scoring.goToSpecifiedPositionCommand(ElevatorPositions.getNetCatapult())
                .andThen(new WaitUntilCommand(()->(clockArm.getDegrees() > 130 && wrist.atPosition())))
                .andThen(intake.setIntakeCommand(1));

            } else {
                return scoring.applyUserPosition();
            }
        }, Set.of(scoring, intake, clockArm, wrist)));

        INTAKE.whileTrue(new DeferredCommand(() -> scoring.intakeCommand(), Set.of(scoring))).onFalse(intake.stopIntakeCommand());
        
        SCORE.whileTrue(new DeferredCommand(() -> scoring.outtakeCoralCommand(), Set.of(scoring))).onFalse(intake.stopIntakeCommand());
        // SCORE_ALGAE.whileTrue(new DeferredCommand(() -> scoring.outtakeAlgaeCommand(), Set.of(scoring))).onFalse(intake.stopIntakeCommand());

        ALIGN_TO_REEF.whileTrue(
            new DeferredCommand(
                () -> scoreCoral.driveToClosestReefTag(),
                Set.of(swerve)
            ) 
        );

        //LED_TEST.onTrue(setLEDsCommand(LEDS.TEAL)).onFalse(setLEDsCommand(LEDS.OFF));

        DEEP_CLIMB_INTAKE.whileTrue(deepclimb.setDeepClimbIntakeCommand(-1));

        DEEP_CLIMB_DEPLOY.onTrue(
            scoring.goToPosition(ElevatorPositions.getDeepClimb()).andThen(
                deepclimb.setDeepClimbGrabPositionCommand()
            )
        );//.onFalse(deepclimb.setDeepClimbIntakeCommand(0));

        WINCH_UP.whileTrue(
            deepclimb.setDeepClimbCommand(-1)
            //  .onlyIf(
            //      () -> scoring.isAtPosition(ElevatorPositions.getDeepClimb())
            //  )
        ).onFalse(deepclimb.setDeepClimbCommand(0));
        
        WINCH_DOWN.whileTrue(
            deepclimb.setDeepClimbCommand(1)
            // .onlyIf(
            //     () 
            //     -> scoring.isAtPosition(ElevatorPositions.getDeepClimb())
            // )
        ).onFalse(deepclimb.setDeepClimbCommand(0));

        GROUND_ALGAE_PERRY.onTrue(
            scoring.setUserPosition(ElevatorPositions.getGroundAlgae())
        );

        STOW_ALGAE_PERRY.onTrue(
            scoring.setUserPosition(ElevatorPositions.getStowAlgae())
        );

    };



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }

    /**
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command){
        if(command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf){
            subsystem.setDefaultCommand(command);
        }
        else{
            //If you want to force-allow setting a cancel-incoming default command, directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }
}
