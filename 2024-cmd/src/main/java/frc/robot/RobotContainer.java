// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import static frc.robot.commands.NewtonCommands.*;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.ScoreCoral;
import frc.robot.subsystems.ScoreCoral.LeftOrRight;
import frc.robot.subsystems.ScoreCoral.ScoreLevels;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.subsystems.vision.Vision;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.elevator.ClockArm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Scoring;
import frc.robot.subsystems.elevator.Scoring.ElevatorPositions;
import frc.robot.subsystems.elevator.Wrist;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private final Vision vision;
    private final Scoring scoring;

    private final ClockArm clockArm;
    private final Elevator elevator;
    private final Wrist wrist;
    private final Intake intake;
   
    private ScoreCoral scoreCoral;
    private OdometryUpdates odometryUpdates;
    

    //TODO: Add all controls here
    //Driver controls

    private boolean isCoralMode = true;

    private final Trigger INTAKE = driverController.leftTrigger();
    private final Trigger SCORE = driverController.rightTrigger();

    private final Trigger SLOW_MODE = driverController.rightBumper();
    private final Trigger RESET_HEADING = driverController.back();
    private final Trigger ROBOT_RELATIVE = driverController.leftBumper();
    private final Trigger SNAP_NORTH = driverController.pov(0);
    private final Trigger SNAP_SOUTH = driverController.pov(180);
    private final Trigger SNAP_EAST = driverController.pov(90);
    private final Trigger SNAP_WEST = driverController.pov(270);


    private final Trigger GO_TO_L4 = driverController.y();
    private final Trigger STOW = driverController.x();
    private final Trigger GO_TO_POSITION = driverController.a();

    //Operator controls

    private final Trigger PRIME_L1 = (coralController.button(3).or(coralController.button(4))).and(()->isCoralMode);
    private final Trigger PRIME_L2 = (coralController.button(2).or(coralController.button(1))).and(()->isCoralMode);
    private final Trigger PRIME_L3 = (coralController.button(8).or(coralController.button(6))).and(()->isCoralMode);
    private final Trigger PRIME_L4 = (coralController.button(7).or(coralController.button(5))).and(()->isCoralMode);

    private final Trigger ALIGN_RIGHT = (coralController.button(2).or(coralController.button(3)).or(coralController.button(8)).or(coralController.button(7))).and(()->isCoralMode);
    private final Trigger ALIGN_LEFT = (coralController.button(1).or(coralController.button(4)).or(coralController.button(6)).or(coralController.button(5))).and(()->isCoralMode);

    private final Trigger ALIGN_CENTER = (coralController.button(2).or(coralController.button(3)).or(coralController.button(8)).or(coralController.button(7))).and(()->!isCoralMode);
    
    private final Trigger PRIME_L2_ALGAE = coralController.button(4).and(()->!isCoralMode);
    private final Trigger PRIME_L3_ALGAE = coralController.button(6).and(()->!isCoralMode);
    private final Trigger PRIME_NET = coralController.button(5).and(()->!isCoralMode);
    private final Trigger PRIME_PROCESSOR = coralController.button(1).and(()->!isCoralMode);
    // private final Trigger GROUND_INTAKE = coralController.button();
    private final Trigger MODE_SWITCH_ALGAE = coralController.button(9);
    private final Trigger MODE_SWITCH_CORAL = coralController.button(10);

    


    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve();
        vision = new Vision();
        scoreCoral = new ScoreCoral(swerve);
        odometryUpdates = new OdometryUpdates(swerve, vision);
        
        clockArm = new ClockArm();
        wrist = new Wrist();
        elevator = new Elevator();
        intake = new Intake();
        
        scoring = new Scoring(elevator, clockArm, wrist);

        passSubsystems();
        configureBindings();
        configureDefaults();

        AutoManager.prepare();
    }

    /**
     * Pass subsystems everywhere they're needed
     */
    private void passSubsystems(){
        AutoManager.addSubsystems(swerve);
        AutoCommand.addSubsystems(swerve);
        LargeCommand.addSubsystems(swerve);
        NewtonCommands.addSubsystems(swerve, scoring);
        Suppliers.addSubsystems(swerve);
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

        
        setDefaultCommand(elevator, elevator.stopElevatorCommand());
        setDefaultCommand(wrist, wrist.stopWrist());
        setDefaultCommand(clockArm, clockArm.stopArmCommand());

    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // Driver controls:
        // Operator:
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

        SNAP_NORTH.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        SNAP_SOUTH.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        SNAP_EAST.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        SNAP_WEST.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        PRIME_L1.onTrue(scoring.setPosition(ElevatorPositions.L1));
        PRIME_L2.onTrue(scoring.setPosition(ElevatorPositions.L2));
        PRIME_L3.onTrue(scoring.setPosition(ElevatorPositions.L3));
        PRIME_L4.onTrue(scoring.setPosition(ElevatorPositions.L4));
        PRIME_L2_ALGAE.onTrue(scoring.setPosition(ElevatorPositions.L2_ALGAE));
        PRIME_L3_ALGAE.onTrue(scoring.setPosition(ElevatorPositions.L3_ALGAE));
        PRIME_NET.onTrue(scoring.setPosition(ElevatorPositions.NET));
        PRIME_PROCESSOR.onTrue(scoring.setPosition(ElevatorPositions.PROCESSOR));

        // GROUND_INTAKE.onTrue(scoring.setPosition(ElevatorPositions.GROUND_ALGAE));
        
        GO_TO_L4.whileTrue(scoring.setPosition(ElevatorPositions.L4).andThen(scoring.goToPosition()));
        STOW.whileTrue(scoring.setPosition(ElevatorPositions.STOW).andThen(scoring.goToPosition()));
        GO_TO_POSITION.whileTrue(scoring.goToPosition()).onFalse(scoring.stopAll());


        INTAKE.whileTrue(scoring.setPosition(ElevatorPositions.STOW).andThen(scoring.goToPosition().alongWith(intake.setIntakeCommand(0.5)))).onFalse(intake.stopIntakeCommand());
        SCORE.whileTrue(intake.setIntakeCommand(-0.5)).onFalse(intake.stopIntakeCommand());

        // MODE_SWITCH_ALGAE.onTrue(Commands.runOnce(()->{
        //     isCoralMode=false; 
        //     Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/isCoralMode", isCoralMode);
        // }, new Subsystem[0]));

        // MODE_SWITCH_CORAL.onTrue(Commands.runOnce(()->{
        //     isCoralMode=true; 
        //     Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/isCoralMode", isCoralMode);
        // }, new Subsystem[0]));

        // ALIGN_LEFT.onTrue(Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left, ScoreLevels.Level1)));
        // ALIGN_RIGHT.onTrue(Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right, ScoreLevels.Level1)));

        // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
        // this activates tesla full self driving
        // ALIGN_TO_REEF.whileTrue(
        //     new DeferredCommand(
        //         () -> scoreCoral.driveToClosestReefTag(),
        //         Set.of(swerve)
        //     ) 
        // );

        // ALIGN_TO_HPST.whileTrue(
        //     new DeferredCommand(
        //         () -> scoreCoral.driveToClosestHumanPlayerStation(),
        //         Set.of(swerve)
        //     ) 
        // );

        // operatorController.leftTrigger().onTrue(
        //     intakeCommand()
        // ).onFalse(
        //     stopIntakeComand()
        // );

        // operatorController.rightTrigger().onTrue(
        //     outakeCommand()
        // ).onFalse(
        //     stopIntakeComand()
        // );
        
        // D-input; LS; Turbo: Off
        //
        //Joystick is here
        //
        // L4: 3    R4: 4
        // L3: 2    R3: 1
        // L2: 8    R2: 6
        // L1: 7    R1: 5

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_L1).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left, ScoreLevels.Level1))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_L2).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left, ScoreLevels.Level2))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_L3).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left, ScoreLevels.Level3))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_L4).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Left, ScoreLevels.Level4))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_R1).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right, ScoreLevels.Level1))
        // );
        
        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_R2).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right, ScoreLevels.Level2))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_R3).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right, ScoreLevels.Level3))
        // );

        // coralController.button(CONTROLLERS.CORAL_CONTROLLER_R4).onTrue(
        //     Commands.runOnce(() -> scoreCoral.setPosition(LeftOrRight.Right, ScoreLevels.Level4))
        // );
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
