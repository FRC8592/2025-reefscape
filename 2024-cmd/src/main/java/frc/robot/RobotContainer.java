// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.ClockArm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Wrist;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    // The robot's subsystems
    private final Swerve swerve;
    private final Vision vision;
    private final Intake intake;
    private final Elevator elevator;
    private final ClockArm clockArm;
    private final Wrist wrist;
    //TODO: Add more subsystems here

    //TODO: Add all controls here
    //Driver controls
    private final Trigger INTAKE = driverController.leftTrigger();
    private final Trigger SCORE = driverController.rightTrigger();
    private final Trigger ALIGN_TO_SCORE = driverController.leftBumper();
    private final Trigger STOW = driverController.button(100);
    private final Trigger PRIME = driverController.button(100);

    private final Trigger SLOW_MODE = driverController.rightBumper();
    private final Trigger RESET_HEADING = driverController.back();
    private final Trigger ROBOT_RELATIVE = driverController.leftBumper();
    private final Trigger SNAP_NORTH = driverController.pov(0);
    private final Trigger SNAP_SOUTH = driverController.pov(180);
    private final Trigger SNAP_EAST = driverController.pov(90);
    private final Trigger SNAP_WEST = driverController.pov(270);
    

    //Operator controls
    private final Trigger PRIME_L1 = operatorController.button(1);
    private final Trigger PRIME_L2 = operatorController.button(2);
    private final Trigger PRIME_L3 = operatorController.button(3);
    private final Trigger PRIME_L4 = operatorController.button(4);
    private final Trigger PRIME_L2_ALGAE = operatorController.button(5);
    private final Trigger PRIME_L3_ALGAE = operatorController.button(6);
    private final Trigger PRIME_NET = operatorController.button(7);
    private final Trigger PRIME_PROCESSOR = operatorController.button(8);
    private final Trigger GROUND_INTAKE = operatorController.button(9);

    


    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve();
        vision = new Vision();
        intake = new Intake();
        elevator = new Elevator();
        clockArm = new ClockArm();
        wrist = new Wrist();
        // TODO: Add more subsystems and instantiatable helpers here

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
        NewtonCommands.addSubsystems(swerve, elevator, clockArm, wrist, intake);
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

        INTAKE.whileTrue(intakeCommand());
        SCORE.whileTrue(outtakeCommand());

        PRIME_L1.onTrue(NewtonCommands.primeL1Command());
        PRIME_L2.onTrue(NewtonCommands.primeL2Command());
        PRIME_L3.onTrue(NewtonCommands.primeL3Command());
        PRIME_L4.onTrue(NewtonCommands.primeL4Command());
        PRIME_L2_ALGAE.onTrue(NewtonCommands.primeL2AlgaeCommand());
        PRIME_L3_ALGAE.onTrue(NewtonCommands.primeL3AlgaeCommand());
        PRIME_NET.onTrue(NewtonCommands.primeNetCommand());
        PRIME_PROCESSOR.onTrue(NewtonCommands.primeProcessorCommand());
        GROUND_INTAKE.onTrue(NewtonCommands.groundIntakeCommand());
        STOW.onTrue(NewtonCommands.stowCommand());
        PRIME.onTrue(NewtonCommands.goToPrimePositionCommand());
    }



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
