// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import static frc.robot.commands.NewtonCommands.*;

import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.commands.proxies.OverrideEverythingCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Pivot.Positions;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    private static final CommandXboxController operatorController = new CommandXboxController(
        CONTROLLERS.OPERATOR_PORT
    );

    // The robot's subsystems
    private final Swerve swerve;
    private final Intake intake;
    private final Pivot pivot;
    //TODO: Add more subsystems here

    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve();
        intake = new Intake();
        pivot = new Pivot();
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
        AutoManager.addSubsystems(swerve, intake, pivot);
        AutoCommand.addSubsystems(swerve, intake, pivot);
        LargeCommand.addSubsystems(swerve, intake, pivot);
        NewtonCommands.addSubsystems(swerve, intake, pivot);
        Suppliers.addSubsystems(swerve, intake, pivot);
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

        //setting the intakes default command
        setDefaultCommand(intake, intake.run(
            () -> {
                intake.runTopMotor(INTAKE.TOP_MOTOR_DEFAULT_SPEED);
                intake.runBottomMotor(INTAKE.BOTTOM_MOTOR_DEFAULT_SPEED);
            }
        ));
        //setting the pivot default
        setDefaultCommand(pivot, pivot.run(
            () -> pivot.setMotorPower(
                -operatorController.getLeftY() * PIVOT.PIVOT_MANUAL_CONTROL_MAX_SPEED
            )
        ));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        
        // Driver controls:
        // Translate is left stick
        // Rotate is right stick
        // Slow mode is right bumper
        // Rezero is back
        // Robot oriented is left bumper
        // Snap to is d pad
        
        // Operator: 
        // Intake is left trigger
        // Outtake is left bumper
        // Score is right trigger
        // Stow is b button
        // Score Grid position is x button
        // Ground position is "a" button
        // High score position is y button
        driverController.rightBumper().onTrue(
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

        driverController.back().onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> swerve.resetHeading())
        );

        driverController.leftBumper().onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        driverController.pov(0).whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        driverController.pov(180).whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        driverController.pov(90).whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            )
        );

        driverController.pov(270).whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                () -> -driverController.getLeftX(),
                () -> -driverController.getLeftY()
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        operatorController.leftTrigger(0.1).whileTrue(
            runIntakeCommand(INTAKE.TOP_MOTOR_INTAKE_SPEED, INTAKE.BOTTOM_MOTOR_INTAKE_SPEED)
        );

        operatorController.rightTrigger(0.1).whileTrue(
            runIntakeCommand(INTAKE.TOP_MOTOR_SCORE_SPEED, INTAKE.BOTTOM_MOTOR_SCORE_SPEED)
        );

        operatorController.leftBumper().whileTrue(
            runIntakeCommand(
                INTAKE.TOP_MOTOR_OUTTAKE_SPEED, INTAKE.BOTTOM_MOTOR_OUTTAKE_SPEED
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        
        operatorController.b().onTrue(
            setPivotPositionCommand(Positions.REST).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        operatorController.x().onTrue(
            setPivotPositionCommand(Positions.SCORE_GRID).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        operatorController.a().onTrue(
            setPivotPositionCommand(Positions.GROUND).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

        operatorController.y().onTrue(
            setPivotPositionCommand(Positions.SCORE_HIGH).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        );

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
