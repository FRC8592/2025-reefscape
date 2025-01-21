// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.NewtonCommands;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SwerveSubsystem.DriveModes;

import lib.team8592.MatchMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

import static frc.robot.commands.NewtonCommands.*;

public class RobotContainer {
    // The robot's subsystems
    private final SubsystemManager manager;

    // private final SwerveSubsystem swerve;
    // private final RollerSubsystem rollers;
    // private final ElevatorSubsystem elevator;
    // private final ClockSubsystem clock;
    // private final WristSubsystem wrist;
    // private final VisionSubsystem vision;

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer(boolean logToShuffleboard) {
        this.manager = new SubsystemManager(logToShuffleboard);
        // this.swerve = manager.swerve;
        // this.rollers = manager.rollers;
        // this.elevator = manager.elevator;
        // this.clock = manager.clock;
        // this.wrist = manager.wrist;
        // this.vision = manager.vision;

        passSubsystems();
        configureBindings();
        configureDefaults();

        AutoManager.prepare();
    }

    /**
     * Pass subsystems everywhere they're needed
     */
    private void passSubsystems(){
        AutoManager.addSubsystems(manager);
        AutoCommand.addSubsystems(manager);
        LargeCommand.addSubsystems(manager);
        NewtonCommands.addSubsystems(manager);
        Suppliers.addSubsystems(manager);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        manager.swerve.setDefaultCommand(manager.swerve.run(() -> {
            manager.swerve.drive(manager.swerve.processJoystickInputs(
                -Controls.getDriver().getLeftX(),
                -Controls.getDriver().getLeftY(),
                -Controls.getDriver().getRightX()
            ), DriveModes.AUTOMATIC);
        }));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // Driver controls:
        // Operator:
        Controls.slowMode.onTrue(
            // The Commands.runOnce (instead of swerve.runOnce) is a special case here
            // to allow this to run while other swerve commands (the default driving
            // command, for example) run. This is usually a horrible idea and shouldn't
            // be used outside of special cases like this.

            // The .ignoringDisable makes sure slow mode won't get stuck on or off if
            // the robot is disabled.
            Commands.runOnce(() -> manager.swerve.setSlowMode(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerve.setSlowMode(false)).ignoringDisable(true)
        );

        Controls.zeroGryoscope.onTrue(
            // Similar comment on Commands.runOnce as slow mode above
            Commands.runOnce(() -> manager.swerve.resetHeading())
        );

        Controls.robotRelative.onTrue(
            // Similar comment on Commands.runOnce and ignoringDisable as slow mode above
            Commands.runOnce(() -> manager.swerve.setRobotRelative(true)).ignoringDisable(true)
        ).onFalse(
            Commands.runOnce(() -> manager.swerve.setRobotRelative(false)).ignoringDisable(true)
        );

        Controls.snapNorth.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(0),
                () -> -Controls.getDriver().getLeftX(),
                () -> -Controls.getDriver().getLeftY()
            )
        );

        Controls.snapSouth.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(180),
                () -> -Controls.getDriver().getLeftX(),
                () -> -Controls.getDriver().getLeftY()
            )
        );

        Controls.snapEast.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(270),
                () -> -Controls.getDriver().getLeftX(),
                () -> -Controls.getDriver().getLeftY()
            )
        );

        Controls.snapWest.whileTrue(
            swerveSnapToCommand(
                Rotation2d.fromDegrees(90),
                () -> -Controls.getDriver().getLeftX(),
                () -> -Controls.getDriver().getLeftY()
            ).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        Controls.intakeCoral.whileTrue(intakeCommand());
        Controls.intakeAlgae.onTrue(NewtonCommands.groundIntakeCommand());
        Controls.scorePiece.whileTrue(outtakeCommand());

        Controls.stow.onTrue(NewtonCommands.stowCommand());
        Controls.goToPrimedPosition.onTrue(NewtonCommands.goToPrimePositionCommand());

        Controls.primeL1.onTrue(NewtonCommands.primeL1Command());
        Controls.primeL2.onTrue(NewtonCommands.primeL2Command());
        Controls.primeL3.onTrue(NewtonCommands.primeL3Command());
        Controls.primeL4.onTrue(NewtonCommands.primeL4Command());
        Controls.primeAlgaeL2.onTrue(NewtonCommands.primeL2AlgaeCommand());
        Controls.primeAlgaeL3.onTrue(NewtonCommands.primeL3AlgaeCommand());
        Controls.primeNet.onTrue(NewtonCommands.primeNetCommand());
        Controls.primeProcessor.onTrue(NewtonCommands.primeProcessorCommand());

        Controls.alignToLeftBranch.whileTrue(
            NewtonCommands.driveToReefCommand(
                Constants.CORAL_ALIGN.LEFT_OFFSET, 
                Controls.driveTranslateX,
                Controls.driveTranslateY, 
                Controls.driveRotate
            )
        );

        Controls.alignToRightBranch.whileTrue(
            NewtonCommands.driveToReefCommand(
                Constants.CORAL_ALIGN.RIGHT_OFFSET, 
                Controls.driveTranslateX,
                Controls.driveTranslateY, 
                Controls.driveRotate
            )
        );
    }

    public void onModeInit(MatchMode mode){
        CommandScheduler.getInstance().schedule(Commands.runOnce(() -> manager.onModeInit(mode), new SubsystemBase[0]));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
    }
}
