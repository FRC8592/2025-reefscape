// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Controls.ControlSets;
import frc.robot.commands.autonomous.*;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.DriveModes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;

public class RobotContainer {
    // The robot's subsystems
    private final Swerve swerve;
    //TODO: Add more subsystems here

    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = Swerve.instantiate();
        // TODO: Add more subsystems and instantiatable helpers here

        configureBindings(ControlSets.MAIN_TELEOP);
        configureDefaults();


        AutoManager.prepare();
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.commands.driveCommand(
            Controls.driveTranslateX, Controls.driveTranslateY, Controls.driveRotate, DriveModes.AUTOMATIC
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        //TODO: Set more subsystems' default commands here
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     *
     * @param controlSet the set of controls to use
     */
    private void configureBindings(ControlSets controlSet) {
        CommandScheduler.getInstance().getDefaultButtonLoop().clear();
        Controls.applyControlSet(controlSet);

        Controls.slowMode.onTrue(
            swerve.commands.slowModeCommand(true) // Enable slow mode
        ).onFalse(
            swerve.commands.slowModeCommand(false) // Disable slow mode
        );

        Controls.zeroGryoscope.onTrue(
            swerve.commands.resetHeadingCommand()
        );

        Controls.robotRelative.onTrue(
            swerve.commands.robotRelativeCommand(true) // Enable robot-oriented driving
        ).onFalse(
            swerve.commands.robotRelativeCommand(false) // Disable robot-oriented driving
        );

        Controls.snapForward.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(0), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapBack.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(180), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapLeft.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(270), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );
        Controls.snapRight.whileTrue(
            swerve.commands.snapToCommand(Controls.driveTranslateX, Controls.driveTranslateY, Rotation2d.fromDegrees(90), DriveModes.AUTOMATIC)
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        );

        // TODO: Add more bindings from controls to commands here
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
