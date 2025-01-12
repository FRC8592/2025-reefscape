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
    // private final Swerve swerve;
    private final Intake intake;
    //TODO: Add more subsystems here

    // Helpers
    // TODO: Add instantiatable helpers here

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        // swerve = new Swerve();
        intake = new Intake();
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
        // AutoManager.addSubsystems(swerve);
        // AutoCommand.addSubsystems(swerve);
        // LargeCommand.addSubsystems(swerve);
        NewtonCommands.addSubsystems(intake);

        // Suppliers.addSubsystems(swerve);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        setDefaultCommand(intake, intake.run(() -> {
            intake.stop();
        }));
        // Set the swerve's default command to drive with joysticks
        // setDefaultCommand(swerve, swerve.run(() -> {
        //     swerve.drive(swerve.processJoystickInputs(
        //         -driverController.getLeftX(),
        //         -driverController.getLeftY(),
        //         -driverController.getRightX()
        //     ), DriveModes.AUTOMATIC);
        // }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }


    //Any commands that are reused a lot but can't go in a separate class go here

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        driverController.a().whileTrue(intakeCommand());
        driverController.x().whileTrue(outtakeCommand());
        
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
