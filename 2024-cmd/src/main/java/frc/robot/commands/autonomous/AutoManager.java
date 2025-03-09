// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.autos.*;
import frc.robot.commands.autonomous.autos.OmniCoralRedAuto.RedOrBlue;
import frc.robot.commands.proxies.*;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.scoring.Scoring;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static Swerve swerve;
    private static Scoring scoring;
    private static LEDs leds;
    
    public static void addSubsystems(Swerve swerve, Scoring scoring, LEDs leds){
        AutoManager.swerve = swerve;
        AutoManager.scoring = scoring;
        AutoManager.leds = leds;
    }
    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(){
        autoCommands = new ArrayList<>();


        // autoCommands.add(new ExampleAuto());
        // TODO: Add autos here

        autoCommands.add(new OmniCoralRedAuto(1, RedOrBlue.RED).withAutoName("OneCoralRedAuto"));
        autoCommands.add(new OmniCoralRedAuto(2, RedOrBlue.RED).withAutoName("TwoCoralRedAuto"));
        autoCommands.add(new OmniCoralRedAuto(3, RedOrBlue.RED).withAutoName("ThreeCoralRedAuto"));
        autoCommands.add(new OmniCoralRedAuto(4, RedOrBlue.RED).withAutoName("FourCoralRedAuto"));

        autoCommands.add(new OmniCoralRedAuto(1, RedOrBlue.BLUE).withAutoName("OneCoralBlueAuto"));
        autoCommands.add(new OmniCoralRedAuto(2, RedOrBlue.BLUE).withAutoName("TwoCoralBlueAuto"));
        autoCommands.add(new OmniCoralRedAuto(3, RedOrBlue.BLUE).withAutoName("ThreeCoralBlueAuto"));
        autoCommands.add(new OmniCoralRedAuto(4, RedOrBlue.BLUE).withAutoName("FourCoralBlueAuto"));

        // autoCommands.add(new FourCoralBlueAuto());
        // autoCommands.add(new OneCoralBlueAuto());
        // //autoCommands.add(new OneCoralOneAlgaeMiddleAuto());
        // autoCommands.add(new ThreeCoralBlueAuto());
        // autoCommands.add(new TwoCoralBlueAuto());
        autoCommands.add(new TestAuto());


        autoChooser = new SendableChooser<>();
        
        autoChooser.setDefaultOption("DEFAULT - No auto", new AutoCommand());
        for(AutoCommand c : autoCommands){
            autoChooser.addOption(
                c.getAutoName(), c
            );
        }
        Shuffleboard.getTab("Autonomous Config").add(autoChooser);
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();
        return getAutonomousInitCommand().andThen(
            // If we don't keep this command from registering as composed,
            // the code will crash if we try to run an auto twice without
            // restarting robot code.
            new MultiComposableCommand(autoCommand)
        );
    }

    /**
     * Parallel command group that runs all subsystems' autonomous init commands.
     *
     * @return the command
     */
    private static Command getAutonomousInitCommand(){
        return new ParallelCommandGroup(
            swerve.runOnce(() -> {
                swerve.stop();
                swerve.resetHeading();
            })
            // TODO: Add any other commands that need to be run on autonomous init here
        );
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
