package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class ThreeCoralLeftAuto extends AutoCommand{
    public ThreeCoralLeftAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftToERight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("ERightToHPLeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFLeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("FLeftToHPLeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPLeftToFRight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("FRightBackUp"), Suppliers.robotRunningOnRed)
        );

        setStartStateFromChoreoTrajectory("LeftToERight");
    }
}
