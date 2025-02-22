package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class AllAlgaeAuto extends AutoCommand{
    public AllAlgaeAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RemoveAlgaeC"), Suppliers.robotRunningOnRed),
            new FollowPathCommand(getChoreoTrajectory("RemoveAlgaeB"), Suppliers.robotRunningOnRed),
            new FollowPathCommand(getChoreoTrajectory("RemoveAlgaeA"), Suppliers.robotRunningOnRed)
        );

        setStartStateFromChoreoTrajectory("RemoveAlgaeC");

    }
}
