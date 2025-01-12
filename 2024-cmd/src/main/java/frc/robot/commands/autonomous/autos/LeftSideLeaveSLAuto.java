package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class LeftSideLeaveSLAuto extends AutoCommand{
    public LeftSideLeaveSLAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("LeftSideLeaveSL"), Suppliers.robotRunningOnRed)
        );
    }
}
