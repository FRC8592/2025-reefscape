package frc.robot.commands.autonomous.autos;

import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class TestAuto extends AutoCommand {
    public TestAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("testPath"), () -> false)
        );
    }
}
