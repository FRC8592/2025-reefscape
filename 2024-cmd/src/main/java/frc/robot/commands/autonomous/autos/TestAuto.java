package frc.robot.commands.autonomous.autos;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;



public class TestAuto extends AutoCommand{
    public TestAuto(){
        super(
        new FollowPathCommand(getChoreoTrajectory("HPLeftToFL`eft"), Suppliers.robotRunningOnRed)


        );
        setStartStateFromChoreoTrajectory("HPLeftToFLeft");
    }
}
