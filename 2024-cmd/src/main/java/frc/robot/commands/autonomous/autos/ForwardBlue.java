package frc.robot.commands.autonomous.autos;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;



public class ForwardBlue extends AutoCommand{
    public ForwardBlue(){
        super(
        new FollowPathCommand(getChoreoTrajectory("MiddleToFrontBlue"), Suppliers.isRedAlliance, "")


        );
    }
}
