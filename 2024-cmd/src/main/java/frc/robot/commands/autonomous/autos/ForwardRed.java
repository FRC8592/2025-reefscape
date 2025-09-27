package frc.robot.commands.autonomous.autos;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;



public class ForwardRed extends AutoCommand{
    public ForwardRed(){
        super(
        new FollowPathCommand(getChoreoTrajectory("MiddleToFrontRed"), Suppliers.isRedAlliance, "")


        );
    }
}
