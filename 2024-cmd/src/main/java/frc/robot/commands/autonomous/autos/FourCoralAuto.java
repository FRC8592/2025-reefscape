package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class FourCoralAuto extends AutoCommand{
    public FourCoralAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("SLToBranch2"), Suppliers.robotRunningOnRed),
            new WaitCommand(1),
            new FollowPathCommand(getChoreoTrajectory("Branch2ToLeftHP"), Suppliers.robotRunningOnRed),
            new WaitCommand(.5),
            new FollowPathCommand(getChoreoTrajectory("LeftHPToBranch12"), Suppliers.robotRunningOnRed),
            new WaitCommand(1),
            new FollowPathCommand(getChoreoTrajectory("Branch12ToLeftHP"), Suppliers.robotRunningOnRed),
            new WaitCommand(.5),
            new FollowPathCommand(getChoreoTrajectory("LeftHPToBranch11"), Suppliers.robotRunningOnRed),
            new WaitCommand(1),
            new FollowPathCommand(getChoreoTrajectory("Branch11ToLeftHP"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("LeftHPToBranch10"), Suppliers.robotRunningOnRed)
        );
    }
}
