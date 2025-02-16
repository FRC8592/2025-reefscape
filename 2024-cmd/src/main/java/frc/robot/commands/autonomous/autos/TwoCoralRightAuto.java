package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class TwoCoralRightAuto extends AutoCommand{
    public TwoCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToNELeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("NELeftToHPRight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPRightToSERight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("SERightToHPRight"), Suppliers.robotRunningOnRed)
        );

        setStartStateFromChoreoTrajectory("RightToNELeft");
    }
}
