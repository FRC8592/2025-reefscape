package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class FourCoralRightAuto extends AutoCommand{
    public FourCoralRightAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("RightToNELeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("NELeftToHPRight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPRightToSERight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("SERightToHPRight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPRightToSELeft"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("SELeftToHPRight"), Suppliers.robotRunningOnRed),
            new WaitCommand(0.5),
            new FollowPathCommand(getChoreoTrajectory("HPRightToSRight"), Suppliers.robotRunningOnRed),
            new FollowPathCommand(getChoreoTrajectory("SRightBackUp"), Suppliers.robotRunningOnRed)
        );

        setStartStateFromChoreoTrajectory("RightToNELeft");
    }
}
