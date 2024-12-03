package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;

public class PreloadBucket2Auto extends AutoCommand {
    
    public PreloadBucket2Auto() {
        super(
            new FollowPathCommand(getChoreoTrajectory("PreloadBucket2Path"), Suppliers.robotRunningOnRed)
            .andThen(Commands.waitSeconds(1))
            .andThen(
                new FollowPathCommand(getChoreoTrajectory("PreloadBucket2Path2"), Suppliers.robotRunningOnRed)
            )
        );
        setStartStateFromChoreoTrajectory("PreloadBucket2Path");
    }

}
