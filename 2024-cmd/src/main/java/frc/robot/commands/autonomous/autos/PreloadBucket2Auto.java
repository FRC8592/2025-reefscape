package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;

public class PreloadBucket2Auto extends AutoCommand {
    
    public PreloadBucket2Auto() {
        super(
            
            swerve.commands.followPathCommand(getChoreoTrajectory("PreloadBucket2Path"), Suppliers.robotRunningOnRed)
            .andThen(Commands.waitSeconds(1))
            .andThen(
                swerve.commands.followPathCommand(getChoreoTrajectory("PreloadBucket2Path2"), Suppliers.robotRunningOnRed)
            )

        );
        setStartStateFromChoreoTrajectory("PreloadBucket2Path");
    }

}
