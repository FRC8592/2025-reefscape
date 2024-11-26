package frc.robot.commands.autonomous;

import frc.robot.Suppliers;
import frc.robot.subsystems.swerve.Swerve;

public class TestAuto extends AutoCommand {
    
    public TestAuto() {

        super(
        
            swerve.commands.followPathCommand(getChoreoTrajectory("testPath"), Suppliers.robotRunningOnRed)

        );

        setStartStateFromChoreoTrajectory("testPath");

    }

}
