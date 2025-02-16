package frc.robot.commands.autonomous.autos;

import frc.robot.Robot;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;
import frc.robot.commands.proxies.TimingSimulatedCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Scoring.ElevatorPositions;

public class MiddlePreloadMobilityAuto extends AutoCommand{
    public MiddlePreloadMobilityAuto(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MiddleToNorthLeft"), Suppliers.robotRunningOnRed)
            //.alongWith(new TimingSimulatedCommand(scoring.goToSpecifiedPosition(ElevatorPositions.L4)))
            // .andThen(intake.setIntakeCommand(-0.5).withTimeout(0.5)),
            ,new FollowPathCommand(getChoreoTrajectory("FaceDToBarge"), Suppliers.robotRunningOnRed)
           // .andThen(new TimingSimulatedCommand(scoring.stowCommand()))
        );
        setStartStateFromChoreoTrajectory("MiddleToNorthLeft");
    }
}
