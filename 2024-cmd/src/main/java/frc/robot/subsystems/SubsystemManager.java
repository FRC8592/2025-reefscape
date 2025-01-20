package frc.robot.subsystems;

import java.util.*;

import lib.team8592.MatchMode;

public class SubsystemManager {
    private List<NewtonSubsystem> subsystems = new ArrayList<>();

    public final SwerveSubsystem swerve;
    public final ElevatorSubsystem elevator;
    public final ClockSubsystem clock;
    public final WristSubsystem wrist;
    public final VisionSubsystem vision;
    public final ClimbSubsystem climb;
    public final RollerSubsystem rollers;

    public SubsystemManager(boolean logToShuffleboard){
        this.swerve = new SwerveSubsystem(logToShuffleboard);
        this.elevator = new ElevatorSubsystem(logToShuffleboard);
        this.clock = new ClockSubsystem(logToShuffleboard);
        this.wrist = new WristSubsystem(logToShuffleboard);
        this.vision = new VisionSubsystem(logToShuffleboard);
        this.climb = new ClimbSubsystem(logToShuffleboard);
        this.rollers = new RollerSubsystem(logToShuffleboard);

        this.subsystems = List.of(
            swerve, 
            elevator, 
            clock, 
            wrist, 
            vision, 
            climb, 
            rollers
        );
    }

    public void onModeInit(MatchMode mode) {
        this.subsystems.forEach(subsystem -> subsystem.onInit(mode));
    }
}