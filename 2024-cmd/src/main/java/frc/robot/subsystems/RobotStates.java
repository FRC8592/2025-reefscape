package frc.robot.subsystems;

public enum RobotStates {
    L1_CORAL(0, 0, 0),
    L2_CORAL(0, 0, 0),
    L3_CORAL(0, 0, 0),
    L4_CORAL(0, 0, 0),

    L2_ALGAE(0, 0, 0),
    L3_ALGAE(0, 0, 0),

    NET(0, 0, 0),

    PROCESSOR(0, 0, 0),

    HP_INTAKE(0, 0, 0),
    GROUND_INTAKE(0, 0, 0),

    STOW(0, 0, 0),
    ;

    public final double elevatorInches, clockDegrees, wristDegrees;

    private RobotStates(double elevator, double clock, double wrist) {
        this.elevatorInches = elevator;
        this.clockDegrees = clock;
        this.wristDegrees = wrist;
    }
}