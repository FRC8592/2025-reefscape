package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.SIMULATION.*;

public class Superstructure extends SubsystemBase {
    private Mechanism2d superstructureMech;
    private MechanismRoot2d mechRoot;
    private MechanismLigament2d elevatorLigament;
    private MechanismLigament2d clockLigament;
    private MechanismLigament2d wristLigament;

    private ElevatorSim elevatorSim;
    private SingleJointedArmSim clockSim;
    private SingleJointedArmSim wristSim;

    private double lastTime = 0d;

    private ProfiledPIDController elevatorCtrl = new ProfiledPIDController(
        1000d, 
        0, 
        0, 
        new Constraints(24, 24)
    );

    private ProfiledPIDController clockCtrl = new ProfiledPIDController(
        100d, 
        0, 
        0, 
        new Constraints(12, 24)
    );

    private ProfiledPIDController wristCtrl = new ProfiledPIDController(
        100d, 
        0, 
        0, 
        new Constraints(12, 24)
    );

    public enum Positions {
        STOW(31.25, 180d, 0),
        L1(31.25, 60d, 90),
        L2(38d, 60d, 90),
        L3(41d, 60d, 90),
        L4(48d, 75d, 135),
        ;

        public final double elevatorInches, clockDegrees, wristDegrees;
        private Positions(double elevatorInches, double clockDegrees, double wristDegrees) {
            this.elevatorInches = elevatorInches;
            this.clockDegrees = clockDegrees;
            this.wristDegrees = wristDegrees;
        }
    }

    public Superstructure() {
        this.superstructureMech = new Mechanism2d(SUPERSTRUCTURE_WIDTH_METERS, SUPERSTRUCTURE_HEIGHT_METERS);
        this.mechRoot = this.superstructureMech.getRoot("Root", ELEVATOR_OFFSET_X, BASE_ATTACHMENT_Y);
        this.elevatorLigament = this.mechRoot.append(
            new MechanismLigament2d(
                "Elevator", 
                Units.inchesToMeters(31.25), 
                90
            )
        );

        this.clockLigament = this.elevatorLigament.append(
            new MechanismLigament2d(
                "Clock", 
                CLOCK_LENGTH_METERS, 
                180d
            )
        );

        this.wristLigament = this.clockLigament.append(
            new MechanismLigament2d(
                "Wrist", 
                WRIST_LENGTH_METERS, 
                0d
            )
        );

        this.elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1), 
            Constants.ELEVATOR.OVERALL_GEAR_RATIO, 
            FULL_ELEVATOR_SYSTEM_MASS,
            ELEVATOR_DRUM_RADIUS_METERS,
            Units.inchesToMeters(31.25), 
            Units.inchesToMeters(48.907), 
            false, 
            Units.inchesToMeters(31.25)
        );

        this.clockSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            100d, 
            0.504, 
            CLOCK_LENGTH_METERS, 
            -Math.PI, 
            Math.PI, 
            false, 
            Math.PI
        );

        this.wristSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1), 
            100d, 
            0.504, 
            WRIST_LENGTH_METERS,
            -Math.PI,
            Math.PI,
            false,
            0d
        );

        clockCtrl.enableContinuousInput(-Math.PI, Math.PI);
        wristCtrl.enableContinuousInput(-Math.PI, Math.PI);

        SmartDashboard.putData("Superstructure", superstructureMech);
    }

    public void setPosition(Positions position) {
        setElevatorPosition(position.elevatorInches);
        setClockAngle(position.clockDegrees);
        setWristAngle(position.wristDegrees);
    }

    private void setElevatorPosition(double inches) {
        elevatorSim.setInput(elevatorCtrl.calculate(elevatorSim.getPositionMeters(), Units.inchesToMeters(inches)));
        elevatorLigament.setLength(elevatorSim.getPositionMeters());
    }

    private void setClockAngle(double degrees) {
        clockSim.setInput(clockCtrl.calculate(clockSim.getAngleRads(), degrees*Math.PI/180));
        clockLigament.setAngle(clockSim.getAngleRads()*180/Math.PI);
    }

    private void setWristAngle(double degrees) {
        wristSim.setInput(wristCtrl.calculate(wristSim.getAngleRads(), degrees*Math.PI/180));
        wristLigament.setAngle(wristSim.getAngleRads()*180/Math.PI);
    }

    @Override
    public void simulationPeriodic() {
        // Update the simulation of the elevator
        elevatorSim.update(Timer.getFPGATimestamp() - lastTime);
        clockSim.update(Timer.getFPGATimestamp() - lastTime);
        wristSim.update(Timer.getFPGATimestamp() - lastTime);

        lastTime = Timer.getFPGATimestamp();
        
        SmartDashboard.putNumber("Elevator Length", elevatorSim.getPositionMeters());
        SmartDashboard.putNumber("Clock Angle", clockSim.getAngleRads());
        SmartDashboard.putNumber("Wrist Angle", wristSim.getAngleRads());
    }
}