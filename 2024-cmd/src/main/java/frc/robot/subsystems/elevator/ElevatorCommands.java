package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.NewtonCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class ElevatorCommands {

    private static Elevator elevator;
    private static ClockArm clockArm;
    private static Wrist wrist;
    
    public enum ElevatorPositions {

        L1(0, 0, 0),
        L2(0, 0, 0),
        L3(0, 0, 0),
        L4(0, 0, 0),
        GROUND_ALGAE(0, 0, 0),
        HP_INTAKE(0, 0, 0),
        STOW(0, 0, 0),
        L2_ALGAE(0, 0, 0),
        L3_ALGAE(0, 0, 0),
        PROCESSOR(0, 0, 0),
        NET(0, 0, 0);

        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private  ElevatorPositions(double elevator, double clockArm, double wrist) {

          elevatorPos = elevator;
          wristPos = wrist;
          clockArmPos =  clockArm;

        }
    }

    public static void addSubsystems(Elevator elevator, ClockArm clockArm, Wrist wrist){
        ElevatorCommands.elevator = elevator;
        ElevatorCommands.clockArm = clockArm;
        ElevatorCommands.wrist = wrist;
    }

    public static double getWristAbsoluteDegrees() {
        return clockArm.getArmPositionDegrees() + wrist.getWristDegrees();
    }



    public static Command setElevatorPosCommand(ElevatorPositions targetPosition){
        clockArm.setArmPositionCommand(targetPosition.clockArmPos);
        elevator.setExtensionCommand(targetPosition.elevatorPos);
        wrist.setWristCommand(targetPosition.wristPos);
        return Commands.none();
    }

}
