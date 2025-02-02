package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.NewtonCommands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.Swerve;

public class ElevatorCommands {

    private static Elevator elevator;

    
    public enum ElevatorPositions {

        L1(0),
        L2(0),
        L3(0),
        L4(0),
        GROUND_ALGAE(0),
        HP_INTAKE(0),
        STOW(0),
        L2_ALGAE(0),
        L3_ALGAE(0),
        PROCESSOR(0),
        NET(0);

        public double elevatorPos = 0;
        public double wristPos = 0;
        public double clockArmPos = 0;
        
        private  ElevatorPositions(double elevator) {

          elevatorPos = elevator;


        }
    }

    public static void addSubsystems(Elevator elevator){
        ElevatorCommands.elevator = elevator;
    }



    public static Command setElevatorPosCommand(ElevatorPositions targetPosition){
        elevator.setExtensionCommand(targetPosition.elevatorPos);
        return Commands.none();
    }

}
