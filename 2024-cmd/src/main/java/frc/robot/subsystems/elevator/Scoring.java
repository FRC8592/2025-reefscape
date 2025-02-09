package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.commands.proxies.WaitUntilCommand;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Scoring {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;

    private ElevatorPositions targetPosition;

    public enum ElevatorPositions {
        L1(15, 30, 90),
        L2(15, 30, 90),
        L3(12, 135, 90),
        L4(18, 135, 90),
        GROUND_ALGAE(0, 0, 0),
        STARTING(0, 0, 0),
        STOW(0, 10, 45),
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

    public Scoring(){
        
        this.elevator = new Elevator();
        this.clockArm = new ClockArm();
        this.wrist = new Wrist();

        targetPosition = ElevatorPositions.STARTING;

    }

    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
            targetPosition = eposition;
        }, new Subsystem[0]);
    }

    public Command goToL4Command(){
        return clockArm.setArmPositionCommand(135)
        .alongWith(elevator.setExtensionCommand(18), new WaitUntilCommand(wrist.setWristCommand(90), ()->clockArm.getArmPositionDegrees()>30));
    }

    public Command goToPosition(){
        return clockArm.setArmPositionCommand(30)
        .alongWith(elevator.setExtensionCommand(targetPosition.elevatorPos))
        .andThen(wrist.setWristCommand(targetPosition.wristPos),
         clockArm.setArmPositionCommand(targetPosition.clockArmPos));
    }

    // public Command starting(){
    //     return clockArm.setArmPositionCommand(30)
    //     .alongWith(elevator.setExtensionCommand(0)).andThen(wrist.setWristCommand(0), clockArm.setArmPositionCommand(0));
    // }

    public Command stopAll(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWrist(), clockArm.stopArmCommand());
    }

}
