package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.proxies.WaitUntilCommand;

public class Scoring extends SubsystemBase {

    private Elevator elevator;
    private ClockArm clockArm;
    private Wrist wrist;
    private Intake intake;

    private static ElevatorPositions targetPosition;

    public static enum ElevatorPositions {
        L1(11.8, 0, 144),
        // elevator front: , back: , arm: , wrist: 
        L2(14.4, 12, 133),
        // front: , back: , arm: , wrist: 
        L3(0, 172, 155),
        // front: , back: , arm: , wrist: 
        L4(19.5, 175, 135),
        // front: , back: , arm: , wrist: 
        GROUND_ALGAE(0, 0, 0),
        STARTING(0, 0, 0),
        STOW(0, 0, 0), //0, 10, -45
        // elevator front: 0.117, back: -0.165, arm: -0.4277, wrist: -0.2622
        L2_ALGAE(14, 30, 120),
        L3_ALGAE(3, 150, 120),
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

    public Scoring(Elevator elevator, ClockArm arm, Wrist wrist, Intake intake){
        this.elevator = elevator;
        this.clockArm = arm;
        this.wrist = wrist;
        this.intake = intake;

        targetPosition = ElevatorPositions.STARTING;

    }

    public Command setPosition(ElevatorPositions eposition){
        return Commands.runOnce(()->{
            Logger.recordOutput(Constants.SHARED.LOG_FOLDER + "/targetPosition", eposition.toString());
            SmartDashboard.putString("targetPosition", eposition.toString());
            targetPosition = eposition;
        }, new Subsystem[0]);
    }

    public Command goToPosition(){
        return elevator.setExtensionPositionCommand(()->targetPosition.elevatorPos)
        .alongWith(wrist.setWristPositionCommand(()->targetPosition.wristPos))
        .alongWith(clockArm.setArmPositionCommand(()->targetPosition.clockArmPos));
    }

    public Command goToSpecifiedPosition(ElevatorPositions eposition){
        return setPosition(targetPosition).andThen(goToPosition());
    }

    public Command stowCommand(){
        return clockArm.setArmPositionCommand(()->45).onlyIf(()-> clockArm.getTargetArmPositionDegrees() != ElevatorPositions.STOW.clockArmPos).until(()->clockArm.atPosition()) 
        .andThen(goToSpecifiedPosition(ElevatorPositions.STOW));
    }
    

    public Command intakeCommand(){
        return intake.setIntakeCommand(0.5).until(() -> intake.robotHasCoral());
    }

    public Command outtakeCommand(){
        return intake.setIntakeCommand(-0.5).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command stopAllCommand(){
        return elevator.stopElevatorCommand().alongWith(wrist.stopWristCommand(), clockArm.stopArmCommand());
    }

}
