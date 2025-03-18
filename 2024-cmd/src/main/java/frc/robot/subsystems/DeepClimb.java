package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;

public class DeepClimb extends SubsystemBase {

    NewtonMotor deepClimbMotor;
    NewtonMotor deepClimbIntakeMotor;

    public DeepClimb() {
        deepClimbMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_MOTOR_CAN_ID, true); 
        deepClimbMotor.setIdleMode(IdleMode.kBrake);

        deepClimbIntakeMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_INTAKE_MOTOR_CAN_ID, true);
        deepClimbIntakeMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setDeepClimbPercentOutput(double percent){
        deepClimbMotor.setPercentOutput(percent);
    }

    public void setDeepClimbIntakePercentOutput(double percent){
        deepClimbIntakeMotor.setPercentOutput(percent);
    }

    // public boolean isDeepClimbDeployed() {
    //     return deepClimbMotor.getRotations() > DEEP_CLIMB.DEEP_CLIMB_START_POSITION;
    // }

    public Command setDeepClimbGrabPositionCommand(){
        return this.run(()->deepClimbMotor.setPercentOutput(-1)).until(
            () -> deepClimbMotor.getRotations() <= DEEP_CLIMB.DEEP_CLIMB_GRAB_POSITION
        ).finallyDo(() -> deepClimbMotor.setPercentOutput(0));
    }


    public void stopDeepClimb() {
        setDeepClimbPercentOutput(0);
    }

    public void stopDeepIntakeClimb() {
        setDeepClimbIntakePercentOutput(0);
    }

    public Command setDeepClimbCommand(double percent){
        return this.run(()->{setDeepClimbPercentOutput(percent);}) 
        .until(()->deepClimbMotor.getRotations()>=DEEP_CLIMB.DEEP_CLIMB_MAX_POSITION);
    }

    public Command setDeepClimbIntakeCommand(double percent){
        return this.run(()->{setDeepClimbIntakePercentOutput(percent);});
    }

    public Command stopDeepClimbCommand(){
        return this.runOnce(()->{
            setDeepClimbPercentOutput(0);
        });
    }

    public Command stopDeepClimbIntakeCommand(){
        return this.runOnce(()->{
            setDeepClimbIntakePercentOutput(0);
        });
    }

    public void periodic() {
        Logger.recordOutput(SHARED.LOG_FOLDER + "Winch Rotations", deepClimbMotor.getRotations());

    }
}
