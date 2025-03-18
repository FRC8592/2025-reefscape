package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkFlexMotor;

public class DeepClimb extends SubsystemBase {

    NewtonMotor deepClimbMotor;
    NewtonMotor deepClimbIntakeMotor;
    double motorRotations = 0;
    boolean motorRotationsSet = false;


    public DeepClimb() {
        deepClimbMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_MOTOR_CAN_ID, true); 
        deepClimbMotor.setIdleMode(IdleMode.kBrake);

        deepClimbIntakeMotor = new SparkFlexMotor(CAN.DEEP_CLIMB_INTAKE_MOTOR_CAN_ID, true);
        deepClimbIntakeMotor.setIdleMode(IdleMode.kBrake);

        deepClimbMotor.withGains(new PIDProfile().setPID(DEEP_CLIMB.DEEP_CLIMB_HOLD_P, 0, 0));
    }

    public void setDeepClimbPercentOutput(double percent){
        if(percent != 0){
            deepClimbMotor.setPercentOutput(percent);
            motorRotationsSet = false;
        }
        else{
            deepClimbMotor.setPercentOutput(percent);
            motorRotationsSet = true;
            motorRotations = deepClimbMotor.getRotations();
        }
    }

    public void setDeepClimbIntakePercentOutput(double percent){
        deepClimbIntakeMotor.setPercentOutput(percent);
    }

    // public boolean isDeepClimbDeployed() {
    //     return deepClimbMotor.getRotations() > DEEP_CLIMB.DEEP_CLIMB_START_POSITION;
    // }

    public Command setDeepClimbGrabPositionCommand(){
        return this.run(() -> setDeepClimbPercentOutput(-1)).until(
            () -> deepClimbMotor.getRotations() <= DEEP_CLIMB.DEEP_CLIMB_GRAB_POSITION
        ).finallyDo(() -> setDeepClimbPercentOutput(0));
    }


    public void stopDeepClimb() {
        setDeepClimbPercentOutput(0);
    }

    public void stopDeepIntakeClimb() {
        setDeepClimbIntakePercentOutput(0);
    }

    public Command setDeepClimbCommand(double percent){
        return this.run(()->{setDeepClimbPercentOutput(percent);}) 
        .until(() -> (
            deepClimbMotor.getRotations()>=DEEP_CLIMB.DEEP_CLIMB_MAX_POSITION
            && percent > 0
        ))
        .finallyDo(() -> this.setDeepClimbPercentOutput(0));
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
        if(motorRotationsSet){
            deepClimbMotor.setPosition(motorRotations);
        }
    }
}
