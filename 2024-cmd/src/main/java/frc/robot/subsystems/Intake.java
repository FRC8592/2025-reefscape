package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Intake extends SubsystemBase {

    NewtonMotor intakeMotor;
    public Intake() {
        intakeMotor = new KrakenX60Motor(CAN.INTAKE_MOTOR_CAN_ID, true);
    }

    public void setIntakePercentOutput(double percent){
        intakeMotor.setPercentOutput(percent);
    }

    public void stop() {
        setIntakePercentOutput(0);
    }

    public Command setIntakeCommand(double percent){
        return this.run(()->{setIntakePercentOutput(percent);});
    }

    public Command stopIntakeCommand(){
        return this.run(()->{setIntakePercentOutput(0);});
    }

    public void periodic() {
    }
}
