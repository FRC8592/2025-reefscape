package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Intake extends SubsystemBase {

    NewtonMotor intakeMotor;
    public Intake() {
        intakeMotor = new KrakenX60Motor(CAN.INTAKE_INNER_MOTOR_CAN_ID, true);
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
