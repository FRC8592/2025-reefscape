package frc.robot.subsystems.scoring;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
// import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

public class Intake extends SubsystemBase {

    NewtonMotor intakeMotor;
    private DigitalInput intakeSensor;
    public Intake() {
        intakeMotor = new KrakenX60Motor(CAN.INTAKE_MOTOR_CAN_ID, true);
        intakeSensor = new DigitalInput(INTAKE.INTAKE_BEAM_BREAK_DIGITAL_ID);
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

    public Command stopCommand(){
        return this.run(()->{setIntakePercentOutput(0);});
    }

    public boolean robotHasCoral(){
        return  !intakeSensor.get();
        
    }

    public void periodic() {
        Logger.recordOutput(INTAKE.LOG_PATH+"Intake|HasCoral", robotHasCoral());
    }
}
