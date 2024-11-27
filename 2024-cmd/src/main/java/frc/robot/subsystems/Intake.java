package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.SparkFlexControl;
import frc.robot.helpers.SparkMaxControl;

public class Intake extends SubsystemBase {
    //private TalonFX topMotor = new TalonFX(CAN.INTAKE_TOP_MOTOR_CAN_ID);
    private SparkMaxControl topMotor = new SparkMaxControl(CAN.INTAKE_TOP_MOTOR_CAN_ID,true);
    // private TalonFX bottomMotor = new TalonFX(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID);
    private SparkFlexControl bottomMotor = new SparkFlexControl(CAN.INTAKE_BOTTOM_MOTOR_CAN_ID, true);

    public Intake() {
        // TalonFXConfiguration motorConfigs = new TalonFXConfiguration();
        // topMotor.getConfigurator().apply(motorConfigs);
    }

    public void runTopMotor(int speedRPM) {
        topMotor.setPercentOutput(speedRPM/6000d);
        //topMotor.setControl(voltage);
    }

    public void runBottomMotor(int speedRPM) {
        bottomMotor.setPercentOutput(speedRPM/6000d);
    }

    public void stop() {
        runTopMotor(0);
        runBottomMotor(0);
    }
}
