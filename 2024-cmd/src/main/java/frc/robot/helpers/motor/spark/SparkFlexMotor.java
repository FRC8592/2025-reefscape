package frc.robot.helpers.motor.spark;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.helpers.motor.MotorConstants;

public class SparkFlexMotor extends SparkBaseMotor<SparkFlex, SparkFlexConfig> {
    public SparkFlexMotor(int motorID) {
        this(motorID, false);
    }

    public SparkFlexMotor(int motorID, boolean inverted) {
        super(
            new SparkFlex(motorID, MotorType.kBrushless), 
            new SparkFlexConfig(),
            inverted,
            new MotorConstants(
                6784d, 
                211d, 
                3.6, 
                575.1
            )
        );
    }
}