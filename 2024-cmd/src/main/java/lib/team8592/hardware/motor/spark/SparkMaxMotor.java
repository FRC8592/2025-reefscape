package lib.team8592.hardware.motor.spark;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import lib.team8592.hardware.motor.MotorConstants;

public class SparkMaxMotor extends SparkBaseMotor<SparkMax, SparkMaxConfig> {
    public SparkMaxMotor(int motorID) {
        this(motorID, false);
    }

    public SparkMaxMotor(int motorID, boolean inverted) {
        super(
            new SparkMax(motorID, MotorType.kBrushless), 
            new SparkMaxConfig(),
            inverted, 
            new MotorConstants(
                5676d, 
                105d, 
                2.6,
                493.5
            )
        );
    }
}