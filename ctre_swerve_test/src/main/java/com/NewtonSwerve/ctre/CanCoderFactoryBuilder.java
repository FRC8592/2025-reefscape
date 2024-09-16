package com.NewtonSwerve.ctre;

import com.NewtonSwerve.AbsoluteEncoder;
import com.NewtonSwerve.AbsoluteEncoderFactory;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            config.MagnetSensor.MagnetOffset = Math.toDegrees(configuration.getOffset())/360;
            config.MagnetSensor.SensorDirection = direction == Direction.CLOCKWISE? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;

            CANcoder encoder = new CANcoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(config, 0.25), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(
                    encoder.optimizeBusUtilization(4, 0.25),
                    "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition().getValueAsDouble());
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
