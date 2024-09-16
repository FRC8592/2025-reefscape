package com.NewtonSwerve.ctre;

import com.NewtonSwerve.DriveController;
import com.NewtonSwerve.DriveControllerFactory;
import com.NewtonSwerve.ModuleConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final double CAN_TIMEOUT_SEC = 0.25;
    private static final double STATUS_FRAME_GENERAL_PERIOD_HZ = 4;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    private double kP = Double.NaN;
    private double kI = Double.NaN;
    private double kD = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public Falcon500DriveControllerFactoryBuilder withPidConstants(double proportional, double integral,
            double derivative) {
        this.kP = proportional;
        this.kI = integral;
        this.kD = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(kP) && Double.isFinite(kI) && Double.isFinite(kD);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter()
                    * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasPidConstants()) {
                motorConfiguration.Slot0.kP = kP;
                motorConfiguration.Slot0.kI = kI;
                motorConfiguration.Slot0.kD = kD;
            }

            if (hasVoltageCompensation()) {
                motorConfiguration.Voltage.PeakForwardVoltage = nominalVoltage;
                motorConfiguration.Voltage.PeakReverseVoltage = nominalVoltage;
            }

            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive: InvertedValue.CounterClockwise_Positive;
            TalonFX motor = new TalonFX(driveConfiguration);
            // Set Status Frame Period to 50 Hz
            motor.optimizeBusUtilization(50, 0.25);

            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500");


            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                            motor.optimizeBusUtilization(STATUS_FRAME_GENERAL_PERIOD_HZ, CAN_TIMEOUT_SEC),
                    "Failed to configure Falcon status frame period");

            return new ControllerImplementation(motor, sensorVelocityCoefficient, hasVoltageCompensation());
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation()
                ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage
                : 12.0;
        private final boolean voltageCompensation;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, boolean voltageCompensation) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.voltageCompensation = voltageCompensation;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setVoltage(voltage);
        }

        @Override
        public double getStateVelocity() {
            return motor.getVelocity().getValueAsDouble() * sensorVelocityCoefficient;
        }

        @Override
        public TalonFX getDriveFalcon() {
            return motor;
        }
    }
}
