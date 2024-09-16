package com.NewtonSwerve;


import com.NewtonSwerve.SwerveModule;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class NewtonModule {
    public SwerveModule module;
    private double wheelCircumference;

    public NewtonModule(SwerveModule module, double wheelCircumference) {
        this.module = module;
        this.wheelCircumference = wheelCircumference;
    }

    public TalonFX getThrottleMotor() {
        return module.getDriveController().getDriveFalcon();
    }

    public TalonFX getAzimuthMotor() {
        return module.getSteerController().getSteerMotor();
    }

    public double getThrottleEncoder() {
        return this.getThrottleMotor().getPosition().getValueAsDouble();
    }

    public void resetThrottleEncoder() {
        this.getThrottleMotor().setPosition(0);
    }

    public void resetAbsoluteAngle() {
        this.module.getSteerController().resetAbsoluteAngle();
    }

    private final VelocityDutyCycle velocityMode = new VelocityDutyCycle(0);
    public void setThrottleVelocity(double inputVelocity, SwerveModule module) {
        this.getThrottleMotor().setControl(velocityMode.withVelocity(inputVelocity));
    }

    public void setModule(double steerAngle, double velocityMetersPerSecond) {
        double velocityToApply;
        if (module.setModuleSteerAngle(steerAngle)) {
            velocityToApply = -velocityMetersPerSecond;
        } else {
            velocityToApply = velocityMetersPerSecond;
        }
        setThrottleVelocity(velocityToApply, module);
    }

    public void setThrottleCurrentLimit(double currentLimit) {
        ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(0.02).withTorqueClosedLoopRampPeriod(0.02).withVoltageClosedLoopRampPeriod(0.02);
        var talonFXConfigurator = this.getThrottleMotor().getConfigurator();
        var limitConfigs = new CurrentLimitsConfigs();

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = currentLimit;
        limitConfigs.StatorCurrentLimitEnable = true;

        talonFXConfigurator.apply(rampConfigs);
        talonFXConfigurator.apply(limitConfigs);
    }

    public double getAppliedCurrent() {
        return this.getThrottleMotor().getStatorCurrent().getValueAsDouble();
    }

    public double getThrottleVelocity(SwerveModule module) {
        return this.getThrottleMotor().getVelocity().getValueAsDouble();
    }

    public double getAzimuthVelocity(SwerveModule module) {
        return this.getAzimuthMotor().getVelocity().getValueAsDouble();
    }

    public double getSteerAngle() {
        return this.module.getSteerAngle();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(this.getThrottleEncoder() / 4096.0 / this.wheelCircumference,
                new Rotation2d(this.getSteerAngle()));
    }
}
