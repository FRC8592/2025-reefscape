package frc.robot.helpers.motor.spark;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor;

public abstract class SparkBaseMotor<M extends SparkBase, C extends SparkBaseConfig> extends NewtonMotor {
    protected M motor;
    protected SparkClosedLoopController motorCtrl;
    protected RelativeEncoder encoder;
    protected C config;

    protected SparkBaseMotor(M motor, C config, boolean inverted, MotorConstants constants) {
        super(motor.getDeviceId(), inverted, constants);
        this.motor = motor;
        this.motorCtrl = motor.getClosedLoopController();
        this.encoder = motor.getEncoder();
        
        this.config = config;
        this.config.inverted(inverted);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.config.inverted(inverted);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        ClosedLoopSlot slot;
        if (gains.getSlot() == 1) {
            slot = ClosedLoopSlot.kSlot1;
        } else if (gains.getSlot() == 2) {
            slot = ClosedLoopSlot.kSlot2;
        } else if (gains.getSlot() == 3) {
            slot = ClosedLoopSlot.kSlot3;
        } else {
            slot = ClosedLoopSlot.kSlot0;
        }

        this.config.closedLoop
            .p(gains.kP, slot)
            .i(gains.kI, slot)
            .d(gains.kD, slot)
            .velocityFF(gains.kV, slot)
            ;

        if (gains.softLimit) {
            this.config.softLimit.forwardSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.forwardSoftLimit(gains.softLimitMax);
            this.config.softLimit.reverseSoftLimitEnabled(gains.softLimit);
            this.config.softLimit.reverseSoftLimit(gains.softLimitMax);
        }

        this.config.closedLoop.maxMotion
            .maxVelocity(gains.maxVelocity, slot)
            .maxAcceleration(gains.maxAcceleration, slot)
            .positionMode(com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode.kMAXMotionTrapezoidal, slot)
            .allowedClosedLoopError(gains.tolerance, slot)
        ;

        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
    }

    @Override
    public void setVelocity(double desiredVelocityRPM, int pidSlot) {
        if (motorPIDGains.size() > 0) {
            Utils.clamp(
                desiredVelocityRPM, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }

        ClosedLoopSlot slot = getSlot(pidSlot);

        // double arbFF = 0d;
        // if (feedForward.size() > 0) {
        //     arbFF = feedForward.get(pidSlot).calculate(getVelocityRPM(), Robot.CLOCK.dt());
        // }

        this.motorCtrl.setReference(
            desiredVelocityRPM, 
            ControlType.kMAXMotionVelocityControl, 
            slot
        );
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains != null) {
            Utils.clamp(
                desiredRotations, 
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        
        this.motorCtrl.setReference(desiredRotations, ControlType.kMAXMotionPositionControl, getSlot(pidSlot));
    }

    @Override
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
        this.config.follow(master.getAsSparkFlex().motor);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        this.config.smartCurrentLimit(currentAmps);
        this.config.secondaryCurrentLimit(currentAmps);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        com.revrobotics.spark.config.SparkBaseConfig.IdleMode mode;
        switch (idleMode) {
            case kCoast:
                mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast;
                break;
            case kBrake:
            // On default set to brake mode
            default:
            mode = com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake;
                break;
        }

        this.config.idleMode(mode);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    @Override
    public double getRotations() {
        return encoder.getPosition();
    }

    @Override
    public double getAppliedVoltage() {
        return motor.getBusVoltage();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.encoder.setPosition(rotations);
    }

    private ClosedLoopSlot getSlot(int slot) {
        switch (slot) {
            case 1:
                return ClosedLoopSlot.kSlot1;
            case 2:
                return ClosedLoopSlot.kSlot2;
            case 3:
                return ClosedLoopSlot.kSlot3;
            default:
                return ClosedLoopSlot.kSlot0;
        }
    }
}