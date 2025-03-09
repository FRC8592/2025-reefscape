package frc.robot.helpers.motor.talonfx;


import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.Utils;
import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor;

public abstract class TalonFXMotor extends NewtonMotor {
    protected TalonFX motor;

    private TalonFXConfiguration configuration;

    private PositionVoltage positionOutput;
    private VelocityVoltage velocityOutput;
    private DutyCycleOut percentOutput;
    private MotionMagicVoltage motionMagicOutput;

    protected TalonFXMotor(int motorID, MotorConstants constants) {
        this(motorID, false, constants);
    }

    protected TalonFXMotor(int motorID, boolean inverted, MotorConstants constants) {
        super(motorID, inverted, constants);
        
        this.motor = new TalonFX(motorID);
        this.configuration = new TalonFXConfiguration();
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.motor.getConfigurator().apply(configuration);

        this.positionOutput = new PositionVoltage(0.0);
        this.velocityOutput = new VelocityVoltage(0.0);
        this.percentOutput = new DutyCycleOut(0);
        this.motionMagicOutput = new MotionMagicVoltage(0);
        this.percentOutput.OverrideBrakeDurNeutral = false;
        this.positionOutput.OverrideBrakeDurNeutral = false;
        this.velocityOutput.OverrideBrakeDurNeutral = false;
        this.motionMagicOutput.OverrideBrakeDurNeutral = false;
       }

    public void configureMotionMagic(double maxAcceleration, double cruiseVelocity ){
        MotionMagicConfigs motionMagicConfig = configuration.MotionMagic;

        motionMagicConfig.withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(maxAcceleration));
        motionMagicConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(cruiseVelocity));
        motionMagicConfig.withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(maxAcceleration*100));
        motor.getConfigurator().apply(configuration);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.configuration.MotorOutput.Inverted = inverted ? 
            InvertedValue.Clockwise_Positive :
            InvertedValue.CounterClockwise_Positive;

        this.motor.getConfigurator().apply(configuration);
    }

    

    @Override
    public void withGains(PIDProfile gains) {
        super.motorPIDGains.add(gains.getSlot(), gains);
        
        switch (gains.pidSlot) {
            case 0:
                Slot0Configs slot0Config = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot0Config);
            case 1:
                Slot1Configs slot1Config = configuration.Slot1
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot1Config);
                break;
            case 2:
                Slot2Configs slot2Config = configuration.Slot2
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slot2Config);
                break;
            default:
                Slot0Configs slotConfig = configuration.Slot0
                    .withKP(gains.kP)
                    .withKI(gains.kI)
                    .withKD(gains.kD)
                    .withKA(gains.kA)
                    .withKV(gains.kV)
                    .withKS(gains.kS);

                this.motor.getConfigurator().apply(slotConfig);
                break;
        }

        
    }

    @Override
    public void setPercentOutput(double percent) {
        this.motor.setControl(percentOutput.withOutput(percent));
    }

    @Override
    public void setVoltage(double voltage, int slot) {
        this.motor.setVoltage(voltage);
        motor.getMotorVoltage();
    }

    public double getVoltage(){
        return motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void setVelocity(double desiredRPM, int pidSlot) {
        double desiredRPS = desiredRPM / 60.0;
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRPS, 
                -motorPIDGains.get(pidSlot).maxVelocity,
                motorPIDGains.get(pidSlot).maxVelocity
            );
        }
        this.motor.setControl(velocityOutput.withSlot(pidSlot).withVelocity(desiredRPS));
    }

    @Override
    public void setPosition(double desiredRotations, int pidSlot) {
        if (motorPIDGains.get(pidSlot) != null) {
            Utils.clamp(
                desiredRotations,
                motorPIDGains.get(pidSlot).softLimitMin,
                motorPIDGains.get(pidSlot).softLimitMax
            );
        }
        this.motor.setControl(motionMagicOutput.withSlot(pidSlot).withPosition(desiredRotations));
    }

    @Override
    public void setFollowerTo(NewtonMotor master, boolean reversed) {
        this.motor.setControl(new Follower(master.getAsTalonFX().motor.getDeviceID(), reversed));
    }

    @Override
    public void setCurrentLimit(int currentAmps) {
        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimit = currentAmps;
        currentConfigs.StatorCurrentLimitEnable = true;

        this.configuration.CurrentLimits = currentConfigs;
        this.motor.getConfigurator().apply(configuration);
    }

    public void setPositionSoftLimit(double low, double high) {

        SoftwareLimitSwitchConfigs soft_limit_motor = new SoftwareLimitSwitchConfigs();
        soft_limit_motor.ForwardSoftLimitEnable = true;
        soft_limit_motor.ReverseSoftLimitEnable = true;

        soft_limit_motor.ForwardSoftLimitThreshold = high;
        soft_limit_motor.ReverseSoftLimitThreshold = low;

        this.configuration.withSoftwareLimitSwitch(soft_limit_motor);

        this.motor.getConfigurator().apply(this.configuration);

    }

    @Override
    public void setIdleMode(IdleMode idleMode) {
        NeutralModeValue neutralMode = NeutralModeValue.Brake;
        switch(idleMode) {
            case kCoast:
                neutralMode = NeutralModeValue.Coast;
                break;
            case kBrake: default: // Should default to brake mode
                break;
        }
        this.motor.setNeutralMode(NeutralModeValue.Brake);
    }


    @Override
    public double getVelocityRPM() {
        return this.motor.getVelocity().getValueAsDouble();
    }

    @Override
    public double getRotations() {
        return this.motor.getPosition().getValueAsDouble();
    }

    @Override
    public double getAppliedVoltage() {
        return this.motor.getMotorVoltage().getValueAsDouble();
    }

    @Override
    public void resetEncoderPosition(double rotations) {
        this.motor.setPosition(rotations);
    }



}