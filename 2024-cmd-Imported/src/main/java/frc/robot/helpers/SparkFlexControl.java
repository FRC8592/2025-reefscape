package frc.robot.helpers;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SmartMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class SparkFlexControl {
    public SparkFlex motor;
    public RelativeEncoder motorEncoder;
    public SparkClosedLoopController motorControl;
    private SparkFlexConfig motorConfig;

    /**
     * Create a Newton² controller for a NEO Vortex (with Spark Flex)
     *
     * @param motorCanID the motor's ID on the CAN bus
     * @param coastMode if true, the motor coasts when asked to apply 0 power.
     * Otherwise, it brakes.
     */
    public SparkFlexControl(int motorCanID, boolean coastMode){
        motor = new SparkFlex(motorCanID, MotorType.kBrushless);

        motorConfig = new SparkFlexConfig();
        
        motorControl = motor.getClosedLoopController();
        motorEncoder = motor.getEncoder();
        if (coastMode){
            motorConfig.idleMode(IdleMode.kCoast);
        }
        else{
            motorConfig.idleMode(IdleMode.kCoast);
        }
        this.apply(motorConfig);
        motor.set(0);
    }

    /**
     * Run the motor at a set velocity using the default PID slot
     *
     * @param RPM the velocity in RPM
     */
    public void setVelocity(double RPM){
        motorControl.setReference(RPM, ControlType.kVelocity);
    }

    /**
     * Run the motor at a set velocity using a specific PID slot
     *
     * @param RPM the velocity in RPM
     * @param slotID the PID slot to use
     */
    public void setVelocity(double RPM, int slotID){
        motorControl.setReference(RPM, ControlType.kVelocity, getSlotOf(slotID));
    }

    /**
     * Drive the motor to a set position using the default PID slot
     *
     * @param rotations the target position in motor rotations
     */
    public void setPosition(double rotations){
        motorControl.setReference(rotations, ControlType.kPosition);
    }

    /**
     * Drive the motor with a set amount of power
     *
     * @param power the amount ofpower to apply, where -1 is full negative
     * power, 0 will apply the set neutral mode (brake or coast), and 1 applies
     * full forward power
     */
    public void setPercentOutput(double power){
        motor.set(power);
    }

    /**
     * Cut power to the motor hub and apply the set neutral mode (brake or coast)
     */
    public void stop(){
        motor.set(0);
    }

    /**
     * Configure PID for the specified slot ID
     *
     * @param P
     * @param I
     * @param D
     * @param FF
     * @param slotID
     */
    public void setPIDF(double P, double I, double D, double FF, int slotID){
        ClosedLoopConfig pidConfig = new ClosedLoopConfig().pidf(P, I, D, FF, getSlotOf(slotID));
        motorConfig.apply(pidConfig);
        this.apply(motorConfig);
    }

    /**
     * Configure SmartMotion for the specified slot ID
     */
    public void configSmartMotion(double acceleration, double maxVelocity, int slotID){
        ClosedLoopConfig pidConfig = motorConfig.closedLoop;
        SmartMotionConfig smartMotionConfig = (
            new SmartMotionConfig()
            .maxAcceleration(acceleration, getSlotOf(slotID))
            .maxVelocity(maxVelocity, getSlotOf(slotID))
        );
        pidConfig.apply(smartMotionConfig);
        motorConfig.apply(pidConfig);
        this.apply(motorConfig);
    }

    /**
     * Returns the motor's velocity in RPM
     */
    public double getVelocity(){
        return motorEncoder.getVelocity();
    }

    /**
     * Returns the motor's position in rotations
     */
    public double getPosition(){
        return motorEncoder.getPosition();
    }

    /**
     * Invert the motor (positional setpoints, velocity setpoints,
     * the direction power mode drives it, etc)
     */
    public void setInverted(){
        motor.setInverted(true);
    }

    /**
     * Returns the motor's position in ticks
     */
    public double getTicks(){
        return motorEncoder.getPosition()*4096;
    }

    /**
     * Set a motor for this motor to mimmick the movements of.
     *
     * @param motorToFollow the motor for this one to follow
     */
    public void setFollower(SparkFlexControl motorToFollow){
        motorConfig.follow(motor);
        this.apply(motorConfig);
    }

    /**
     * Set a motor for this motor to mimmick the movements of.
     *
     * @param sfc the motor for this one to follow
     * @param inverted whether to invert this motor's movements
     * relative to the motor it's following
     */
    public void follow(SparkFlexControl sfc, boolean inverted) {
        motorConfig.follow(motor, inverted);
        this.apply(motorConfig);
    }

    /**
     * Set a soft limit for the motor.
     *
     * @param direction the direction the motor must be traveling to
     * trigger the limit
     * @param rotations the number of rotations to limit the motor to
     */
    public void setSoftLimit(boolean forward, double rotations){
        SoftLimitConfig softLimitConfig = motorConfig.softLimit;
        if(forward){
            softLimitConfig.forwardSoftLimit(rotations);
            softLimitConfig.forwardSoftLimitEnabled(true);
        }
        else{
            softLimitConfig.reverseSoftLimit(rotations);
            softLimitConfig.reverseSoftLimitEnabled(true);
        }
        motorConfig.apply(softLimitConfig);
        this.apply(motorConfig);
    }

    /**
     * Set a target position for the motor to go to using a Smart
     * Motion profile.
     *
     * @param rotations the target position in rotations
     */
    public void setPositionSmartMotion(double rotations){
        motorControl.setReference(rotations, ControlType.kSmartMotion);
    }

    /**
     * Set the current limit of the motor.
     *
     * @param stallLimit the current limit when the motor is stalled
     * @param fullRPMLimit the current limit when at 5,700 RPM (the
     * motor's free speed)
     */
    public void setCurrentLimit(int stallLimit, int fullRPMLimit){
        this.motorConfig.smartCurrentLimit(stallLimit, fullRPMLimit);
        this.apply(motorConfig);
    }

    /**
     * @return a ClosedLoopSlot corresponding to the integer ID passed in
     */
    private ClosedLoopSlot getSlotOf(int id){
        ClosedLoopSlot slot;
        switch(id){
            case 0:
                slot = ClosedLoopSlot.kSlot0;
                break;
            case 1:
                slot = ClosedLoopSlot.kSlot1;
                break;
            case 2:
                slot = ClosedLoopSlot.kSlot2;
                break;
            case 3:
                slot = ClosedLoopSlot.kSlot3;
                break;
            default:
                throw new RuntimeException("The only available PID slots on the Spark Flex are 0, 1, 2, and 3.");
        }
        return slot;
    }
    private void apply(SparkFlexConfig config){
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}