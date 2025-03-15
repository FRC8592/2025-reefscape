package frc.robot.helpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutTime;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;

public class SysID {

    private Voltage volts;
    private TalonFX[] swerveMotors;
    private String name;
    private Subsystem subsystemBase;
    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;
    
    // /**
    //  * Creates an instance of the SysID object which can be used to calculate feedforward, pid values, voltage, 
    //  * and run the necessary SysID tests to help tune PID faster. 
    //  * 
    //  * Should only be used for singular motor tests with SparkFlex motors that do not work with another motor.
    //  * 
    //  * @param testMotor the motor being used for the SysID test.
    //  * @param name the name of the mechanism for logging purposes.
    //  * @param subsystemBase the subsystem the motor is a part of for logging purposes.
    //  */
    // public SysID(SparkFlexControl testMotor, String name, SubsystemBase subsystemBase){
    //     this.testMotor = testMotor;
    //     this.name = name;
    //     this.subsystemBase = subsystemBase;
    // }

    // /**
    //  * Creates an instance of the SysID object which can be used to calculate feedforward, pid values, voltage, 
    //  * and run the necessary SysID tests to help tune PID faster. 
    //  * 
    //  * Should be used for mechanisms with two SparkFlex motors working together and moving the same mechanism together.
    //  * 
    //  * @param testMotor1 the motor of which the data is being logged.
    //  * @param testMotor2 the motor which is following testMotor1.
    //  * @param name the name of the mechanism for logging purposes.
    //  * @param subsystemBase the subsystem the motor is a part of for logging purposes.
    //  */
    // public SysID(SparkFlexControl testMotor1, SparkFlexControl testMotor2, String name, SubsystemBase subsystemBase){
    //     testMotor = testMotor1;
    //     this.testMotor2 = testMotor2;
    //     this.name = name;
    //     this.subsystemBase = subsystemBase;
    // }

    /**
     * Creates an instance of the SysID object which can be used to calculate feedforward, pid values, voltage, 
     * and run the necessary SysID tests to help tune PID faster for the swerve. 
     * 
     * This should be used for the swerve drive which will take in the 4 motors and run them simultaneously during SysID tests
     * as well as log their data to return SysID values.
     * 
     * @param swerveMotor1 Front Left Motor.
     * @param swerveMotor2 Front Right Motor.
     * @param swerveMotor3 Back Left Motor.
     * @param swerveMotor4 Back Right Motor.
     * @param name the name of the mechanism for logging purposes.
     * @param subsystemBase the subsystem the motor is a part of for logging purposes.
     */
    public SysID(TalonFX swerveMotor1, TalonFX swerveMotor2, TalonFX swerveMotor3, TalonFX swerveMotor4, String name, Subsystem subsystemBase){
        swerveMotors = new TalonFX[4];
        swerveMotors[0] = swerveMotor1;
        swerveMotors[1] = swerveMotor2;
        swerveMotors[2] = swerveMotor3;
        swerveMotors[3] = swerveMotor4;

        this.name = name;
        this.subsystemBase = subsystemBase;
    }

    /**
     * The voltage assigned converted to WPILib voltage units to use in SysID tests.
     * 
     * @param voltage voltage of the motor
     * @return the voltage in the WPILib units
     */
    private Voltage getVoltageAsWpiUnit(double voltage){

        Voltage volt = Volts.of(voltage);
        MutVoltage mutVolt = volt.mutableCopy();

        double motorVoltage = volts.baseUnitMagnitude();

        return mutVolt.mut_replace(motorVoltage, Volts);


    }

    // /**
    //  * The current position of the motor in terms of rotations converted to WPILib angle units to use in SysID tests.
    //  *
    //  * @param position position of the motor in rotations
    //  * @return the angle of the motor in WPILib units
    //  */
    // private Angle getAngularPositionAsWpiUnit(double position){

    //     Angle angle = Rotations.of(0);
    //     MutAngle mutAngle = angle.mutableCopy();

    //     return mutAngle.mut_replace(position, Rotations);


    // }

    /**
     * The current position in terms of rotations converted to WPILib distance units in meters to use in SysID tests.
     * 
     * @param position position of the motor in rotations
     * @return the distance the motor has traveled in WPILib meter units
     */
    private Distance getLinearPositionAsWpiUnit(double position){
        
        Distance distance = Meters.of(0);
        MutDistance mutDistance = distance.mutableCopy();
        
        return mutDistance.mut_replace(position, Meters);
        
    }



    /**
     * The current velocity in RPM converted to WPILib angular velocity units to use in SysID tests.
     * 
     * @param velocity velocity of the motor
     * @return the angular velocity of the motor in WPILib units
     */
    private AngularVelocity getAngularVelocityAsWpiUnit(double velocity){

        AngularVelocity finalVelocity = RotationsPerSecond.of(0);
        MutAngularVelocity mutFinalVelocity = finalVelocity.mutableCopy();

        return mutFinalVelocity.mut_replace(velocity, RotationsPerSecond);

    }

    /**
     * The current velocity in RPM converted to WPILib linear velocity in MetersPerSeconds units to use in SysID tests.
     * 
     * @param velocity velocity of the motor
     * @return the linear velocity of the motor in WPILib units
     */
    private LinearVelocity getLinearVelocityAsWpiUnit(double velocity){

        LinearVelocity finalVelocity = MetersPerSecond.of(0);
        MutLinearVelocity mutFinalVelocity = finalVelocity.mutableCopy();

        return mutFinalVelocity.mut_replace(velocity, MetersPerSecond);

    }

    // /**
    //  * Runs one SparkFlex motor in voltage mode and logs the information in advantage kit
    //  * 
    //  * @param volts volts in WPILib units
    //  */
    // private void runMotorAtVoltage(Voltage volts){
    //     this.volts = volts;
    //     Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
    //     Logger.recordOutput(name + "/Position", testMotor.getPosition());
    //     Logger.recordOutput(name + "/Velocity", testMotor.getVelocity());

    //     testMotor.motor.setVoltage(volts.baseUnitMagnitude());
    // }

    // /**
    //  * Runs two SparkFlex motors together in voltage mode and logs the information in advantage kit
    //  * 
    //  * @param volts volts in WPILib units
    //  */
    // private void runTwoMotorsAtVoltage(Voltage volts){
    //     this.volts = volts;
    //     Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
    //     Logger.recordOutput(name + "/Position", testMotor.getPosition());
    //     Logger.recordOutput(name + "/Velocity", testMotor.getVelocity());

    //     testMotor.motor.setVoltage(volts.baseUnitMagnitude());
    //     testMotor2.motor.setVoltage(-volts.baseUnitMagnitude());
    // }

    /**
     * Runs one TalonFX motor in voltage mode and logs the information in advantage kit
     * 
     * @param volts volts in WPILib units
     * @param swerveMotor which swerve motor is being run
     * @param nameString the name of the motor for Logging in Advantage Kit
     */
    private void runSwerveMotorAtVoltage(Voltage volts, TalonFX swerveMotor, String nameString){
        this.volts = volts;
        Logger.recordOutput(name + "/Voltage", volts.baseUnitMagnitude());
        Logger.recordOutput(name + "/Position", swerveMotor.getPosition().getValueAsDouble());
        Logger.recordOutput(name + "/VelocityRPM", swerveMotor.getVelocity().getValueAsDouble());

        swerveMotor.setVoltage(volts.baseUnitMagnitude());
    }

    /**
     * Runs the swerve motors together in voltage mode and logs the information in advantage kit
     * 
     * @param volts volts in WPILib units
     */
    private void runSwerveMotorsAtVoltage(Voltage volts){
        runSwerveMotorAtVoltage(volts, swerveMotors[0], "frontLeftMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[1], "frontRightMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[2], "backLeftMotor");
        runSwerveMotorAtVoltage(volts, swerveMotors[3], "backRightMotor");
    }

    /**
     * Logs the 4 swerve motors to make code more readable in the routine
     * 
     * @param log a SysIdRoutineLog object made to put everything in its own log
     */
    private void logSwerveMotors(SysIdRoutineLog log){
        log.motor("frontLeftMotor")
            .voltage(getVoltageAsWpiUnit(swerveMotors[0].getMotorVoltage().getValueAsDouble()))
            .linearPosition(getLinearPositionAsWpiUnit(swerveMotors[0].getPosition().getValueAsDouble()))
            .linearVelocity(getLinearVelocityAsWpiUnit(swerveMotors[0].getVelocity().getValueAsDouble()));

        log.motor("frontRightMotor")
            .voltage(getVoltageAsWpiUnit(swerveMotors[1].getMotorVoltage().getValueAsDouble()))
            .linearPosition(getLinearPositionAsWpiUnit(swerveMotors[1].getPosition().getValueAsDouble()))
            .linearVelocity(getLinearVelocityAsWpiUnit(swerveMotors[1].getVelocity().getValueAsDouble()));

        log.motor("backLeftMotor")
            .voltage(getVoltageAsWpiUnit(swerveMotors[2].getMotorVoltage().getValueAsDouble()))
            .linearPosition(getLinearPositionAsWpiUnit(swerveMotors[2].getPosition().getValueAsDouble()))
            .linearVelocity(getLinearVelocityAsWpiUnit(swerveMotors[2].getVelocity().getValueAsDouble()));

        log.motor("backRightMotor")
            .voltage(getVoltageAsWpiUnit(swerveMotors[3].getMotorVoltage().getValueAsDouble()))
            .linearPosition(getLinearPositionAsWpiUnit(swerveMotors[3].getPosition().getValueAsDouble()))
            .linearVelocity(getLinearVelocityAsWpiUnit(swerveMotors[3].getVelocity().getValueAsDouble()));
    }

    /**
     * Creates a SysIDRoutine for a motor which needs to have PID tuned in terms of speed or angular position.
     * 
    //  * @return a SysIDRoutine for that motor.
    //  */
    // public SysIdRoutine createAngularRoutine(){

    //     SysIdRoutine routine = new SysIdRoutine(
    //         new SysIdRoutine.Config(),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> {runMotorAtVoltage(voltage);}, 
    //             (log)->{log.motor(name)
    //                 .voltage(getVoltageAsWpiUnit(testMotor.getVoltage()))
    //                 .angularPosition(getAngularPositionAsWpiUnit(testMotor.getPosition()))
    //                 .angularVelocity(getAngularVelocityAsWpiUnit(testMotor.getVelocity()));
    //             }, subsystemBase
    //         )
    //     );

    //     return routine;

    // }

    // /**
    //  * Creates a SysIDRoutine for a motor which needs to have PID tuned in terms of distance.
    //  * 
    //  * @return a SysIDRoutine for that motor.
    //  */
    // public SysIdRoutine createLinearRoutine(){

    //     SysIdRoutine routine = new SysIdRoutine(
    //         new SysIdRoutine.Config(),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> {runMotorAtVoltage(voltage);}, 
    //             (log)->{log.motor(name)
    //                 .voltage(getVoltageAsWpiUnit(testMotor.getVoltage()))
    //                 .linearPosition(getLinearPositionAsWpiUnit(testMotor.getPosition()))
    //                 .linearVelocity(getLinearVelocityAsWpiUnit(testMotor.getVelocity()));
    //             }, subsystemBase
    //         )
    //     );

    //     return routine;

    // }

    /**
     * Creates a SysIDRoutine for a mechanism which needs to have PID tuned in terms of speed or angular position but the two motors
     * running it need to move together.
     * 
     * @return a SysIDRoutine for those motors.
     */
    // public SysIdRoutine createAngularTwoMotorRoutine(){

    //     SysIdRoutine routine = new SysIdRoutine(
    //         new SysIdRoutine.Config(null, null, null, null),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> {runTwoMotorsAtVoltage(voltage);}, 
    //             (log)->{log.motor(name)
    //                 .voltage(getVoltageAsWpiUnit(testMotor.getVoltage()))
    //                 .angularPosition(getAngularPositionAsWpiUnit(testMotor.getPosition()))
    //                 .angularVelocity(getAngularVelocityAsWpiUnit(testMotor.getVelocity()));
    //             }, subsystemBase
    //         )
    //     );

    //     return routine;

    // }

    /**
     * Creates a SysIDRoutine for a mechanism which needs to have PID tuned in terms of distance but the two motors running it
     * need to move together.
     * 
     * @return a SysIDRoutine for those motors.
     */
    // public SysIdRoutine createLinearTwoMotorRoutine(){

    //     SysIdRoutine routine = new SysIdRoutine(
    //         new SysIdRoutine.Config(null, null, null, null),
    //         new SysIdRoutine.Mechanism(
    //             (voltage) -> {runTwoMotorsAtVoltage(voltage);}, 
    //             (log)->{log.motor(name)
    //                 .voltage(getVoltageAsWpiUnit(testMotor.getVoltage()))
    //                 .linearPosition(getLinearPositionAsWpiUnit(testMotor.getPosition()))
    //                 .linearVelocity(getLinearVelocityAsWpiUnit(testMotor.getVelocity()));
    //             }, subsystemBase
    //         )
    //     );

    //     return routine;

    // }

    /**
     * Creates a SysIDRoutine for the swerve PID with a timeout to prevent the robot causing accidents.
     * 
     * @return a SysIDRoutine for those motors.
     */
    public SysIdRoutine createSwerveRoutine(int timeout){

        Time time = Seconds.of(timeout); 
        MutTime mutTime = time.mutableCopy(); 

        SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, time),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                    runSwerveMotorsAtVoltage(voltage);
                }, 
                (log)->{
                    logSwerveMotors(log);
                }, subsystemBase
            )
        );

        return routine;

    }

    /**
     * Set the PID of the PIDController to anticipate calculating the PID for the voltage.
     * 
     * @param kP 
     * @param kI
     * @param kD
     */
    public void setPID( double kP, double kI, double kD){
        pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Setting the SVA of the feedforward to eventually calculate the final voltage and feedforward.
     * 
     * @param kS
     * @param kV
     * @param kA
     */
    public void setFeedforward(double kS, double kV, double kA){
        feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    }

    /**
     * Calculates the feedforward using the kS, kV, and kA so the motors can use it.
     * 
     * @param targetSpeed
     * @return the calculated feedforward to be used in the PID controller of the motor
     */
    public double calculatedFeedforward(double targetSpeed){
        return feedforward.calculate(targetSpeed);
    }

    /**
     * Calculates the voltage to be sent to the motor if running in voltage mode.
     * 
     * @param velocity the current velocity of the motor to calculate the accurate PID and voltage
     * @param targetSpeed the target speed trying to be met
     * @return the voltage to be fed to the motor if running in voltage mode 
     */
    public double calculateVoltage(double velocity, double targetSpeed){

        return pidController.calculate(velocity, targetSpeed) + feedforward.calculate(targetSpeed);
    }
}