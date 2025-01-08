package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.*;
import edu.wpi.first.util.WPISerializable;

/**
 * Class for storing useful suppliers/lambdas
 */
public final class Suppliers {
    public static class LoggedWPILibSupplier<T extends WPISerializable> implements Supplier<T>{
        private Supplier<T> supplier;
        private String name;
        public LoggedWPILibSupplier(Supplier<T> supplier, String name){this.supplier = supplier; this.name = name;}
        public T get(){T t = supplier.get(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, t); return t;}
    }
    public static class LoggedEnumSupplier<E extends Enum<E>> implements Supplier<E>{
        private Supplier<E> supplier;
        private String name;
        public LoggedEnumSupplier(Supplier<E> supplier, String name){this.supplier = supplier; this.name = name;}
        public E get(){E e = supplier.get(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, e); return e;}
    }
    public static class LoggedBooleanSupplier implements BooleanSupplier{
        private BooleanSupplier supplier;
        private String name;
        public LoggedBooleanSupplier(BooleanSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public boolean getAsBoolean(){boolean b = supplier.getAsBoolean(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, b); return b;}
    }
    public static class LoggedDoubleSupplier implements DoubleSupplier{
        private DoubleSupplier supplier;
        private String name;
        public LoggedDoubleSupplier(DoubleSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public double getAsDouble(){double d = supplier.getAsDouble(); Logger.recordOutput(SUPPLIERS.LOG_PATH+name, d); return d;}
    }
    public static class LoggedIntSupplier implements IntSupplier{
        private IntSupplier supplier;
        private String name;
        public LoggedIntSupplier(IntSupplier supplier, String name){this.supplier = supplier; this.name = name;}
        public int getAsInt(){int d = supplier.getAsInt(); Logger.recordOutput(name, d); return d;}
    }

    /**
     * {@code getAsBoolean()} returns {@code true} when the robot it running on the red side and
     * {@code false} when on the blue side. Defaults to {@code false} if the alliance color is
     * inaccessible.
     */
    public static final LoggedBooleanSupplier robotRunningOnRed = new LoggedBooleanSupplier(
        () -> DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue,
        "RobotRunningOnRed"
    );

    /**
     * {@code get()} returns the direction the gyroscope should believe it's facing
     * when the front of the robot is aimed away from the driver station.
     */
    public static final Supplier<Rotation2d> currentGyroscopeRotationOffset = new LoggedWPILibSupplier<Rotation2d>(
        () -> robotRunningOnRed.getAsBoolean() && !DriverStation.isAutonomous() 
        ? SWERVE.RED_PERSPECTIVE_ROTATION
        : SWERVE.BLUE_PERSPECTIVE_ROTATION,
        "CurrentGyroscopeRotationOffset"
    );

    //TODO: Add more useful suppliers here
}
