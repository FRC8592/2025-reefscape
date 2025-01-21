package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public final class SHARED {
        public static final String LOG_FOLDER = "CustomLogs";
    }

    public final class LOGS {
        public static final String GAME = "Reefscape";
        public static final String YEAR = "2025";
        public static final String ROBOT = "Unnamed";
        public static final String TEAM = "8592NewtonSquared";
    }

    public final class CONVERSIONS {
        public static final double METERS_SECOND_TO_TICKS_TALONFX = ((2048 * 6.75 * 60) / (200 * Math.PI * 0.0508));

        public static final double RPM_TO_TICKS_100_MS_TALONFX = 2048.0 / 600.0;

        public static final double ANGLE_DEGREES_TO_TICKS_SPARKFLEX = 4096 / 360.0;
        public static final double TICKS_TO_ANGLE_DEGREES_SPARKFLEX = 360.0 / 4096.0;

        public static final double DEG_TO_RAD = 0.0174533;
        public static final double RAD_TO_DEG = 57.2958;
        public static final double IN_TO_METERS = 0.0254;
        public static final double METERS_TO_FEET = 3.28084;
    }

    public final class MEASUREMENTS {
        public static final double FIELD_LENGTH_METERS = 17.548;
        public static final double FIELD_WIDTH_METERS = 8.052;
    }

    public final class CONTROLLERS {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }

    public final class POWER {
        public static final int SWERVE_MAX_VOLTAGE = 12;
        public static final int SWERVE_DRIVE_CURRENT_LIMIT = 80;
        public static final int SWERVE_STEER_CURRENT_LIMIT = 40;
    }

    public final class VISION {
        public static final String CAM_NAME = "Arducam_OV9782_E";
        public static final Transform3d CAMERA_OFFSET = new Transform3d();
    }

    public final class SUPPLIERS {
        public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Suppliers/";
    }

    public final class SIMULATION {
        public static final double SUPERSTRUCTURE_WIDTH_METERS = Units.inchesToMeters(36d);
        public static final double SUPERSTRUCTURE_HEIGHT_METERS = Units.inchesToMeters(36d);
        public static final double ELEVATOR_OFFSET_X = Units.inchesToMeters(9d);

        public static final double ELEVATOR_HEIGHT_FROM_GROUND = Units.inchesToMeters(3d);

        public static final double ELEVATOR_INITIAL_HEIGHT = Units.inchesToMeters(34.5d);

        public static final double ELEVATOR_DRUM_RADIUS_METERS = Units.inchesToMeters(2d);

        public static final double CLOCK_LENGTH_METERS = Units.inchesToMeters(24d);
        public static final double WRIST_LENGTH_METERS = Units.inchesToMeters(12d);

        public static final double FULL_ELEVATOR_SYSTEM_MASS = 20d; // kg

        public static final double BASE_ATTACHMENT_Y = 0.1; // meters
    }
    
    public final class SWERVE {
        public static final double STEER_P = 100;
        public static final double STEER_I = 0;
        public static final double STEER_D = 0.2;
        public static final double STEER_S = 0;
        public static final double STEER_V = 1.5;
        public static final double STEER_A = 0;

        public static final double DRIVE_P = 3;
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 0;
        public static final double DRIVE_S = 0;
        public static final double DRIVE_V = 0;
        public static final double DRIVE_A = 0;

        //TODO: Double check that these PID constants still work
        public static final double SNAP_TO_kP = 3.7;
        public static final double SNAP_TO_kI = 0.0;
        public static final double SNAP_TO_kD = 0.1;

        public static final int STEER_STATOR_CURRENT_LIMIT = 60;

        public static final int CALCULATED_SLIP_CURRENT = 150;

        public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
        public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);
        public static final double COUPLING_GEAR_RATIO = 3.5714285714285716;
        public static final double DRIVE_GEAR_RATIO = 6.746031746031747;
        public static final double STEER_GEAR_RATIO = 21.428571428571427;
        public static final double WHEEL_RADIUS_INCHES = 2;

        public static final boolean INVERT_LEFT_SIDE = false;
        public static final boolean INVERT_RIGHT_SIDE = true;

        public static final double SIMULATED_STEER_INERTIA = 0.00001;
        public static final double SIMULATED_DRIVE_INERTIA = 0.06;
        public static final double SIMULATION_LOOP_PERIOD = 0.005;
        public static final double STEER_FRICTION_VOLTAGE = 0.25;
        public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

        //TODO: Tone these down appropriately as per BB rules
        public static final double TRANSLATE_POWER_FAST = 1.0;
        public static final double ROTATE_POWER_FAST = 0.75;
        public static final double TRANSLATE_POWER_SLOW = 0.5;
        public static final double ROTATE_POWER_SLOW = 0.3;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 2;

        public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
        public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

        // public static final double BLACK_FRONT_LEFT_STEER_OFFSET = -0.388427734375;
        // public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.462646484375;
        // public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.18017578125;
        // public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.4853515625;

        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_STEER_OFFSET = 0.062255859375;
        public static final double ORANGE_FRONT_RIGHT_STEER_OFFSET = -0.37890625;
        public static final double TEAL_BACK_LEFT_STEER_OFFSET = -0.0029296875;
        public static final double WHITE_BACK_RIGHT_STEER_OFFSET = -0.293701171875;

        public static final boolean BLACK_FRONT_LEFT_STEER_INVERT = true;
        public static final boolean ORANGE_FRONT_RIGHT_STEER_INVERT = true;
        public static final boolean TEAL_BACK_LEFT_STEER_INVERT = true;
        public static final boolean WHITE_BACK_RIGHT_STEER_INVERT = true;


        //TODO: Set these
        public static final double BLACK_FRONT_LEFT_X_POSITION = 8.375;
        public static final double BLACK_FRONT_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double ORANGE_FRONT_RIGHT_X_POSITION = 8.375;
        public static final double ORANGE_FRONT_RIGHT_Y_POSITION = -8.375;

        //TODO: Set these
        public static final double TEAL_BACK_LEFT_X_POSITION = -8.375;
        public static final double TEAL_BACK_LEFT_Y_POSITION = 8.375;

        //TODO: Set these
        public static final double WHITE_BACK_RIGHT_X_POSITION = -8.375;
        public static final double WHITE_BACK_RIGHT_Y_POSITION = -8.375;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_TRANSLATE_kP = 6d;
        public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
        public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

        //TODO: Double check that these still work
        public static final double PATH_FOLLOW_ROTATE_kP = 6d;
        public static final double PATH_FOLLOW_ROTATE_kI = 0d;
        public static final double PATH_FOLLOW_ROTATE_kD = 0.1;

        public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
        public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 4 * Math.PI;

        public static final double PATH_FOLLOW_POSITION_TOLERANCE = 0.01;
        public static final double PATH_FOLLOW_VELOCITY_TOLERANCE = 0.01;
    }

    public class ELEVATOR{
        public static final double L1_SCORE_HEIGHT_INCHES = 0d;
        public static final double L2_SCORE_HEIGHT_INCHES = 0d;
        public static final double L3_SCORE_HEIGHT_INCHES = 0d;
        public static final double L4_SCORE_HEIGHT_INCHES = 0d;

        public static final double L2_ALGAE_HEIGHT_INCHES = 0d;
        public static final double L3_ALGAE_HEIGHT_INCHES = 0d;

        public static final double GROUND_INTAKE_HEIGHT_INCHES = 0d;
        public static final double STOW_HEIGHT_INCHES = 0d;
        public static final double HP_INTAKE_HEIGHT_INCHES = 0d;
        public static final double PROCESSOR_HEIGHT_INCHES = 0d;
        public static final double NET_HEIGHT_INCHES = 0d;

        public static final double GEARBOX_RATIO = 125d;
        public static final double SPROCKET_RATIO = 1d;
        public static final double DRIVEN_SPROCKET_DIAMETER_INCHES = 1d;
        public static final double OVERALL_GEAR_RATIO = GEARBOX_RATIO*SPROCKET_RATIO; // Motor-to-Subsystem Ratio

        public static final double ELEVATOR_TOLERANCE_ROTATIONS = 0.5;
    }

    public final class INTAKE {
        public static final double INNER_MOTOR_INTAKE_VELOCITY = 1000; // TODO: Set the velocity 
        public static final double OUTER_MOTOR_INTAKE_VELOCITY = 0;// TODO: Set the velocity 
        public static final int INTAKE_BEAM_BREAK_DIGITAL_ID = 0; // TODO: Set the ID  
        public static final double INNER_MOTOR_OUTAKE_VELOCITY = 1000; // TODO: Set the velocity
        public static final double OUTER_MOTOR_OUTAKE_VELOCITY = 0; // TODO: set the velociy
    }

    public final class CLOCK {
        public static final double GEARBOX_RATIO = 25d;
        public static final double EXTERNAL_GEAR_RATIO = 48d / 16d; // 16t gear driving a 48t gear
        public static final double OVERALL_GEAR_RATIO = GEARBOX_RATIO * EXTERNAL_GEAR_RATIO;
        public static final double WITHIN_THRESHOLD_DEGREES = 1d; // degrees
    }

    public final class WRIST {
        public static final double GEARBOX_RATIO = 125d;
        public static final double SPROCKET_RATIO = 1d;
        public static final double OVERALL_GEAR_RATIO = GEARBOX_RATIO * SPROCKET_RATIO;
        public static final double WITHIN_THRESHOLD_DEGREES = 1d; // degrees
    }

    public final class SUPERSTRUCTURE {
        public static final double FUNNEL_TOP_FROM_BASE_ELEVATOR_INCHES = 9d; // inches
        public static final double ELEVATOR_STOWED_THRESHOLD = 3d; // inches
        public static final double WRIST_LOCKED_UNDER_FUNNEL_THRESHOLD = 5d; // degrees
        public static final double CLOCK_SWING_OUT_ANGLE = 45d; // degrees
        public static final double CLOCK_STOW_ANGLE = 20d; // degrees
    }

    public final class CORAL_ALIGN {
        public static final double X_KP = 1; // Used to be 1. Currently testing x_KP of 0
        public static final double X_KI = 0;
        public static final double X_KD = 0;

        public static final double Y_KP = 0;
        public static final double Y_KI = 0;
        public static final double Y_KD = 0;

        public static final double ROT_KP = 0; // 0.015 was the original value but set to 0 for testing y axis
        public static final double ROT_KI = 0;
        public static final double ROT_KD = 0;

        public static final double X_OFFSET = 0.35;
        public static final double Y_OFFSET = 0.0;
        public static final double ROT_OFFSET = 180d;
        public static final double SPEED_SCALE = 1.0;
        public static final double SPEED_MAX = 0.5;

        // Arbitrary values for now
        public static final double LEFT_OFFSET = -2.0; 
        public static final double RIGHT_OFFSET = 2.0;
    }
}
