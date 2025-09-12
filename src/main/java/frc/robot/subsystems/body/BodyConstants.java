package frc.robot.subsystems.body;

public class BodyConstants {
    public static final int ELEV_MOTOR_LEFT = 20;
    public static final int ELEV_MOTOR_RIGHT = 21;
    public static final int ELEV_MAGNET_ID = -1;

    public static final int ARM_MOTOR_ID = 30;
    public static final int ARM_MAGNET_ID = -1;

    

    public static final double ARM_GEAR_RATIO = 32D/1D;

    public static final double ELEV_GEAR_RATIO = 5D/1D;

    public static final double ARM_MAX_VOLTAGE_FWD = 10;
    public static final double ARM_MAX_VOLTAGE_REVERSE = -10;

    public static final double ELEV_MAX_VOLTAGE_FWD = 10;
    public static final double ELEV_MAX_VOLATGE_REVERSE = -10;

    public static final double[] ELEV_SLOT_ZERO = {
        1.0, //kP
        0.0, //kI
        0.0, //kD
        0.0, //kS
        0.0, //kG
        0.1, //kV
        0.0 //kA
    };

    public static final double[] ARM_SLOT_ZERO = {
        1.0, //kP
        0.0, //kI
        0.0, //kD
        0.0, //kS
        0.0, //kG
        0.1, //kV
        0.0 //kA
    };

    public static final double[] ELEV_MOTION_MAGIC_CONFIGS = {
        80.0, //Acceleration
        25.0, //Cruise Velocity
        1600.0 //Jerk
    };

    public static final double[] ARM_MOTION_MAGIC_CONFIGS = {
        36.0, //Acceleration
        14.0, //Cruise Velocity
        1600.0 //Jerk
    };

    public static final Limits kArmLimits = new Limits(90.0, 45.0, 0.0, 0);

    public static final Limits kElevLimits = new Limits(70.0, 45.0,0.0, 0.0);

    public record Limits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) { };






}
