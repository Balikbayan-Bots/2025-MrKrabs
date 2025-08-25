package frc.robot.subsystems.body;

public class BodyConstants {
    public static final int ELEV_MOTOR_LEFT = -1;
    public static final int ELEV_MOTOR_RIGHT = -1;
    public static final int ELEV_MAGNET_ID = -1;

    public static final int ARM_MOTOR_ID = -1;
    public static final int ARM_MAGNET_ID = -1;

    public static double armGearRatio = -1D;

    public static double elevGearRatio = -1D;

    public static final Limits kArmLimits = new Limits(90.0, 45.0, 60D / 360D, 0);

    public static final Limits kElevLimits = new Limits(70.0, 45.0, -0.02, 60D / 360D);

    public record Limits(double statorLimit, double supplyLimit, double forwardLimit, double reverseLimit) { };






}
