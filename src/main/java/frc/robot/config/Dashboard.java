package frc.robot.config;

public class Dashboard {
    private Dashboard() {
        throw new IllegalStateException("Utility class");
    }
    
    //*** Show all swerve related telemetry **/
    public static final boolean SHOW_SWERVE = true;

    //*** Show all debug/misc information **/
    public static final boolean SHOW_DEBUG = true;
}
