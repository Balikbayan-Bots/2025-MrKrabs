package frc.robot.controls;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_ROT;
import static frc.robot.subsystems.swerve.SwerveConstants.MAX_TELEOP_SPEED;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Controls {
    private static OperatorInterface oi = OperatorInterface.getInstance();
    private static CommandXboxController driver = oi.getDriver();
    private static CommandXboxController coDriver = oi.getCoDriver();

    private Controls() {
        throw new IllegalStateException("Utility class");
    }

    public final class Swerve {
        private Swerve() {
            throw new IllegalStateException("Utility class");
        }

        /*** Driver 'Left X'*/
        public static final Supplier<Double> translateX = () -> {return -driver.getLeftY() * MAX_TELEOP_SPEED;};

        /*** Driver 'Left Y'*/
        public static final Supplier<Double> translateY = () -> {return -driver.getLeftX() * MAX_TELEOP_SPEED;};

        /*** Driver 'Right X'*/
        public static final Supplier<Double> rotate = () -> {return -driver.getRightX() * MAX_TELEOP_ROT;};

        /*** Driver 'Start'*/
        public static final Trigger reorient = driver.start();

        public static final Trigger test = driver.back();
    }
}
