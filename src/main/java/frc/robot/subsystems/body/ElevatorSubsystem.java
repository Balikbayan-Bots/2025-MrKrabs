package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.body.BodyConstants.*;

public class ElevatorSubsystem extends SubsystemBase {

    public static ElevatorSubsystem m_instance;

    public static ElevatorSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ElevatorSubsystem();
        }
        return m_instance;
    }

    private TalonFX m_leftElevator;
    private TalonFX m_rightElevator;

    public ElevatorSubsystem() {
        m_leftElevator = new TalonFX(ELEV_MOTOR_LEFT);
        m_rightElevator = new TalonFX(ELEV_MOTOR_LEFT);
        configureElev(m_leftElevator, null);
        configureElev(m_rightElevator, m_leftElevator);
        reZero();
    }

    private void configureElev(TalonFX motor, TalonFX leaderMotor) {
        TalonFXConfiguration newConfig = new TalonFXConfiguration();
        if (leaderMotor != null) {
            motor.setControl(new Follower(leaderMotor.getDeviceID(), false));
        }

    }

    public void reZero() {
        m_leftElevator.setPosition(0);
      }

}