package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.body.BodyConstants.*;

public class ArmSubsystem extends SubsystemBase {
    private static ArmSubsystem m_instance;

    public static ArmSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ArmSubsystem();
        }
        return m_instance;
    }

    private TalonFX m_armMotor;

    public ArmSubsystem(){
        m_armMotor = new TalonFX(ARM_MOTOR_ID);
        configureArm(m_armMotor.getConfigurator());
        reZero();
    }

    private void configureArm(TalonFXConfigurator motorConfig) {
        TalonFXConfiguration newConfig = new TalonFXConfiguration();

        var current = newConfig.CurrentLimits;
        current.StatorCurrentLimit = kArmLimits.statorLimit();
        current.StatorCurrentLimitEnable = true;
        current.SupplyCurrentLimit = kArmLimits.supplyLimit();
        current.SupplyCurrentLimitEnable = true;

        var voltage = newConfig.Voltage;
        voltage.PeakForwardVoltage = 13; // out
        voltage.PeakReverseVoltage = -10; // in

        Slot0Configs slot0 = newConfig.Slot0;
        slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0.kP = 60.0;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kS = 0.0;
        slot0.kG = 0.0;
        slot0.kV = 0.41;
        slot0.kA = 0.0;

        // Configuring MotionMagic
        var motionMagic = newConfig.MotionMagic;
        var output = newConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        motionMagic.MotionMagicAcceleration = 36.0;
        motionMagic.MotionMagicCruiseVelocity = 14.0;
        motorConfig.apply(newConfig);
    }

    public void reZero(){
        m_armMotor.setPosition(0);
    }
}

