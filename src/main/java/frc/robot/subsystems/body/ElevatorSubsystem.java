package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import static frc.robot.subsystems.body.BodyConstants.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorSubsystem extends SubsystemBase {

    public static ElevatorSubsystem m_instance;

    public static ElevatorSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ElevatorSubsystem();
        }
        return m_instance;
    }
    private BodySetpoints currentSetPoint = BodySetpoints.STOW_INTAKE;
    private double referenceTravel = 0; 
    private TalonFX m_leftElevator;
    private TalonFX m_rightElevator;
    private MotionMagicVoltage motionMagic;


    public ElevatorSubsystem() {
        m_leftElevator = new TalonFX(ELEV_MOTOR_LEFT);
        m_rightElevator = new TalonFX(ELEV_MOTOR_LEFT);
        configureElev(m_leftElevator, null);
        configureElev(m_rightElevator, m_leftElevator);
        reZero();
        motionMagic = new MotionMagicVoltage(0).withSlot(0);
        
    }

    private void configureElev(TalonFX motor, TalonFX leaderMotor) {
        TalonFXConfiguration newConfig = new TalonFXConfiguration();
        if (leaderMotor != null) {
            motor.setControl(new Follower(leaderMotor.getDeviceID(), false));
        }
          var limits = newConfig.SoftwareLimitSwitch;
    limits.ForwardSoftLimitEnable = true;
    limits.ForwardSoftLimitThreshold = kElevLimits.forwardLimit();

    var current = newConfig.CurrentLimits;
    current.StatorCurrentLimit = kElevLimits.statorLimit();
    current.StatorCurrentLimitEnable = true;
    current.SupplyCurrentLimit = kElevLimits.supplyLimit();
    current.SupplyCurrentLimitEnable = true;

    var voltage = newConfig.Voltage;
    voltage.PeakForwardVoltage = 12; // Down
    voltage.PeakReverseVoltage = -12; // Up

    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = 0.0;
    slot0.kI = 0.0;
    slot0.kD = 0.0;
    slot0.kS = 0.0;
    slot0.kG = 0.0;
    slot0.kV = 0.0;

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    var output = newConfig.MotorOutput;
    output.NeutralMode = NeutralModeValue.Brake;
    motionMagic.MotionMagicAcceleration = 80.0;
    motionMagic.MotionMagicCruiseVelocity = 25.0;
    motionMagic.MotionMagicJerk = 1600.0;
    motor.getConfigurator().apply(newConfig);
    }

    public void setSetpoint(BodySetpoints setPoint){
        currentSetPoint = setPoint;
        updateReferenceTravel(currentSetPoint.getElevTravel());
    }

    public void updateReferenceTravel(double travelDistance){
        m_rightElevator.setControl(new Follower(m_leftElevator.getDeviceID(), false));
        referenceTravel = travelDistance;
    }

    public void reZero() {
        m_leftElevator.setPosition(0);
      }

    @Override
    public void periodic(){
        updateReferenceTravel(currentSetPoint.getElevTravel());
        m_leftElevator.setControl(motionMagic.withPosition(referenceTravel*elevGearRatio).withSlot(0).withFeedForward(0)); 
    }
    
}
