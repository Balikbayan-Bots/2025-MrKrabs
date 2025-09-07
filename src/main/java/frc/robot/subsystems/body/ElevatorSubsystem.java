package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import static frc.robot.subsystems.body.BodyConstants.*;

import edu.wpi.first.util.sendable.SendableBuilder;
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
    voltage.PeakForwardVoltage = ELEV_MAX_VOLTAGE_FWD; // Down
    voltage.PeakReverseVoltage = ELEV_MAX_VOLATGE_REVERSE; // Up

    Slot0Configs slot0 = newConfig.Slot0;
    slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
    slot0.kP = ELEV_SLOT_ZERO[0];
    slot0.kI = ELEV_SLOT_ZERO[1];
    slot0.kD = ELEV_SLOT_ZERO[2];
    slot0.kS = ELEV_SLOT_ZERO[3];
    slot0.kG = ELEV_SLOT_ZERO[4];
    slot0.kV = ELEV_SLOT_ZERO[5];
    slot0.kA = ELEV_SLOT_ZERO[6];

    // Configuring MotionMagic
    var motionMagic = newConfig.MotionMagic;
    var output = newConfig.MotorOutput;
    output.NeutralMode = NeutralModeValue.Brake;
    motionMagic.MotionMagicAcceleration = ELEV_MOTION_MAGIC_CONFIGS[0];
    motionMagic.MotionMagicCruiseVelocity = ELEV_MOTION_MAGIC_CONFIGS[1];
    motionMagic.MotionMagicJerk = ELEV_MOTION_MAGIC_CONFIGS[2];
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

    public double getencoderangel(){
        return (m_leftElevator.getPosition().getValueAsDouble()/ELEV_GEAR_RATIO);
    }
    
    public double getReferenceTravel(){
        return referenceTravel;
    }

    public double getOffset(){
        return getReferenceTravel() - getencoderangel();
    }

    @Override
    public void periodic(){
        updateReferenceTravel(currentSetPoint.getElevTravel());
        m_leftElevator.setControl(motionMagic.withPosition(referenceTravel*ELEV_GEAR_RATIO).withSlot(0).withFeedForward(0)); 
    }



       @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Elevator");
        builder.addDoubleProperty("Elevator position", this::getencoderangel, null);
        builder.addDoubleProperty("Commanded Elevator position", this::getReferenceTravel, null);
        builder.addDoubleProperty("Elevator offset", this::getOffset, null);
    }
}
