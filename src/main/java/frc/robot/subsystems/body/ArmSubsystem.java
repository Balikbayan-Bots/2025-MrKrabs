package frc.robot.subsystems.body;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.util.sendable.SendableBuilder;
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
    private BodySetpoints currentSetPoint = BodySetpoints.STOW_INTAKE;
    private double referenceAngle = 0; 
    private MotionMagicVoltage motionMagic;

    public ArmSubsystem(){
        m_armMotor = new TalonFX(ARM_MOTOR_ID);
        configureArm(m_armMotor.getConfigurator());
        reZero();
        motionMagic = new MotionMagicVoltage(0).withSlot(0);
    }



    public void setSetpoint(BodySetpoints setPoint){
        currentSetPoint = setPoint;
        updateReferenceAngle(currentSetPoint.getArmDegrees());
    }

    private void configureArm(TalonFXConfigurator motorConfig) {
        TalonFXConfiguration newConfig = new TalonFXConfiguration();

        var current = newConfig.CurrentLimits;
        current.StatorCurrentLimit = kArmLimits.statorLimit();
        current.StatorCurrentLimitEnable = true;
        current.SupplyCurrentLimit = kArmLimits.supplyLimit();
        current.SupplyCurrentLimitEnable = true;

        var voltage = newConfig.Voltage;
        voltage.PeakForwardVoltage = ARM_MAX_VOLTAGE_FWD; // out
        voltage.PeakReverseVoltage = ARM_MAX_VOLTAGE_REVERSE; // in

        Slot0Configs slot0 = newConfig.Slot0;
        slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        slot0.kP = ARM_SLOT_ZERO[0];
        slot0.kI = ARM_SLOT_ZERO[1];
        slot0.kD = ARM_SLOT_ZERO[2];
        slot0.kS = ARM_SLOT_ZERO[3];
        slot0.kG = ARM_SLOT_ZERO[4];
        slot0.kV = ARM_SLOT_ZERO[5];
        slot0.kA = ARM_SLOT_ZERO[6];

        // Configuring MotionMagic
        var motionMagic = newConfig.MotionMagic;
        var output = newConfig.MotorOutput;
        output.NeutralMode = NeutralModeValue.Brake;
        motionMagic.MotionMagicAcceleration = ARM_MOTION_MAGIC_CONFIGS[0];
        motionMagic.MotionMagicCruiseVelocity = ARM_MOTION_MAGIC_CONFIGS[1];
        motionMagic.MotionMagicJerk = ARM_MOTION_MAGIC_CONFIGS[2];
        motorConfig.apply(newConfig);
    }

    public void reZero(){
        m_armMotor.setPosition(0);
    }

     public void updateReferenceAngle(double travelDistance){
        referenceAngle = travelDistance;
    }
    @Override
    public void periodic(){
        updateReferenceAngle(currentSetPoint.getArmDegrees());
        m_armMotor.setControl(motionMagic.withPosition(referenceAngle*ARM_GEAR_RATIO).withSlot(0).withFeedForward(calculateFeedForward())); 
    }
    private double calculateFeedForward(){
        return Math.sin(Math.toRadians(getencoderangel()-0))*-1;
    }

    private double getencoderangel(){
        return (m_armMotor.getPosition().getValueAsDouble()/ARM_GEAR_RATIO)*360;
    }

    public double getReferenceAngle(){
        return referenceAngle;
    }
    public double getOffsetAngle(){
        return getReferenceAngle() - getencoderangel();
    }

     @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Arm");
        builder.addDoubleProperty("Arm position", this::getencoderangel, null);
        builder.addDoubleProperty("Commanded Arm position", this::getReferenceAngle, null);
        builder.addDoubleProperty("Arm offset", this::getOffsetAngle, null);
        builder.addDoubleProperty("Arm Feed Forward", this::calculateFeedForward, null);
    }


}
