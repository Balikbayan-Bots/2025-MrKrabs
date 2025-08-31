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

     public void updateReferenceAngle(double travelDistance){
        referenceAngle = travelDistance;
    }
    @Override
    public void periodic(){
        updateReferenceAngle(currentSetPoint.getArmDegrees());
        m_armMotor.setControl(motionMagic.withPosition(referenceAngle*armGearRatio).withSlot(0).withFeedForward(calculateFeedForward())); 
    }
    private double calculateFeedForward(){
        return Math.sin(Math.toRadians(getencoderangel()-0))*-1;
    }

    private double getencoderangel(){
        return (m_armMotor.getPosition().getValueAsDouble()/armGearRatio)*360;
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
