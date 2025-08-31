package frc.robot.subsystems.manipulators;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import static frc.robot.subsystems.manipulators.ManipulatorConstants.*;



import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClawSubsystem extends SubsystemBase{
    public static ClawSubsystem m_instance;

    public static ClawSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ClawSubsystem();
        }
        return m_instance;
    }
    private double referenceSpeed = 0;
    private ClawState state; 

    private TalonFX claw_motorFx;
    private DigitalInput beamBreak;

    public ClawSubsystem(){
        claw_motorFx = new TalonFX(CLAW_MOTOR_ID);
        beamBreak = new DigitalInput(BEAM_BREAK_ID);
         configureclaw(claw_motorFx);
    }
     private void configureclaw(TalonFX motor) {
        TalonFXConfiguration newConfig = new TalonFXConfiguration();
        var limits = newConfig.SoftwareLimitSwitch;
        limits.ForwardSoftLimitEnable = true;
        limits.ForwardSoftLimitThreshold = kclawLimits.forwardLimit();

        var current = newConfig.CurrentLimits;
        current.StatorCurrentLimit = kclawLimits.statorLimit();
        current.StatorCurrentLimitEnable = true;
        current.SupplyCurrentLimit = kclawLimits.supplyLimit();
        current.SupplyCurrentLimitEnable = true;

        var voltage = newConfig.Voltage;
        voltage.PeakForwardVoltage = 12; // Down
        voltage.PeakReverseVoltage = -12; // Up
    }
        public void setSpeed(double speed){
            referenceSpeed = speed;
    }


    public boolean getBeamBreak(){
        return beamBreak.get();
    }


    public double getSpeed(){
        return referenceSpeed;
    }

    public ClawState getState(){
        return state;
    }


    @Override
    public void periodic(){
        if(state == ClawState.HOLDING_ALGAE){
            claw_motorFx.set(algae_speed);
            return;
        } 
        claw_motorFx.set(referenceSpeed); 
        
    }
    public void setState(ClawState state){
        this.state = state;
    }

    public String getStateName(){
        return getState().name();
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("Claw");
        builder.addBooleanProperty("Beam", this::getBeamBreak, null);
        builder.addDoubleProperty("Claw Speed", this::getSpeed, null);
        builder.addStringProperty("State", this::getStateName, null);
    }





}