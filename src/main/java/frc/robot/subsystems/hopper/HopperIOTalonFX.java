package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class HopperIOTalonFX implements HopperIO {
    
    protected final TalonFX indexerMotor;
    protected final TalonFX pushTurretMotor;
    protected final TalonFX extensionMotor;
    protected final DigitalInput homeSensor;

    private final StatusSignal<Voltage> indexerMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> indexerMotorVelocity;
    private final StatusSignal<Current> indexerMotorCurrentAmps;

    private final StatusSignal<Voltage> pushTurretMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> pushTurretMotorVelocity;
    private final StatusSignal<Current> pushTurretMotorCurrentAmps;

    // private final CANcoder enc19 TBD
    private final StatusSignal<Voltage> extensionMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> extensionMotorVelocity;
    private final StatusSignal<Current> extensionMotorCurrentAmps;
    private final MotionMagicVoltage extensionControl;
    
    public HopperIOTalonFX() {
        // Construct Motors + Status Signals
        this.indexerMotor = new TalonFX(Constants.indexerMotorCanID, "canivore");
        this.pushTurretMotor = new TalonFX(Constants.pushTurretMotorCanID, "canivore");
        this.extensionMotor = new TalonFX(Constants.hopperExtensionMotorCanID, "canivore");
        this.homeSensor  = new DigitalInput(Constants.homeSensorDIO);
        this.extensionControl = new MotionMagicVoltage(0);

        this.indexerMotorAppliedVoltage = indexerMotor.getMotorVoltage();
        this.indexerMotorVelocity = indexerMotor.getVelocity();
        this.indexerMotorCurrentAmps = indexerMotor.getSupplyCurrent();

        this.pushTurretMotorAppliedVoltage = pushTurretMotor.getMotorVoltage();
        this.pushTurretMotorVelocity = pushTurretMotor.getVelocity();
        this.pushTurretMotorCurrentAmps = pushTurretMotor.getSupplyCurrent();

        this.extensionMotorAppliedVoltage = extensionMotor.getMotorVoltage();
        this.extensionMotorVelocity = extensionMotor.getVelocity();
        this.extensionMotorCurrentAmps = extensionMotor.getSupplyCurrent();
        
        TalonFXConfiguration extConfig = new TalonFXConfiguration();
        
        extConfig.Slot0.kP = 0.0;
        extConfig.Slot1.kI = 0.0;
        extConfig.Slot2.kD = 0.0;

        extConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        extConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        extConfig.MotionMagic.MotionMagicJerk = 0.0;

        // Internal or external motor control?
        extConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        extConfig.Feedback.SensorToMechanismRatio = 0.0;

        extensionMotor.getConfigurator().apply(extConfig);
    }

    public void updateInputs(HopperIOInputs inputs) {
        // BaseStatusSignal.refreshAll(indexerVoltage, indexerVelocity, indexerCurrent, pushTurretVoltage, 
        // pushTurretVelocity, pushTurretMotorCurrentAmps, extensionMotorAppliedVoltage, 
        // extensionMotorVelocity, extensionMotorCurrentAmps, canHome);
, 
        inputs.indexerVoltage = indexerMotorAppliedVoltage.getValueAsDouble();
        inputs.indexerVelocity = indexerMotorVelocity.getValueAsDouble();
        inputs.indexerCurrent = indexerMotorCurrentAmps.getValueAsDouble();

        inputs.pushTurretVoltage = pushTurretMotorAppliedVoltage.getValueAsDouble();
        inputs.pushTurretVelocity = pushTurretMotorVelocity.getValueAsDouble();
        inputs.pushTurretCurrent = pushTurretMotorCurrentAmps.getValueAsDouble();

        inputs.extensionVoltage = extensionMotorAppliedVoltage.getValueAsDouble();
        inputs.extensionVelocity = extensionMotorVelocity.getValueAsDouble();
        inputs.extensionCurrent = extensionMotorCurrentAmps.getValueAsDouble();

        inputs.canHome = this.homeSensor.get();
    }

    public void indexerOn() {
        indexerMotor.setVoltage(HopperConstants.indexerVoltage);
        pushTurretMotor.setVoltage(HopperConstants.pushToTurretVoltage);

    }

    public void indexerOff() {
        indexerMotor.setVoltage(0.0);
        pushTurretMotor.setVoltage(0.0);
    }

    public void extendOut() {

    }

    public void retract() {
        
    }

    public boolean safeToRetract() {
        return !homeSensor.get();
    }
}
