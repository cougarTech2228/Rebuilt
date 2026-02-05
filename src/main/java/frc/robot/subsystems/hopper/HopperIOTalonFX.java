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
    protected final TalonFX kickerMotor;
    protected final TalonFX extensionMotor;
    protected final DigitalInput homeSensor;

    private final StatusSignal<Voltage> indexerMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> indexerMotorVelocity;
    private final StatusSignal<Current> indexerMotorCurrentAmps;

    private final StatusSignal<Voltage> kickerMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> kickerMotorVelocity;
    private final StatusSignal<Current> kickerMotorCurrentAmps;

    // private final CANcoder enc19 TBD
    private final StatusSignal<Voltage> extensionMotorAppliedVoltage;
    private final StatusSignal<AngularVelocity> extensionMotorVelocity;
    private final StatusSignal<Current> extensionMotorCurrentAmps;
    private final MotionMagicVoltage extensionControl;
    
    public HopperIOTalonFX() {
        // Construct Motors + Status Signals
        this.indexerMotor = new TalonFX(Constants.CAN_ID_INDEXER_MOTOR, frc.robot.RobotContainer.kCanivore);
        this.kickerMotor = new TalonFX(Constants.CAN_ID_INDEXER_MOTOR, frc.robot.RobotContainer.kCanivore);
        this.extensionMotor = new TalonFX(Constants.hopperExtensionMotorCanID, frc.robot.RobotContainer.kCanivore);
        this.homeSensor  = new DigitalInput(Constants.homeSensorDIO);
        this.extensionControl = new MotionMagicVoltage(0);

        this.indexerMotorAppliedVoltage = indexerMotor.getMotorVoltage();
        this.indexerMotorVelocity = indexerMotor.getVelocity();
        this.indexerMotorCurrentAmps = indexerMotor.getSupplyCurrent();

        this.kickerMotorAppliedVoltage = kickerMotor.getMotorVoltage();
        this.kickerMotorVelocity = kickerMotor.getVelocity();
        this.kickerMotorCurrentAmps = kickerMotor.getSupplyCurrent();

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
        // BaseStatusSignal.refreshAll(indexerVoltage, indexerVelocity, indexerCurrent, kickVoltage, 
        // kickVelocity, kickerMotorCurrentAmps, extensionMotorAppliedVoltage, 
        // extensionMotorVelocity, extensionMotorCurrentAmps, canHome);
        
        inputs.indexerVoltage = indexerMotorAppliedVoltage.getValueAsDouble();
        inputs.indexerVelocity = indexerMotorVelocity.getValueAsDouble();
        inputs.indexerCurrent = indexerMotorCurrentAmps.getValueAsDouble();

        inputs.kickVoltage = kickerMotorAppliedVoltage.getValueAsDouble();
        inputs.kickVelocity = kickerMotorVelocity.getValueAsDouble();
        inputs.kickCurrent = kickerMotorCurrentAmps.getValueAsDouble();

        inputs.extensionVoltage = extensionMotorAppliedVoltage.getValueAsDouble();
        inputs.extensionVelocity = extensionMotorVelocity.getValueAsDouble();
        inputs.extensionCurrent = extensionMotorCurrentAmps.getValueAsDouble();

        inputs.canHome = this.homeSensor.get();
    }

    public void indexerOn() {
        indexerMotor.setVoltage(HopperConstants.indexerVoltage);
        kickerMotor.setVoltage(HopperConstants.kickerVoltage);

    }

    public void indexerOff() {
        indexerMotor.setVoltage(0.0);
        kickerMotor.setVoltage(0.0);
    }

    public void extendOut() {

    }

    public void retract() {
        
    }

    public boolean safeToRetract() {
        return !homeSensor.get();
    }
}

