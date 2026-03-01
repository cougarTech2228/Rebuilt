package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;

import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber.ClimberLevel;

public class ClimberIOMotor implements ClimberIO {

    private final TalonFX climberMotor;
    private final SparkMax extensionMotor;

    private final StatusSignal<Angle> climberPositionSignal;
    private final StatusSignal<Current> climberCurrentSignal;
    private final StatusSignal<Temperature> climberTempSignal;

    SparkClosedLoopController extensionMotorPIDController;
    private final MotionMagicVoltage climberPIDController;

    private double extensionSetpoint = 0;
    private double climberSetpoint = 0;

    private boolean extensionHoming = false;
    private boolean climberHoming = false;

    private DigitalInput climberReadyDIO;

    public ClimberIOMotor() {
        climberReadyDIO = new DigitalInput(Constants.DIO_CLIMBER_READY);
        climberMotor = new TalonFX(Constants.CAN_ID_CLIMBER_MAIN,  frc.robot.RobotContainer.kCanivore);
        extensionMotor = new SparkMax(Constants.CAN_ID_CLIMBER_EXTEND, MotorType.kBrushless);

        climberPositionSignal = climberMotor.getRotorPosition();
        climberCurrentSignal = climberMotor.getStatorCurrent();
        climberTempSignal = climberMotor.getDeviceTemp();

        extensionMotorPIDController = extensionMotor.getClosedLoopController();
        SparkMaxConfig extenstionConfig = new SparkMaxConfig();
        extenstionConfig.limitSwitch.reverseLimitSwitchPosition(0);
        extenstionConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
        extenstionConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

        extenstionConfig.closedLoop
            .p(2)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .maxMotion
                .cruiseVelocity(10000)
                .maxAcceleration(40000)
                .allowedProfileError(1);
        extensionMotor.configure(extenstionConfig, 
                    com.revrobotics.ResetMode.kResetSafeParameters, 
                    com.revrobotics.PersistMode.kPersistParameters);

        climberPIDController = new MotionMagicVoltage(0);
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        climberConfig.Slot0.kP = 1;
        climberConfig.Slot0.kI = 0.0;
        climberConfig.Slot0.kD = 0.0;

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // when reverse limit is hit, zero the motor
        climberConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        climberConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        climberConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        climberConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        climberConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0;
        climberConfig.MotionMagic.MotionMagicAcceleration = 200;
        climberConfig.MotionMagic.MotionMagicJerk = 0.0;

        climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        // climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.53;
        // climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        // climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberMotor.getConfigurator().apply(climberConfig);
    }

    private boolean isExtensionHome() {
        return extensionMotor.getReverseLimitSwitch().isPressed();
    }

    private boolean isClimberHome() {
        return climberMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
    }

    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            climberPositionSignal,
            climberCurrentSignal,
            climberTempSignal
        );

        inputs.isExtensionHome = isExtensionHome();
        inputs.isClimberHome = isClimberHome();

        inputs.climberMotorPosition = climberPositionSignal.getValueAsDouble();
        inputs.climberMotorCurrent = climberCurrentSignal.getValueAsDouble();
        inputs.climberMotorTemp = climberTempSignal.getValueAsDouble();

        inputs.extendMotorPosition = extensionMotor.getEncoder().getPosition();
        inputs.extendMotorCurrent = extensionMotor.getOutputCurrent();
        inputs.extendMotorTemp = extensionMotor.getMotorTemperature();

        // if (inputs.isClimberHome) {
        //     climberMotor.setPosition(0);
        // }

        if (inputs.isExtensionHome) {
            extensionMotor.getEncoder().setPosition(0);
        }

        inputs.isClimberExtended = isExtended();
        inputs.climberMotorPIDTarget = climberSetpoint;
        inputs.extendMotorPIDTarget = extensionSetpoint;
        inputs.idClimberReady = !climberReadyDIO.get();

        if (extensionHoming && inputs.isExtensionHome) {
            extensionHoming = false;
            stopExtension();
        }

        if (climberHoming && inputs.isExtensionHome) {
            climberHoming = false;
            stopClimber();
        }
    }

    @Override
    public void extend() {
        extensionSetpoint = ClimberConstants.EXTENSION_EXTENDED_POSITION;
        System.out.println("extension setpoint: extend " + extensionSetpoint);
        extensionMotorPIDController.setSetpoint(extensionSetpoint, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void retract() {
        extensionSetpoint = ClimberConstants.EXTENSION_HOME_POSITION;
        System.out.println("extension setpoint: retract " + extensionSetpoint);
        extensionMotorPIDController.setSetpoint(extensionSetpoint, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void climb(ClimberLevel level) {
        switch (level) {
            case L1:
                climberSetpoint = ClimberConstants.CLIMBER_L1_POSITION;
                break;
            case L3:
                climberSetpoint = ClimberConstants.CLIMBER_L3_POSITION;
                break;
            default:
                climberSetpoint = 0;
                break;
            
        }
        
        System.out.println("climber setpoint: climb " + climberSetpoint);
        climberMotor.setControl(climberPIDController.withPosition(climberSetpoint));
    }

    @Override
    public void descend() {
        climberSetpoint = 0;
        System.out.println("climber setpoint: descend " + climberSetpoint);
        climberMotor.setControl(climberPIDController.withPosition(climberSetpoint));
    }

    @Override
    public boolean isExtended() {
        return extensionSetpoint == ClimberConstants.EXTENSION_EXTENDED_POSITION && extensionMotorPIDController.isAtSetpoint();
    }

    @Override
    public void homeExtension() {
        extensionHoming = true;
        if (isExtensionHome()) {
            extensionMotor.set(0);
        } else {
            extensionMotor.set(ClimberConstants.EXTENSION_HOME_SPEED);
        }
    }

    @Override
    public void homeClimber() {
        climberHoming = true;
        if (isClimberHome()) {
            climberMotor.set(0);
        } else {
            climberMotor.set(ClimberConstants.CLIMBER_HOME_SPEED);
        }
    }

    @Override
    public void stopClimber() {
        climberMotor.set(0);
    }

    public void stopExtension() {
        extensionMotor.set(0);
    }
}
