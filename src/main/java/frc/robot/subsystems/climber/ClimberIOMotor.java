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

    private final BaseStatusSignal[] allSignals;

    SparkClosedLoopController extensionMotorPIDController;
    private final MotionMagicVoltage climberPIDController;

    private double extensionSetpoint = 0;
    private double climberSetpoint = 0;

    private boolean extensionHoming = false;
    private boolean climberHoming = false;

    private DigitalInput climberReadyDIO;

    private boolean hasClimberHomed = false;
    private boolean hasExtensionHomed = false;

    public ClimberIOMotor() {
        climberReadyDIO = new DigitalInput(Constants.DIO_CLIMBER_READY);
        climberMotor = new TalonFX(Constants.CAN_ID_CLIMBER_MAIN,  frc.robot.RobotContainer.kCanivore);
        extensionMotor = new SparkMax(Constants.CAN_ID_CLIMBER_EXTEND, MotorType.kBrushless);

        climberPositionSignal = climberMotor.getRotorPosition();
        climberCurrentSignal = climberMotor.getStatorCurrent();
        climberTempSignal = climberMotor.getDeviceTemp();

        allSignals = new BaseStatusSignal[] {
            climberPositionSignal,
            climberCurrentSignal,
            climberTempSignal
        };

        extensionMotorPIDController = extensionMotor.getClosedLoopController();
        SparkMaxConfig extensionConfig = new SparkMaxConfig();
        extensionConfig.smartCurrentLimit(20);
        extensionConfig.limitSwitch.reverseLimitSwitchPosition(0);
        extensionConfig.limitSwitch.reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotorAndSetPosition);
        extensionConfig.limitSwitch.reverseLimitSwitchType(Type.kNormallyOpen);

        extensionConfig.closedLoop
            .p(1.0)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .maxMotion
                .cruiseVelocity(9000)
                .maxAcceleration(20000)
                .allowedProfileError(1);
        // extensionConfig.closedLoop.feedForward.kV(Constants.NEO_550_KV); // 1 / 11000 (free RPM)
        extensionMotor.configure(extensionConfig,
                    com.revrobotics.ResetMode.kResetSafeParameters, 
                    com.revrobotics.PersistMode.kPersistParameters);

        climberPIDController = new MotionMagicVoltage(0);
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        climberConfig.Slot0.kP = 1.0;
        climberConfig.Slot0.kI = 0.0;
        climberConfig.Slot0.kD = 0.0;

        climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // when reverse limit is hit, zero the motor
        climberConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
        climberConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
        climberConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
        climberConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0;
        climberConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = 90.0;
        climberConfig.MotionMagic.MotionMagicAcceleration = 200;
        climberConfig.MotionMagic.MotionMagicJerk = 0.0;

        climberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        climberMotor.getConfigurator().apply(climberConfig);
    }

    private boolean isExtensionHome() {
        return extensionMotor.getReverseLimitSwitch().isPressed();
    }

    private boolean isClimberHome() {
        return climberMotor.getReverseLimit().getValue() == ReverseLimitValue.Open;
    }

    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(allSignals);

        inputs.isExtensionHome = isExtensionHome();
        inputs.isClimberHome = isClimberHome();

        inputs.climberMotorPosition = climberPositionSignal.getValueAsDouble();
        inputs.climberMotorCurrent = climberCurrentSignal.getValueAsDouble();
        inputs.climberMotorTemp = climberTempSignal.getValueAsDouble();

        inputs.extendMotorPosition = extensionMotor.getEncoder().getPosition();
        inputs.extendMotorCurrent = extensionMotor.getOutputCurrent();
        inputs.extendMotorTemp = extensionMotor.getMotorTemperature();

        if (inputs.isExtensionHome) {
            extensionMotor.getEncoder().setPosition(0);
        }
        
        inputs.isClimberExtended = isExtended(extensionSetpoint);
        inputs.climberMotorPIDTarget = climberSetpoint;
        inputs.extendMotorPIDTarget = extensionSetpoint;
        inputs.isClimberReady = !climberReadyDIO.get();

        if (!hasExtensionHomed && inputs.isExtensionHome) {
            hasExtensionHomed = true;
        }

        if (!hasClimberHomed && inputs.isClimberHome) {
            hasClimberHomed = true;
        }
        inputs.hasClimberHomed = hasClimberHomed;
        inputs.hasExtensionHomed = hasExtensionHomed;

        if (extensionHoming && inputs.isExtensionHome) {
            extensionHoming = false;
            stopExtension();
        }

        if (climberHoming && inputs.isExtensionHome) {
            climberHoming = false;
            stopClimber();
        }

        inputs.isClimbComplete = (Math.abs(inputs.climberMotorPIDTarget - inputs.climberMotorPosition) < ClimberConstants.CLIMBER_PID_THRESHOLD);
    }

    @Override
    public void extend(Climber.ClimberLevel level) {
        switch (level) {
            case L1:
                extensionSetpoint = ClimberConstants.EXTENSION_EXTENDED_L1_POSITION;
                break;
            case L3:
                extensionSetpoint = ClimberConstants.EXTENSION_EXTENDED_L3_POSITION;
                break;
            default:
                climberSetpoint = 0;
                break;
            
        }
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
        System.out.println("climber setpoint: descend " + ClimberConstants.CLIMBER_HOME_POSITION);
        climberMotor.setControl(climberPIDController.withPosition(ClimberConstants.CLIMBER_HOME_POSITION));
    }

    @Override
    public boolean isExtended(Climber.ClimberLevel level) {
        if (!climberReadyDIO.get()) {
            return true;
        }
        double target = 0;
        switch (level) {
            case L1:
                target = ClimberConstants.EXTENSION_EXTENDED_L1_POSITION;
                break;
            case L3:
                target = ClimberConstants.EXTENSION_EXTENDED_L3_POSITION;
                break;
        }
        return extensionSetpoint > 0 &&
            (Math.abs(extensionMotor.getEncoder().getPosition() - target) < ClimberConstants.EXTENSION_PID_THRESHOLD);
    }

    // check against numerical setpoint instead of enum level
    private boolean isExtended(double setpoint) {
        return extensionSetpoint > 0 &&
            (Math.abs(extensionMotor.getEncoder().getPosition() - extensionSetpoint) < ClimberConstants.EXTENSION_PID_THRESHOLD);
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

    @Override
    public void stopExtension() {
        extensionMotor.set(0);
    }
}