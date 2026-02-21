package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ClimberIOMotor implements ClimberIO {

    private final TalonFX climberMotor;
    private final SparkMax extensionMotor;
    private final DigitalInput climberHomeSensor;
    private final DigitalInput extensionHomeSensor;

    private final StatusSignal<Angle> climberPositionSignal;
    private final StatusSignal<Current> climberCurrentSignal;
    private final StatusSignal<Temperature> climberTempSignal;

    SparkClosedLoopController extensionMotorPIDController;
    private final MotionMagicVoltage climberPIDController;

    private double extensionSetpoint = 0;
    private double climberSetpoint = 0;

    public ClimberIOMotor() {
        climberMotor = new TalonFX(Constants.CAN_ID_CLIMBER_MAIN,  frc.robot.RobotContainer.kCanivore);
        extensionMotor = new SparkMax(Constants.CAN_ID_CLIMBER_EXTEND, MotorType.kBrushless);
        climberHomeSensor = new DigitalInput(Constants.DIO_CLIMBER_HOME_SENSOR);
        extensionHomeSensor = new DigitalInput(Constants.DIO_CLIMBER_EXTENSION_HOME_SENSOR);

        climberPositionSignal = climberMotor.getPosition();
        climberCurrentSignal = climberMotor.getStatorCurrent();
        climberTempSignal = climberMotor.getDeviceTemp();

        extensionMotorPIDController = extensionMotor.getClosedLoopController();
        SparkMaxConfig config = new SparkMaxConfig();
        // To set Counter-Clockwise (CCW) as positive (Default):
        // config.inverted(false);

        // To set Clockwise (CW) as positive:
        // config.inverted(true);

        config.closedLoop
            .p(0.1)
            .i(0)
            .d(0)
            .outputRange(-1, 1)
            .maxMotion
                .cruiseVelocity(5000)
                .maxAcceleration(10000)
                .allowedProfileError(1);
        extensionMotor.configure(config, 
                    com.revrobotics.ResetMode.kResetSafeParameters, 
                    com.revrobotics.PersistMode.kPersistParameters);

        climberPIDController = new MotionMagicVoltage(0);
        TalonFXConfiguration climberConfig = new TalonFXConfiguration();

        climberConfig.Slot0.kP = 0.1;
        climberConfig.Slot0.kI = 0.0;
        climberConfig.Slot0.kD = 0.0;

        climberConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;
        climberConfig.MotionMagic.MotionMagicAcceleration = 3;
        climberConfig.MotionMagic.MotionMagicJerk = 0.0;

        // climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.53;
        climberConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
        climberConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climberMotor.getConfigurator().apply(climberConfig);
    }

    private boolean isExtensionHome() {
        return !extensionHomeSensor.get();
    }
    private boolean isClimberHome() {
        return !climberHomeSensor.get();
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

        inputs.extendMotorPosition = extensionMotor.getAbsoluteEncoder().getPosition();
        inputs.extendMotorCurrent = extensionMotor.getOutputCurrent();
        inputs.extendMotorTemp = extensionMotor.getMotorTemperature();

        if (inputs.isClimberHome) {
            climberMotor.setPosition(0);
        }

        if (inputs.isExtensionHome) {
            extensionMotor.getEncoder().setPosition(0);
        }

        inputs.isClimberExtended = isExtended();
        inputs.climberMotorPIDTarget = climberSetpoint;
        inputs.extendMotorPIDTarget = extensionSetpoint;
    }

    @Override
    public void extend() {
        extensionSetpoint = ClimberConstants.EXTENSION_MAX_POSITION;
        extensionMotorPIDController.setSetpoint(extensionSetpoint, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void retract() {
        extensionSetpoint = 0;
        extensionMotorPIDController.setSetpoint(extensionSetpoint, SparkMax.ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void climb() {
        climberSetpoint = ClimberConstants.EXTENSION_MAX_POSITION;
        climberMotor.setControl(climberPIDController.withPosition(climberSetpoint));
    }

    @Override
    public void descend() {
        climberSetpoint = 0;
        climberMotor.setControl(climberPIDController.withPosition(climberSetpoint));
    }

    @Override
    public boolean isExtended() {
        return extensionSetpoint == ClimberConstants.EXTENSION_MAX_POSITION && extensionMotorPIDController.isAtSetpoint();
    }

    @Override
    public void homeExtension() {
        if (isExtensionHome()) {
            extensionMotor.set(0);
        } else {
            extensionMotor.set(ClimberConstants.EXTENSION_HOME_SPEED);
        }
    }

    @Override
    public void homeClimber() {
        if (isClimberHome()) {
            climberMotor.set(0);
        } else {
            climberMotor.set(ClimberConstants.CLIMBER_HOME_SPEED);
        }
    }
}
