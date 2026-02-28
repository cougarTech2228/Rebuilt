package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOMotors implements IntakeIO {

    protected final SparkMax intakeMotor;

    protected final SparkMax angleMotor;
    private final SparkClosedLoopController anglePID;

    private final CANcoder intakeEncoder;
    StatusSignal<Angle> encoderPositionSignal;


    public IntakeIOMotors() {
        angleMotor = new SparkMax(Constants.CAN_ID_INTAKE_ANGLE_MOTOR, MotorType.kBrushless);
        anglePID = angleMotor.getClosedLoopController();

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.2)
            .i(0)
            .d(0);
        angleConfig.closedLoop.maxMotion.maxAcceleration(5000);
        angleConfig.closedLoop.maxMotion.cruiseVelocity(2500);
        angleConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
        angleConfig.closedLoop.maxMotion.allowedProfileError(1);
        angleMotor.configure(angleConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);


        intakeMotor = new SparkMax(Constants.CAN_ID_INTAKE_MOTOR, MotorType.kBrushless);
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(20);
 
        intakeMotor.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        intakeEncoder = new CANcoder(Constants.CAN_ID_INTAKE_ENCODER, frc.robot.RobotContainer.kCanivore);
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoderConfig.MagnetSensor.MagnetOffset = -0.232910; // zero out to the retracted position
        intakeEncoder.getConfigurator().apply(encoderConfig);
        encoderPositionSignal = intakeEncoder.getPosition();
        encoderPositionSignal.waitForUpdate(0.2);

        // seed the motor position based on the cancoder
        angleMotor.getEncoder().setPosition(encoderPositionSignal.getValueAsDouble() * 98.2);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            encoderPositionSignal);
        inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeMotorVelocity = intakeMotor.getEncoder().getVelocity();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        
        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition();
        inputs.angleMotorVoltage = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleMotorVelocity = angleMotor.getEncoder().getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorPIDSetpoint = angleMotor.getClosedLoopController().getSetpoint();

        inputs.intakeEncoder = encoderPositionSignal.getValueAsDouble();

    }

    public void setIntakeAngle(IntakeAngle angle) {
        switch (angle) {
            case HOME:
                anglePID.setSetpoint(IntakeConstants.homePosition, ControlType.kMAXMotionPositionControl);
                break;
            case DEPLOYED:
                anglePID.setSetpoint(IntakeConstants.deployedPosition, ControlType.kMAXMotionPositionControl);
                break;
        }
    }

    public void setIntakeMode(IntakeMode mode) {
        double intakeMotorVoltage = 0.0;
        switch (mode) {
            case INTAKE:
                setIntakeAngle(IntakeAngle.DEPLOYED);
                intakeMotorVoltage = IntakeConstants.intakeVelocity;
                break;
            case SPIT:
                setIntakeAngle(IntakeAngle.DEPLOYED);
                intakeMotorVoltage = IntakeConstants.spitVelocity;
                break;
            case IDLE:
                setIntakeAngle(IntakeAngle.HOME);
                intakeMotorVoltage = IntakeConstants.idleVelocity;
                break;
        }
        intakeMotor.setVoltage(intakeMotorVoltage);
    }

    public void manualSetIntakeVelocity(double voltage) {
        intakeMotor.setVoltage(voltage);
    }   

    public void manualSetIntakeAngle(double angle) {
        anglePID.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }
}