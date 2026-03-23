package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;

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
            .p(1)
            .i(0)
            .d(0);
        angleConfig.closedLoop.feedForward
            .kV(0)   // Velocity gain (Volts per Velocity Unit)
            .kA(0)   // Acceleration gain (Volts per Velocity Unit / s)
            .kG(0)    // Optional: Linear gravity gain for elevators (Volts)
            .kCos(0); // Optional: Cosine gravity gain for arms (Volts)
        angleConfig.closedLoop.maxMotion.maxAcceleration(10000);
        angleConfig.closedLoop.maxMotion.cruiseVelocity(9000);
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
        encoderConfig.MagnetSensor.MagnetOffset = IntakeConstants.INTAKE_ENCODER_OFFSET; // zero out to the retracted position
        intakeEncoder.getConfigurator().apply(encoderConfig);
        encoderPositionSignal = intakeEncoder.getAbsolutePosition();
        encoderPositionSignal.waitForUpdate(0.2);

        // seed the motor position based on the cancoder
        angleMotor.getEncoder().setPosition(encoderPositionSignal.getValueAsDouble() * 100);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        encoderPositionSignal.refresh();

        inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeMotorVelocity = intakeMotor.getEncoder().getVelocity();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        inputs.intakeMotorTemp = intakeMotor.getMotorTemperature();
        
        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition();
        inputs.angleMotorVoltage = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleMotorVelocity = angleMotor.getEncoder().getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
        inputs.angleMotorPIDSetpoint = angleMotor.getClosedLoopController().getSetpoint();

        inputs.intakeEncoder = encoderPositionSignal.getValueAsDouble();

        if (inputs.angleMotorPIDSetpoint == IntakeConstants.ANGLE_MOTOR_HOME_POSITION &&
            inputs.angleMotorCurrent > IntakeConstants.ANGLE_MOTOR_STALL_CURRENT_THRESHOLD && 
            inputs.angleMotorVoltage < IntakeConstants.ANGLE_MOTOR_STALL_VELOCITY_THRESHOLD) {
            inputs.isAngleMotorStalled = true;
        } else {
             inputs.isAngleMotorStalled = false;
        }
    }

    public void setIntakeMode(IntakeMode mode) {
        double intakeMotorVoltage = 0.0;
        switch (mode) {
            case INTAKE:
                intakeMotorVoltage = IntakeConstants.INTAKE_MOTOR_INTAKE_VOLTAGE;
                break;
            case SPIT:
                intakeMotorVoltage = IntakeConstants.INTAKE_MOTOR_SPIT_VOLTAGE;
                break;
            case IDLE:
                intakeMotorVoltage = IntakeConstants.INTAKE_MOTOR_IDLE_VOLTAGE;
                break;
        }
        intakeMotor.setVoltage(intakeMotorVoltage);
    }

    // public void bumpIntake(IntakeMode mode) {
    //     intakeMotor.setVoltage(0.0);
    //     switch (mode) {
    //         case INTAKE:
    //             break;
    //         case SPIT:
    //             break;
    //         case IDLE:
    //             break;
    //         case OSCILLATING:
    //             if (getIntakeAngle() == IntakeConstants.ANGLE_MOTOR_DEPLOYED_POSITION) {
    //                 setIntakeAngle(IntakeAngle.BUMPED);
    //                 break;
    //             } else if (getIntakeAngle() == IntakeConstants.ANGLE_MOTOR_BUMPED_POSITION) {
    //                 setIntakeAngle(IntakeAngle.DEPLOYED);
    //                 break;
    //             } else {
    //                 break;
    //             }
    //     }
    // }

    public void manualSetIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }   

    public void manualSetIntakeAngle(double angle) {
        anglePID.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void stop() {
        intakeMotor.set(0);
        angleMotor.set(0);
    }
}