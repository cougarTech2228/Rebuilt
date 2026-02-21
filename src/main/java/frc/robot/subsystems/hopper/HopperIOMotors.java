package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;

public class HopperIOMotors implements HopperIO {
    
    protected final SparkMax indexerMotor;
    private final SparkClosedLoopController indexerPID;

    protected final SparkMax kickerMotor;
    private final SparkClosedLoopController kickerPID;

    public HopperIOMotors() {
        this.indexerMotor = new SparkMax(Constants.CAN_ID_INDEXER_MOTOR, MotorType.kBrushless);
        this.indexerPID = indexerMotor.getClosedLoopController();

        this.kickerMotor = new SparkMax(Constants.CAN_ID_KICKER_MOTOR, MotorType.kBrushless);
        this.kickerPID = kickerMotor.getClosedLoopController();

        SparkMaxConfig indexerConfig = new SparkMaxConfig();
        indexerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        indexerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0);

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0);

        indexerMotor.configure(indexerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        kickerMotor.configure(indexerConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    }

    public void updateInputs(HopperIOInputs inputs) {
        inputs.indexerVoltage = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        inputs.indexerVelocity = indexerMotor.getEncoder().getVelocity();
        inputs.indexerCurrent = indexerMotor.getOutputCurrent();
        inputs.indexerCurrent = indexerMotor.getOutputCurrent();

        inputs.kickVoltage = kickerMotor.getAppliedOutput() * kickerMotor.getBusVoltage();
        inputs.kickVelocity = kickerMotor.getEncoder().getVelocity();
        inputs.kickCurrent = kickerMotor.getOutputCurrent();

    }

    public void indexerOn(boolean test) {
        if (test) {
            indexerPID.setSetpoint(HopperConstants.testIndexerVoltage, ControlType.kMAXMotionVelocityControl);
        } else {
            indexerPID.setSetpoint(HopperConstants.indexerVoltage, ControlType.kMAXMotionVelocityControl);
        }

    }

    public void indexerOff() {
        indexerPID.setSetpoint(0, ControlType.kMAXMotionVelocityControl);
    }

    public void kickerOn(boolean test) {
        if (test) {
            kickerPID.setSetpoint(HopperConstants.testKickerVoltage, ControlType.kMAXMotionVelocityControl);
        } else {
            kickerPID.setSetpoint(HopperConstants.kickerVoltage, ControlType.kMAXMotionVelocityControl);
        }
    }

    public void kickerOff() {
        kickerPID.setSetpoint(0, ControlType.kMAXMotionVelocityControl);
    }

}

