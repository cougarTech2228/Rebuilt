package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOMotors implements IntakeIO {

    protected final SparkMax intakeMotor;
    private final SparkClosedLoopController intakePID;

    protected final SparkMax angleMotor;
    private final SparkClosedLoopController anglePID;

    public IntakeIOMotors() {
        this.intakeMotor = new SparkMax(Constants.CAN_ID_INTAKE_MOTOR, MotorType.kBrushless);
        this.intakePID = intakeMotor.getClosedLoopController();

        this.angleMotor = new SparkMax(Constants.CAN_ID_INTAKE_ANGLE_MOTOR, MotorType.kBrushless);
        this.anglePID = angleMotor.getClosedLoopController();

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        angleConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0)
            .i(0)
            .d(0);

        intakeMotor.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        angleMotor.configure(angleConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeMotorVoltage = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
        inputs.intakeMotorVelocity = intakeMotor.getEncoder().getVelocity();
        inputs.intakeMotorCurrent = intakeMotor.getOutputCurrent();
        
        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition();
        inputs.angleMotorVoltage = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleMotorVelocity = angleMotor.getEncoder().getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
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
        double intakeMotorVelocity = 0.0;
        switch (mode) {
            case INTAKE:
                intakeMotorVelocity = IntakeConstants.intakeVelocity;
                break;
            case SPIT:
                intakeMotorVelocity = IntakeConstants.spitVelocity;
                break;
            case IDLE:
                intakeMotorVelocity = IntakeConstants.idleVelocity;
                break;
        }
        intakePID.setSetpoint(intakeMotorVelocity, ControlType.kMAXMotionVelocityControl);
    }

    public void manualSetIntakeVelocity(double velocity) {
        intakePID.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
    }   

    public void manualSetIntakeAngle(double angle) {
        anglePID.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }
}