package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
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
    
    private final SparkClosedLoopController angleController;

    public IntakeIOMotors() {
        this.intakeMotor = new SparkMax(Constants.CAN_ID_INTAKE_MOTOR, MotorType.kBrushless);
        this.angleMotor = new SparkMax(Constants.CAN_ID_INTAKE_ANGLE_MOTOR, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        SparkMaxConfig angleConfig = new SparkMaxConfig();
        angleConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        ClosedLoopConfig angleClosedLoopConfig = new ClosedLoopConfig();
        angleClosedLoopConfig.pidf(0, 0, 0, 0);
        angleClosedLoopConfig.maxMotion
            .maxAcceleration(0)
            .cruiseVelocity(100) // probably should be like 1800
            .allowedProfileError(10) 
            .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);

        angleConfig.apply(angleClosedLoopConfig);

        intakeMotor.configure(intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
        angleMotor.configure(angleConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

        this.angleController = angleMotor.getClosedLoopController();
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.angleMotorPosition = angleMotor.getEncoder().getPosition();
        inputs.angleMotorVoltage = angleMotor.getAppliedOutput() * angleMotor.getBusVoltage();
        inputs.angleMotorVelocity = angleMotor.getEncoder().getVelocity();
        inputs.angleMotorCurrent = angleMotor.getOutputCurrent();
    }

    public void setIntakeAngle(IntakeAngle angle) {
        switch (angle) {
            case HOME:
                angleController.setSetpoint(IntakeConstants.homePosition, ControlType.kMAXMotionPositionControl);
                break;
            case DEPLOYED:
                angleController.setSetpoint(IntakeConstants.deployedPosition, ControlType.kMAXMotionPositionControl);
                break;
        }
    }

    public void setIntakeMode(IntakeMode mode) {
        double intakeMotorVoltage = 0.0;
        switch (mode) {
            case INTAKE:
                intakeMotorVoltage = IntakeConstants.intakeVoltage;
                break;
            case SPIT:
                intakeMotorVoltage = IntakeConstants.spitVoltage;
                break;
            case IDLE:
                intakeMotorVoltage = IntakeConstants.idleVoltage;
                break;
        }
        intakeMotor.setVoltage(intakeMotorVoltage);
    }

    public void manualSetIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void manualSetIntakeAngle(double angle) {
        angleController.setSetpoint(angle, ControlType.kMAXMotionPositionControl);
    }
}