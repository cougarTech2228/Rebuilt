package frc.robot.subsystems.intake;

import java.util.InputMismatchException;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX intakeMotor;
    protected final TalonFX angleMotor;
    private final MotionMagicVoltage angleControl;
    protected final DigitalInput homeDigitalInput;

    // Status Signals
    private final StatusSignal<Voltage> intakeMotorVoltage;
    private final StatusSignal<AngularVelocity> intakeMotorVelocity;
    private final StatusSignal<Current> intakeMotorCurrent;

    private final StatusSignal<Angle> angleMotorPosition;
    private final StatusSignal<Voltage> angleMotorVoltage;
    private final StatusSignal<AngularVelocity> angleMotorVelocity;
    private final StatusSignal<Current> angleMotorCurrent;

    public IntakeIOTalonFX() {
        // Constructor
        this.intakeMotor = new TalonFX(Constants.intakeMotorID, "canivore");
        this.angleMotor = new TalonFX(Constants.intakeAngleMotorID, "canivore");
        this.homeDigitalInput = new DigitalInput(Constants.IntakeDeploySensorDIO);

        // Digital Inputs
        intakeMotorVoltage = intakeMotor.getMotorVoltage();
        intakeMotorVelocity = intakeMotor.getVelocity();
        intakeMotorCurrent = intakeMotor.getSupplyCurrent();
        angleMotorPosition = angleMotor.getPosition();
        angleMotorVoltage = angleMotor.getMotorVoltage();
        angleMotorVelocity = angleMotor.getVelocity();
        angleMotorCurrent = angleMotor.getSupplyCurrent();
        
        this.angleControl = new MotionMagicVoltage(0);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;

        config.MotionMagic.MotionMagicCruiseVelocity = 0;
        config.MotionMagic.MotionMagicAcceleration = 0;
        config.MotionMagic.MotionMagicJerk = 0;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = 1;

        angleMotor.getConfigurator().apply(config);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(intakeMotorVoltage, intakeMotorVelocity, intakeMotorCurrent, angleMotorPosition, 
        angleMotorVoltage, angleMotorVelocity, angleMotorCurrent);

        inputs.intakeMotorVoltage = intakeMotorVoltage.getValueAsDouble();
        inputs.intakeMotorVelocity = intakeMotorVelocity.getValueAsDouble();
        inputs.intakeMotorCurrent = intakeMotorCurrent.getValueAsDouble();

        inputs.angleMotorPosition = angleMotorPosition.getValueAsDouble();
        inputs.angleMotorVoltage = angleMotorVoltage.getValueAsDouble();
        inputs.angleMotorVelocity = angleMotorVelocity.getValueAsDouble();
        inputs.angleMotorCurrent = angleMotorCurrent.getValueAsDouble();

        inputs.atHome = homeDigitalInput.get();
        if (inputs.atHome) {
            angleMotor.setPosition(0);
        }

    }

    public void setIntakeAngle(IntakeAngle angle) {
        switch (angle) {
            case HOME:
                angleMotor.setControl(angleControl.withPosition(IntakeConstants.homePosition));
                break;
            case DEPLOYED:
                angleMotor.setControl(angleControl.withPosition(IntakeConstants.deployedPosition));
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
                intakeMotorVoltage = IntakeConstants.idolVoltage;
                break;
        }
        intakeMotor.setVoltage(intakeMotorVoltage);
    }

    // Manual methods
    public void manualSetIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }

    public void manualSetIntakeAngle(double angle) {
        angleMotor.setControl(angleControl.withPosition(angle));
    }

}



