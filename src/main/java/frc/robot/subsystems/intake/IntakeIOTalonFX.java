package frc.robot.subsystems.intake;

import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakePosition;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX intakeMotor = new TalonFX(Constants.intakeMotorID, "canivore");
    protected final TalonFX deployMotor = new TalonFX(Constants.intakeAngleMotorID, "canivore");
    protected final DigitalInput homeBeamSensor = new DigitalInput(Constants.IntakeDeploySensorDIO);

    private final StatusSignal<Voltage> intakeMotorVoltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> intakeMotorVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Current> intakeMotorCurrent = intakeMotor.getSupplyCurrent();

    private final StatusSignal<Angle> deployMotorPosition = deployMotor.getPosition();
    private final StatusSignal<Voltage> deployMotorVoltage = deployMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> deployMotorVelocity = deployMotor.getVelocity();
    private final StatusSignal<Current> deployMotorCurrent = deployMotor.getSupplyCurrent();

    public IntakeIOTalonFX() {
        // Constructor
    }

    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(intakeMotorVoltage, intakeMotorVelocity, intakeMotorCurrent, deployMotorPosition, deployMotorVoltage, deployMotorVelocity, deployMotorCurrent);

        inputs.intakeMotorVoltage = intakeMotorVoltage.getValueAsDouble();
        inputs.intakeMotorVelocity = intakeMotorVelocity.getValueAsDouble();
        inputs.intakeMotorCurrent = intakeMotorCurrent.getValueAsDouble();

        inputs.deployMotorPosition = deployMotorPosition.getValueAsDouble();
        inputs.deployMotorVoltage = deployMotorVoltage.getValueAsDouble();
        inputs.deployMotorVelocity = deployMotorVelocity.getValueAsDouble();
        inputs.deployMotorCurrent = deployMotorCurrent.getValueAsDouble();

        inputs.atHome = homeBeamSensor.get();

    }

    public void setIntakePosition(IntakePosition position) {
        double deployMotorVoltage = 0.0;
        switch (position) {
            case HOME:
                deployMotorVoltage = IntakeConstants.homeVoltage;
                break;
            case DEPLOYED:
                deployMotorVoltage = IntakeConstants.deployVoltage;
                break;
        }
        deployMotor.setVoltage(deployMotorVoltage);
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
    public void setIntakeVoltage(double voltage) {
        intakeMotor.setVoltage(voltage);
    }
    public void setDeployVoltage(double voltage) {
        deployMotor.setVoltage(voltage);
    }

}



