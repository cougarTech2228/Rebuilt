package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
    protected final TalonFX intakeMotor = new TalonFX(Constants.intakeMotorID, "canivore");
    protected final TalonFX angleMotor = new TalonFX(Constants.intakeAngleMotorID, "canivore");

    // TODO: do
    private final StatusSignal<Voltage> intakeMotorAppliedVoltage = intakeMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> intakeMotorVelocity = intakeMotor.getVelocity();
    private final StatusSignal<Current> intakeMotorCurrent = intakeMotor.getSupplyCurrent();

    private final StatusSignal<Angle> angleMotorPos = angleMotor.getPosition();
    private final StatusSignal<Voltage> angleMotorVoltage = angleMotor.getMotorVoltage();
    private final StatusSignal<AngularVelocity> angleMotorVelocity = angleMotor.getVelocity();
    private final StatusSignal<Current> angleMotorCurrent = angleMotor.getSupplyCurrent();

    public IntakeIOTalonFX() {
        // Constructor
    }

    public void updateInputs(IntakeIOInputs inputs) {
        // TODO: do
    }

    // public void setIntakeAngle(IntakePositions position) {
    //     double angleMotorVoltage = 0.0;
    //     switch (position) {
    //         case HOME:
    //             angleMotorVoltage = IntakeConstants.
    //             break;
    //         case DEPLOYED:
    //             break;
    //     }

    public void setIntakeMode(IntakeMode mode) {
        double intakeMotorVoltage = 0.0;
        switch (mode) {
            case INTAKE:
                intakeMotorVoltage = IntakeConstants.intakeVoltage;
                break;
            case SPIT:
                intakeMotorVoltage = IntakeConstants.spitVoltage;
                break;
        }
        intakeMotor.setVoltage(intakeMotorVoltage);
    }

}



