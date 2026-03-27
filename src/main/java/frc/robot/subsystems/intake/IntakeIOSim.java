package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.Intake.IntakeMode;

public class IntakeIOSim implements IntakeIO {
    
    private static final double GEAR_RATIO = 49.0;
    
    private final SingleJointedArmSim angleSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeo550(1), 0.0001, GEAR_RATIO),
        DCMotor.getNeo550(1),
        GEAR_RATIO,
        0.3,
        Units.degreesToRadians(-10),
        Units.degreesToRadians(150),
        true,
        0.0
    );

    private final ProfiledPIDController angleFeedback = new ProfiledPIDController(
        1, 0.0, 0.0,
        new TrapezoidProfile.Constraints(
            9000.0 / 60.0,
            10000.0 / 60.0
        )
    );

    private double angleSetpointUnits = IntakeConstants.ANGLE_MOTOR_HOME_POSITION;
    private double intakeAppliedVolts = 0.0;

    public IntakeIOSim() {
        angleFeedback.reset(0);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        angleSim.update(0.020);

        double currentMotorRotations = (angleSim.getAngleRads() / (2 * Math.PI)) * GEAR_RATIO;
        
        double angleAppliedVolts = MathUtil.clamp(
            angleFeedback.calculate(currentMotorRotations, angleSetpointUnits),
            -12.0, 12.0
        );
        angleSim.setInput(angleAppliedVolts);

        inputs.angleMotorPosition = currentMotorRotations;
        inputs.angleMotorVelocity = (angleSim.getVelocityRadPerSec() / (2 * Math.PI)) * GEAR_RATIO * 60.0;
        inputs.angleMotorVoltage = angleAppliedVolts;
        inputs.angleMotorCurrent = angleSim.getCurrentDrawAmps();
        inputs.angleMotorPIDSetpoint = angleSetpointUnits;

        inputs.intakeEncoder = angleSim.getAngleRads() / (2 * Math.PI);

        inputs.intakeMotorVoltage = intakeAppliedVolts;
        inputs.intakeMotorVelocity = (intakeAppliedVolts / 12.0) * 5676.0;
    }

    @Override
    public void manualSetIntakeAngle(double angle) {
        this.angleSetpointUnits = angle;
    }

    @Override
    public void setIntakeMode(IntakeMode mode) {
        switch (mode) {
            case INTAKE:
                manualSetIntakeVoltage(IntakeConstants.INTAKE_MOTOR_INTAKE_VOLTAGE);
                break;
            case SPIT:
                manualSetIntakeVoltage(IntakeConstants.INTAKE_MOTOR_SPIT_VOLTAGE);
                break;
            case IDLE:
                manualSetIntakeVoltage(IntakeConstants.INTAKE_MOTOR_IDLE_VOLTAGE);
                break;
        }
    }

    // @Override
    // public void bumpIntake(IntakeMode mode) {
    //     manualSetIntakeVelocity(0.0);
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

    @Override
    public void manualSetIntakeVoltage(double voltage) {
        this.intakeAppliedVolts = voltage;
    }

    @Override
    public void stop() {
        this.intakeAppliedVolts = 0.0;
        this.angleSetpointUnits = (angleSim.getAngleRads() / (2 * Math.PI)) * GEAR_RATIO;
    }
}