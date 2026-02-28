package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase{
    private final IntakeIO io;
    
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();

    public enum IntakeAngle {
        HOME,
        DEPLOYED
    }

    public enum IntakeMode {
        INTAKE,
        SPIT,
        IDLE
    }

    private IntakeMode currentIntakeMode = IntakeMode.IDLE;

    public Intake(IntakeIO io) {
        // Constructor
        this.io = io;

        SmartDashboard.putNumber("IntakePosition", 0.0);
        SmartDashboard.putNumber("IntakeVelocity", 0.0);
    }

    public void setIntakeAngle(double angle) {
        io.manualSetIntakeAngle(angle);
    }

    public void setIntakeVelocity(double rpm) {
        io.manualSetIntakeVelocity(rpm);
    }
    
    /**
     * Sets intake mode
     * @param mode Intake, Spit, or Idle
     */
    public void setIntakeMode(IntakeMode mode) {
        currentIntakeMode = mode;
        io.setIntakeMode(mode);
    }

    public void toggleIntake() {
        System.out.println("toggleIntake()");
        if (currentIntakeMode == IntakeMode.IDLE) {
            System.out.println("setIntakeMode(IntakeMode.INTAKE)");
            setIntakeMode(IntakeMode.INTAKE);
        } else {
            System.out.println("setIntakeMode(IntakeMode.IDLE)");
            setIntakeMode(IntakeMode.IDLE);
        }
    }

    /**
     * Sets intake angle
     * @param angle Home or Deployed
     */
    public void setIntakeAngle(IntakeAngle angle) {
        System.out.println("setIntakeAngle");
        io.setIntakeAngle(angle);
    }

    public void periodic() {
        io.updateInputs(intakeInputs);
        Logger.processInputs("Intake", intakeInputs);
    };
}
