package frc.robot.subsystems.intake;

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

    public Intake(IntakeIO io) {
        // Constructor
        this.io = io;
    }

    /**
     * Sets intake mode
     * @param mode Intake, Spit, or Idle
     */
    public void setIntakeMode(IntakeMode mode) {
        io.setIntakeMode(mode);
    }

    /**
     * Sets intake angle
     * @param angle Home or Deployed
     */
    public void setIntakeAngle(IntakeAngle angle) {
        io.setIntakeAngle(angle);
    }

    public void periodic() {};
}
