package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum IntakePosition {
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

    public void setIntakeMode(IntakeMode mode) {
        io.setIntakeMode(mode);
    }

    public void setIntakePosition(IntakePosition position) {
        io.setIntakePosition(position);
    }
}
