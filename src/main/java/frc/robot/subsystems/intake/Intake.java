package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake {
    private final IntakeIO io;

    public enum IntakePositions {
        HOME,
        DEPLOYED
    }

    public enum IntakeMode {
        INTAKE,
        SPIT
    }

    public Intake(IntakeIO io) {
        this.io = io;
    }
}
