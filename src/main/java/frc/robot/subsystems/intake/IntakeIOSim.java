package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakeAngle;

public class IntakeIOSim extends IntakeIOMotors {
    
    public IntakeIOSim() {
        // Constructor
    }

    public void setIntakeAngle(IntakeAngle angle) {};

    public void setIntakeMode(IntakeMode mode) {};

    public void manualSetIntakeVoltage(double voltage) {};

    public void manualSetIntakeAngle(double angle) {};
}
