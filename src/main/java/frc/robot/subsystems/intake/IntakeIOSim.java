package frc.robot.subsystems.intake;

import frc.robot.subsystems.intake.Intake.IntakeMode;
import frc.robot.subsystems.intake.Intake.IntakePosition;

public class IntakeIOSim extends IntakeIOTalonFX {
    
    public IntakeIOSim() {
        // Constructor
    }

    public void setIntakePosition(IntakePosition position) {};

    public void setIntakeMode(IntakeMode mode) {};

    public void setIntakeVoltage(double voltage) {};

    public void setDeployVoltage(double voltage) {};
}
