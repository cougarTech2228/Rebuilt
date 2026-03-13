package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.Intake.IntakeMode;

public class OscillateIntakeOnce extends Command {
    
    private final Intake intake;
    private final Climber climber;

    public OscillateIntakeOnce(Intake intake, Climber climber) {
        this.intake = intake;
        this.climber = climber;
        addRequirements(intake);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (climber.isExtensionHome()) {
            if (!intake.isRetracted()) {
                intake.setIntakeAngle(IntakeAngle.DEPLOYED);
            } else {
                intake.setIntakeAngle(IntakeAngle.BUMPED);
            }
        }
    }
}
