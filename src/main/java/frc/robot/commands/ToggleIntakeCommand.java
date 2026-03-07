package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

public class ToggleIntakeCommand extends Command {
    private final Intake intake;
    private final Climber climber;
    private boolean toggeled;

    public ToggleIntakeCommand(Intake intake, Climber climber) {
        this.intake = intake;
        this.climber = climber;
        // only require intake, climber is only used for reading status
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        toggeled = false;
    }

    @Override
    public void execute() {
        // if the climber is extender, just ignore this request, and finish the command
        if (climber.isExtensionHome()) {
            intake.toggleIntake();
        } 
        toggeled = true;
    }

    @Override
    public boolean isFinished() {
        return toggeled;
    }
}
