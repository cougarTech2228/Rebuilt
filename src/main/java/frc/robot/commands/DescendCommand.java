package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class DescendCommand extends Command {
    private final Climber climber;

    public DescendCommand (Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        climber.descend();
    }

    @Override
    public boolean isFinished() {
        return climber.isClimberHome();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.stop();
        }
    }
}
