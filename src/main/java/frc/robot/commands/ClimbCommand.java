package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;

public class ClimbCommand extends Command {
    private final Climber climber;
    private final Climber.ClimberLevel level;
    private boolean climbCommandSent = false;

    public ClimbCommand(Climber climber, Climber.ClimberLevel level) {
        this.climber = climber;
        this.level = level;
    }

    @Override
    public void initialize() {
        climbCommandSent = false;
    }

    @Override
    public void execute() {
        if (climber.isSafeToClimb(level) && !climbCommandSent) {
            climber.climb(level);
            climbCommandSent = true;
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isClimbComplete();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.stop();
        }
    }
}
