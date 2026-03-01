package frc.robot.commands;

import frc.robot.subsystems.climber.Climber;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeClimberCommand extends Command{
    private Climber climber;

    public HomeClimberCommand(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void execute() {
        if (!climber.isClimberHome()){
            climber.homeClimber();
        } else {
            if (!climber.isExtensionHome()) {
                climber.homeExtension();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isClimberHome() && climber.isExtensionHome();
    }
}
