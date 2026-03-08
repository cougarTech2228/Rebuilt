package frc.robot.commands;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.wpilibj2.command.Command;

public class HomeClimberCommand extends Command{
    private final Climber climber;
    private final Turret  turret;

    public HomeClimberCommand(Climber climber, Turret turret) {
        this.climber = climber;
        this.turret = turret;
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

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.stop();
        }
        turret.climbMode(false);
    }
}
