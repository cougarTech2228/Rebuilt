package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Turret;

public class ShootCommand extends Command {
    private final Hopper hopper;
    private final Turret turret;

    public ShootCommand(Hopper hopper, Turret turret) {
        this.hopper = hopper;
        this.turret = turret;
    }

    @Override
    public void execute() {
        turret.enableShooter(true);
        if (turret.canShoot()) {
            hopper.indexerOn(false);
            hopper.kickerOn(false);
        } else {
            hopper.indexerOff();
            hopper.kickerOff();
        }
    }

    @Override
    public void end(boolean interrupted) {
        turret.enableShooter(false);
        hopper.indexerOff();
        hopper.kickerOff();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
