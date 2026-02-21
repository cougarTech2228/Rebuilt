package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Turret;

public class StopFiringCommand extends Command {
	private final Hopper hopper;
	private final Turret turret;

	public StopFiringCommand(Hopper hopper, Turret turret) {
		this.hopper = hopper;
		this.turret = turret;
	}

	private boolean initialized = false;

	@Override
	public void initialize() {
		turret.enableShooter(false);
		hopper.indexerOff();
	}

	@Override
	public void execute() {

	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return turret.canShoot();
	}
}