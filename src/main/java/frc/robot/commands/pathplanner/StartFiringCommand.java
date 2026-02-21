package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Turret;

public class StartFiringCommand extends Command {
	private final Hopper hopper;
	private final Turret turret;

	public StartFiringCommand(Hopper hopper, Turret turret) {
		this.hopper = hopper;
		this.turret = turret;
	}

	private boolean initialized = false;
	private boolean isIndexerOn = false;
	
	@Override
	public void initialize() {
		turret.enableShooter(true);
	}

	@Override
	public void execute() {
		if (turret.canShoot() && isIndexerOn == false) {
			hopper.indexerOn(false);
			isIndexerOn = true;
		}
	}

	@Override
	public void end(boolean interrupted) {
		if(interrupted) {
			turret.enableShooter(false);
			hopper.indexerOff();
		}
		isIndexerOn = false;
	}

	@Override
	public boolean isFinished() {
		return turret.canShoot();
	}
}