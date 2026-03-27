package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.OscillateType;
import frc.robot.subsystems.turret.Turret;

public class StartFiringCommand extends Command {
	private final Hopper hopper;
	private final Turret turret;
	private final Intake intake;

	public StartFiringCommand(Hopper hopper, Turret turret, Intake intake) {
		this.hopper = hopper;
		this.turret = turret;
		this.intake = intake;
	}

	private boolean isIndexerOn = false;

	@Override
	public void execute() {
		AutoAimCommand.autoAim = true;
		turret.enableShooter(true);
		if (turret.canShoot() && isIndexerOn == false) {
			hopper.indexerOn(false);
			hopper.kickerOn(false);
			isIndexerOn = true;
			intake.oscillate(OscillateType.WAVE);
		}
	}

	@Override
	public void end(boolean interrupted) {
		if(interrupted) {
			turret.enableShooter(false);
			hopper.indexerOff();
			hopper.kickerOff();
		}
		isIndexerOn = false;
	}

	@Override
	public boolean isFinished() {
		return isIndexerOn;
	}

}