package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoAimCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.OscillateType;
import frc.robot.subsystems.turret.Turret;

public class StopFiringCommand extends Command {
	private final Hopper hopper;
	private final Turret turret;
	private final Intake intake;

	public StopFiringCommand(Hopper hopper, Turret turret, Intake intake) {
		this.hopper = hopper;
		this.turret = turret;
		this.intake = intake;
	}


	@Override
	public void initialize() {
		turret.enableShooter(false);
		hopper.indexerOff();
		hopper.kickerOff();
		intake.oscillate(OscillateType.STOP);
		AutoAimCommand.autoAim = false;
	}


	@Override
	public boolean isFinished() {
		return true;
	}
}