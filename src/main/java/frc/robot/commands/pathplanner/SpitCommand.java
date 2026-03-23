package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeMode;

public class SpitCommand extends Command {
	private final Hopper hopper;
	private final Intake intake;

	public SpitCommand(Hopper hopper, Intake intake) {
		this.hopper = hopper;
		this.intake = intake;
	}
	
	@Override
	public void initialize() {

	}

	@Override
	public void execute() {
		intake.setIntakeMode(IntakeMode.SPIT);
	}

	@Override
	public void end(boolean interrupted) {

	}

	@Override
	public boolean isFinished() {
		return true;
	}
}