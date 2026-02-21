package frc.robot.commands.pathplanner;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.intake.Intake.IntakeMode;

public class PerformClimbCommand extends Command {
    private Hopper hopper;
    private Intake intake;
    private Climber climber;
    
	public PerformClimbCommand(Hopper hopper, Intake intake, Climber climber) {
		this.hopper = hopper;
		this.intake = intake;
	}

	private boolean initialized = false;
    private boolean didStartClimbing = false;
	
	@Override
	public void initialize() {
        intake.setIntakeMode(IntakeMode.IDLE);
        intake.setIntakeAngle(IntakeAngle.HOME);
        climber.extend();
	}

	@Override
	public void execute() {
        if(climber.isExtended()) {
            climber.climb();
            didStartClimbing = true;
        }
	}

	@Override
	public void end(boolean interrupted) {
        
	}

	@Override
	public boolean isFinished() {
		return didStartClimbing;
	}
}
