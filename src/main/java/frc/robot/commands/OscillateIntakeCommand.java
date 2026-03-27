package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;

public class OscillateIntakeCommand extends Command {
    private final Intake intake;

    public OscillateIntakeCommand(Climber climber, Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        intake.oscillate(Intake.OscillateType.WAVE);
    }

    @Override
    public boolean isFinished() {
        boolean finished = !intake.isOscillateActive();
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            intake.oscillate(Intake.OscillateType.STOP);
        }
    }
}
