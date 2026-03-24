package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeAngle;
import frc.robot.subsystems.turret.Turret;

public class ExtendClimberCommand extends Command {
    private final Climber climber;
    private final Intake intake;
    private final Climber.ClimberLevel level;
    private final Turret turret;
    private boolean extendCommandSent = false;

    public ExtendClimberCommand(Climber climber, Intake intake, Climber.ClimberLevel level, Turret turret) {
        this.climber = climber;
        this.intake = intake;
        this.level = level;
        this.turret = turret;

        addRequirements(climber, intake);
    }

    @Override
    public void initialize() {
        turret.climbMode(true);
        extendCommandSent = false;
    }

    @Override
    public void execute() {
        // we MUST prevent the climber extension from being deployed while the intake is out
        // or we will get a penality for extending on 2 sides of the bot
        if (!intake.isRetracted()){
            intake.setIntakeAngle(IntakeAngle.HOME);
        } else if (!extendCommandSent){
            climber.extend(level);
            extendCommandSent = true;
        }
    }

    @Override
    public boolean isFinished() {
        return climber.isExtended(level);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            climber.stop();
            intake.stop();
        }
    }
}
