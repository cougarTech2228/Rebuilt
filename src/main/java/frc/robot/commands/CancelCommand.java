package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class CancelCommand extends Command {
    
    private Command wantedCommand;

    public CancelCommand(Command wantedCommand) {
        this.wantedCommand = wantedCommand;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        CommandScheduler.getInstance().cancel(wantedCommand);
    }

    @Override
    public boolean isFinished() {
        if (wantedCommand.isScheduled()) {
            // The command instance is currently running
            return false;
        } else {
            // The command instance is not running
            return true;
        } 
    }
}   
