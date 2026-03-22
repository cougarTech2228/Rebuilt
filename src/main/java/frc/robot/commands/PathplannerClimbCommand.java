package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.util.CTSequentialCommandGroup;

public class PathplannerClimbCommand extends CTSequentialCommandGroup{
    public PathplannerClimbCommand (String auto, Command newCommand) {
        addCommands(new PathPlannerAuto(auto),
        newCommand);
    }
}
