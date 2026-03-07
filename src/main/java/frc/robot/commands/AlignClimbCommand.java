package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignClimbCommand extends Command {
    
    private final Drive driveSubsystem;
    private final Climber climberSubsystem;

    private Command subCommand;
    private Pose2d endPoint;
    // private final boolean useApproachPoint;

    private PathConstraints globalConstraints = new PathConstraints(3, 2, Math.PI, Math.PI);
    private PathConstraints endConstraints = new PathConstraints(1.5, 1, Math.PI, Math.PI);
    private ArrayList<ConstraintsZone> listCZones;

    public AlignClimbCommand(Drive driveSubsystem, Climber climberSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.climberSubsystem = climberSubsystem;

        listCZones = new ArrayList<>();
        listCZones.add(new ConstraintsZone(0, 0, endConstraints));
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveSubsystem.getPose();
        PathPlannerPath path;

        // if (Zone.)
        
    }
     
    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        Alliance alliance = Alliance.Blue;
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get();
        }
        if (Zone.HOME_ALLIANCE_ZONE.inZone(currentPose, alliance)) {
            if (Zone.HOME_ALLIANCE_ZONE_NORTH.inZone(currentPose, alliance)) {
                // destination = // LADDER LEFT POSITION
                
            } else if (Zone.HOME_ALLIANCE_ZONE_SOUTH.inZone(currentPose, alliance)) {
                // destination = // LADDER RIGHT POSITION
            }

        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
