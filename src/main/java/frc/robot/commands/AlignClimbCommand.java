package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

public class AlignClimbCommand extends Command {
    
    private final Drive driveSubsystem;
    private final Climber climberSubsystem;
    private final Turret turretSubsystem;

    private DriverStation.Alliance alliance;
    // 0.970
    private static final Pose2d BLUE_TOWER_NORTH = new Pose2d(0.970, 4.560, Rotation2d.fromDegrees(0)); // +0.5
    // Blue tower south needs readjusting
    private static final Pose2d BLUE_TOWER_SOUTH = new Pose2d(1.198, 2.852, Rotation2d.fromDegrees(0)); // +0.5
    private static final Pose2d RED_TOWER_NORTH = new Pose2d(15.318, 5.336, Rotation2d.fromDegrees(0)); // -0.5
    private static final Pose2d RED_TOWER_SOUTH = new Pose2d(15.635, 3.357, Rotation2d.fromDegrees(0)); // -0.5

    private static final Rotation2d BLUE_NORTH_ROTATION = Rotation2d.fromDegrees(90); // 180 - 60 (or 300)
    private static final Rotation2d BLUE_SOUTH_ROTATION = Rotation2d.fromDegrees(270); // 180 - 60 (or 300)

    private static final Rotation2d RED_NORTH_ROTATION = Rotation2d.fromDegrees(90);
    private static final Rotation2d RED_SOUTH_ROTATION = Rotation2d.fromDegrees(270);

    private Command subCommand;
    // private final boolean useApproachPoint;

    private PathConstraints globalConstraints = new PathConstraints(1, 1.5, Math.PI, Math.PI);
    private PathConstraints endConstraints = new PathConstraints(0.5, 0.75, Math.PI, Math.PI);
    private ArrayList<ConstraintsZone> listCZones;

    public AlignClimbCommand(Drive driveSubsystem, Climber climberSubsystem, Turret turretSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.turretSubsystem = turretSubsystem;

        listCZones = new ArrayList<>();
        listCZones.add(new ConstraintsZone(0, 0, endConstraints));
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d targetPose = new Pose2d();
        Rotation2d finalRotation = new Rotation2d();
        double approachYOffset = 0;
        double approachXOffset = 0;

        Optional<Alliance> allianceCheck = DriverStation.getAlliance();

        assert allianceCheck.isPresent();
        alliance = allianceCheck.get();

        // Get Target Pose
        if (Zone.HOME_ALLIANCE_ZONE_NORTH.inZone(currentPose, alliance)) {
            targetPose = (alliance == Alliance.Blue) ? BLUE_TOWER_NORTH : RED_TOWER_NORTH;
            finalRotation = (alliance == Alliance.Blue) ? BLUE_NORTH_ROTATION : RED_NORTH_ROTATION;
            // approachYOffset = 1.5;
            approachXOffset = (alliance == Alliance.Blue) ? 1.0 : -1.0;
        } else if (Zone.HOME_ALLIANCE_ZONE_SOUTH.inZone(currentPose, alliance)) {
             targetPose = (alliance == Alliance.Blue) ? BLUE_TOWER_SOUTH : RED_TOWER_SOUTH;
             finalRotation = (alliance == Alliance.Blue) ? BLUE_SOUTH_ROTATION : RED_SOUTH_ROTATION;
             approachYOffset = -1.0;
             approachXOffset = (alliance == Alliance.Blue) ? -0.35 : 0.35;
        } else {
            this.cancel();
            return;
        } 

        turretSubsystem.climbMode(true);
        // Create approach point, via same x-axis and y-axis offset
        Pose2d approachPose = new Pose2d(targetPose.getX() + approachXOffset, targetPose.getY(), targetPose.getRotation());
        Pose2d southApproachPose = new Pose2d(targetPose.getX() + approachXOffset, targetPose.getY() + approachYOffset, targetPose.getRotation());

        // Gen waypoints, get direction of travel
        Rotation2d travelDirection = (alliance == Alliance.Blue) ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0);

        List<Waypoint> waypoints;
        if (Zone.HOME_ALLIANCE_ZONE_SOUTH.inZone(currentPose, alliance)) {
            waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), travelDirection),
            new Pose2d(southApproachPose.getTranslation(), travelDirection),
            new Pose2d(approachPose.getTranslation(), travelDirection),
            new Pose2d(targetPose.getTranslation(), travelDirection)
            );
        } else {
            waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(currentPose.getTranslation(), travelDirection),
            new Pose2d(approachPose.getTranslation(), travelDirection),
            new Pose2d(targetPose.getTranslation(), travelDirection)
            );
        }
        
        // Make path
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            new ArrayList<>(), 
            new ArrayList<>(), 
            listCZones,
            new ArrayList<>(), 
            globalConstraints,
            null, 
            new GoalEndState(0.0, finalRotation), // PathPlanner snaps to this angle at the end
            false
        );
       
        path.preventFlipping = true;

        subCommand = AutoBuilder.followPath(path);
        subCommand.addRequirements(driveSubsystem);
        CommandScheduler.getInstance().schedule(subCommand);

        // if (zone != null) {
        //     // Rotation2d targetAngle = zone.getAngle(alliance);
        //     // Pose2d turnPose;

        //     if (alliance == DriverStation.Alliance.Blue) {
        //         turnPose = new Pose2d();
        //     } else {
        //         turnPose = new Pose2d();
        //     }
        // }
        

    }
     
    @Override
    public boolean isFinished() {
        if (subCommand.isFinished()) {
            return true;
        }
        return false;
    }
}
