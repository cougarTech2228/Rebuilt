package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.Climber.ClimberLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import com.pathplanner.lib.util.FlippingUtil;

public class AlignL1ClimbCommand extends Command {
   
    private final Drive driveSubsystem;
    private final Climber climberSubsystem;
    private final Turret turretSubsystem;

    private DriverStation.Alliance alliance;

    // Only Blue Alliance poses and rotations are needed now
    // NEED CALIBRATION BEFORE USE
    private static final Pose2d BLUE_TOWER_NORTH = new Pose2d(1.015, 4.683, Rotation2d.fromDegrees(0));
    private static final Pose2d BLUE_TOWER_SOUTH = new Pose2d(1.143, 2.689, Rotation2d.fromDegrees(0));

    private static final Rotation2d BLUE_NORTH_ROTATION = Rotation2d.fromDegrees(90);
    private static final Rotation2d BLUE_SOUTH_ROTATION = Rotation2d.fromDegrees(270);

    private Command subCommand;

    private PathConstraints globalConstraints = new PathConstraints(1, 1.5, Math.PI, Math.PI);
    private PathConstraints endConstraints = new PathConstraints(0.5, 0.75, Math.PI, Math.PI);
    private ArrayList<ConstraintsZone> listCZones;

    public AlignL1ClimbCommand(Drive driveSubsystem, Climber climberSubsystem, Turret turretSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.climberSubsystem = climberSubsystem;
        this.turretSubsystem = turretSubsystem;

        listCZones = new ArrayList<>();
        listCZones.add(new ConstraintsZone(0, 0, endConstraints));
    }

    @Override
    public void initialize() {
        Pose2d currentPose = driveSubsystem.getPose();
        Optional<Alliance> allianceCheck = DriverStation.getAlliance();
        assert allianceCheck.isPresent();
        alliance = allianceCheck.get();

        // Initialize with defaults to satisfy compiler
        Pose2d targetPose = new Pose2d();
        Rotation2d finalRotation = new Rotation2d();
        Pose2d approachPose = new Pose2d();
        Pose2d southApproachPose = null;
        boolean isSouthZone = false;

        // Define the path in "Blue Terms" 
        // If we are Red North, we use Blue South geometry as our base.
        if (Zone.HOME_ALLIANCE_ZONE_NORTH.inZone(currentPose, alliance)) {
            // This is Blue North or Red North (via symmetry)
            Pose2d base = (alliance == Alliance.Blue) ? BLUE_TOWER_NORTH : BLUE_TOWER_SOUTH;
            targetPose = base;
            finalRotation = (alliance == Alliance.Blue) ? BLUE_NORTH_ROTATION : BLUE_SOUTH_ROTATION;
            
            // Offset math: North tower on Blue side needs +X, 
            // but remember: Red North is actually Blue South flipped, so it needs -X.
            double xOff = (alliance == Alliance.Blue) ? 1.0 : -0.35;
            approachPose = new Pose2d(base.getX() + xOff, base.getY(), base.getRotation());

        } else if (Zone.HOME_ALLIANCE_ZONE_SOUTH.inZone(currentPose, alliance)) {
            // This is Blue South or Red South (via symmetry)
            Pose2d base = (alliance == Alliance.Blue) ? BLUE_TOWER_SOUTH : BLUE_TOWER_NORTH;
            targetPose = base;
            finalRotation = (alliance == Alliance.Blue) ? BLUE_SOUTH_ROTATION : BLUE_NORTH_ROTATION;
            
            double xOff = (alliance == Alliance.Blue) ? -0.20 : 1.0;
            double yOff = (alliance == Alliance.Blue) ? -0.50 : 1.0;

            approachPose = new Pose2d(base.getX() + xOff, base.getY(), base.getRotation());
            southApproachPose = new Pose2d(base.getX() + xOff, base.getY() + yOff, base.getRotation());
            isSouthZone = true;
        } else {
            this.cancel();
            return;
        }

        // Flip poses
        Rotation2d travelDirection = Rotation2d.fromDegrees(180);
        
        if (alliance == Alliance.Red) {
            targetPose = FlippingUtil.flipFieldPose(targetPose);
            finalRotation = FlippingUtil.flipFieldRotation(finalRotation);
            approachPose = FlippingUtil.flipFieldPose(approachPose);
            if (southApproachPose != null) {
                southApproachPose = FlippingUtil.flipFieldPose(southApproachPose);
            }
            travelDirection = FlippingUtil.flipFieldRotation(travelDirection);
        }

        // Waypoint Generation (Uses the flipped poses)
        turretSubsystem.climbMode(true);
        List<Waypoint> waypoints;
        if (isSouthZone && southApproachPose != null) {
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

        PathPlannerPath path = new PathPlannerPath(
            waypoints, new ArrayList<>(), new ArrayList<>(), listCZones,
            new ArrayList<>(), globalConstraints, null, 
            new GoalEndState(0.0, finalRotation), false
        );
        
        path.preventFlipping = true;
        subCommand = AutoBuilder.followPath(path);
        // subCommand.addRequirements(driveSubsystem);
        CommandScheduler.getInstance().schedule(subCommand);
    }
     
    @Override
    public boolean isFinished() {
        return subCommand != null && subCommand.isFinished(); // Added null check just in case
    }
}
