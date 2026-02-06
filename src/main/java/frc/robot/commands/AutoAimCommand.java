package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretAimTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimCommand extends Command {

    private final Drive driveSubsystem;
    private final Turret turretSubsystem;

    public AutoAimCommand(Drive driveSubsystem, Turret turretSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
    }


    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        Alliance currentAlliance = Alliance.Blue;
        if (DriverStation.getAlliance().isPresent()) {
            currentAlliance = DriverStation.getAlliance().get();
        }
        if (Destination.HOME_ALLIANCE_ZONE.inZone(currentPose, currentAlliance)){
            turretSubsystem.setAimTarget(TurretAimTarget.Hub);
        } else if (Destination.NEUTRAL_ZONE_NORTH.inZone(currentPose, currentAlliance)){
            turretSubsystem.setAimTarget(TurretAimTarget.LobUpper);
        } else if (Destination.NEUTRAL_ZONE_SOUTH.inZone(currentPose, currentAlliance)){
            turretSubsystem.setAimTarget(TurretAimTarget.LobLower);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
