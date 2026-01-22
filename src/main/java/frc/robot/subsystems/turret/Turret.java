package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

public class Turret extends SubsystemBase{
    public enum TurretAimTarget  {
        Hub,
        LobLower,
        LobUpper
    };

    private TurretIO turretIO;
    private Drive driveSubsystem;

    public Turret(TurretIO turretIO, Drive driveSubsystem) {
        this.turretIO = turretIO;
        this.driveSubsystem = driveSubsystem;
    }

    public void setAimTarget(TurretAimTarget target) {
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = getTargetPoint(target);
    
    }

    public void enableShooter(boolean enable) {

    }

    public boolean canShoot() {
        
    }

    private Pose2d getTargetPoint(TurretAimTarget target) {
        Alliance currentAlliance = DriverStation.getAlliance().get();
        Pose2d pose = new Pose2d();

        switch (target) {
            case Hub: {
                double x = 4.626;
                double y = 4.031;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobLower: {
                double x = 2;
                double y = 1.685;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobUpper: {
                double x = 2;
                double y = aprilTagLayout.getFieldWidth() - 1.685;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
        }
        return pose;
    }
}
