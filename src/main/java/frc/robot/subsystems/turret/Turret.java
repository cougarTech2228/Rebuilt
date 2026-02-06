package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase{
    public enum TurretAimTarget  {
        Hub,
        LobLower,
        LobUpper
    };

    private TurretIO turretIO;
    private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

    private Drive driveSubsystem;

    public Turret(TurretIO turretIO, Drive driveSubsystem) {
        this.turretIO = turretIO;
        this.driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("TurretHoodElevation", 0.0);
        SmartDashboard.putNumber("TurretFlywheelVelocity", 0.0);
        SmartDashboard.putNumber("TurretAngle", 0.0);
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        Logger.processInputs("Turret", turretInputs);

        RobotContainer.turretPose = new Pose3d(
            RobotContainer.turretPose.getX(),
            RobotContainer.turretPose.getY(),
            RobotContainer.turretPose.getZ(),
            new Rotation3d(turretInputs.turretAngle));
    }

    public void setAimTarget(double turretAngle) {
        turretIO.setTurretAngle(turretAngle);
    }

    public void setAimTarget(TurretAimTarget target) {
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = getTargetPoint(target);
        turretInputs.turretTargetPoint = targetPose;
    
        // Get the Translation (X, Y) of both poses
        Translation2d robotXY = robotPose.getTranslation();
        Translation2d targetXY = targetPose.getTranslation();

        // Calculate the difference vector (Vector from Robot -> Target)
        Translation2d difference = targetXY.minus(robotXY);

        // Get the angle of that vector relative to the field
        Rotation2d angleToTargetFieldRelative = difference.getAngle();

        // Subtract robot's rotation to make it relative to the robot front
        // (Field Relative Angle - Robot Heading = Robot Relative Angle)
        Rotation2d angleToTargetRobotRelative = angleToTargetFieldRelative.minus(robotPose.getRotation());

        double degrees = angleToTargetRobotRelative.getDegrees();

        // normalize 0-360
        double turretAngle = MathUtil.inputModulus(degrees, 0, 360);

        turretIO.setTurretAngle(turretAngle);
    }

    public void setHoodElevation(double elevation) {
        turretIO.setHoodAngle(elevation);
    }

    private double getVelocityForTarget() {
        // a bunch of math at some point to calc all the things
        return 15;
    }

    public void setFlywheelVelocity(double velocity) {
        turretIO.setFlywheelVelocity(velocity);
    }

    public void enableShooter(boolean enable) {
        if (enable) {
            setFlywheelVelocity(getVelocityForTarget()); 
        } else {
            setFlywheelVelocity(0);
        }

        
    }

    public boolean canShoot() {
        return turretIO.isFlywheelAtVelocity();
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
                double y = 1;
                if (currentAlliance == Alliance.Blue) {
                    pose = new Pose2d(x, y, new Rotation2d());
                } else {
                    pose = new Pose2d(aprilTagLayout.getFieldLength() - x, y, new Rotation2d());
                }
                break;
            }
            case LobUpper: {
                double x = 2;
                double y = aprilTagLayout.getFieldWidth() - 1;
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
