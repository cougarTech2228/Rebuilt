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

    private TurretAimTarget aimTarget = TurretAimTarget.Hub;

    public Turret(TurretIO turretIO, Drive driveSubsystem) {
        this.turretIO = turretIO;
        this.driveSubsystem = driveSubsystem;

        SmartDashboard.putNumber("TurretHoodElevation", 0.0);
        SmartDashboard.putNumber("TurretFlywheelVelocity", 0.0);
        SmartDashboard.putNumber("TurretTestDistance", 0.0);
        SmartDashboard.putNumber("TurretTestFlywheelRatio", 1.0);

        SmartDashboard.putNumber("TurretAngle", 0.0);
    }

    private double getTargetDistance() {
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = getTargetPoint(aimTarget);
        return robotPose.getTranslation().getDistance(targetPose.getTranslation());
    }

    @Override
    public void periodic() {
        turretIO.updateInputs(turretInputs);
        
        turretInputs.targetDistance = getTargetDistance();
        Logger.processInputs("Turret", turretInputs);

        RobotContainer.turretPose = new Pose3d(
            RobotContainer.turretPose.getX(),
            RobotContainer.turretPose.getY(),
            RobotContainer.turretPose.getZ(),
            new Rotation3d(turretInputs.turretAngle));
        
        RobotContainer.turretHoodPose = new Pose3d(
            RobotContainer.turretHoodPose.getX(),
            RobotContainer.turretHoodPose.getY(),
            RobotContainer.turretHoodPose.getZ(),
            new Rotation3d(0,turretInputs.hoodMotorPosition,turretInputs.turretAngle.getRadians()));
    }

    public void setAimTarget(double turretAngle) {
        turretIO.setTurretAngle(turretAngle);
    }

    public void setAimTarget(TurretAimTarget target) {
        aimTarget = target;
        Pose2d robotPose = driveSubsystem.getPose();
        Pose2d targetPose = getTargetPoint(target);
        turretInputs.turretTargetPoint = targetPose;

        // Get the Translation (X, Y) of both poses
        Translation2d robotXY = robotPose.getTranslation().plus(TurretConstants.TurretOffset.getTranslation().toTranslation2d());
        Translation2d targetXY = targetPose.getTranslation();

        // Calculate the difference vector (Vector from Robot -> Target)
        Translation2d difference = targetXY.minus(robotXY);

        // Get the angle of that vector relative to the field
        Rotation2d angleToTargetFieldRelative = difference.getAngle();

        // Subtract robot's rotation to make it robot-relative
        Rotation2d angleToTargetRobotRelative = angleToTargetFieldRelative.minus(robotPose.getRotation());

        double turretAngle = MathUtil.inputModulus(angleToTargetRobotRelative.getDegrees(), 0, 360);

        double TURRET_KEEPOUT_START = TurretConstants.TURRET_MAX_ROTATION;
        double TURRET_KEEPOUT_END   = 360 + TurretConstants.TURRET_MIN_ROTATION;

        // If the target falls in the keep-out zone, snap to the nearest safe boundary.
        turretInputs.isTargetInKeepOut = turretAngle > TURRET_KEEPOUT_START &&
                                        turretAngle < TURRET_KEEPOUT_END;

        if (turretInputs.isTargetInKeepOut) {
            double distToStart = Math.abs(turretAngle - TURRET_KEEPOUT_START);
            double distToEnd   = Math.abs(turretAngle - TURRET_KEEPOUT_END);
            turretAngle = (distToStart < distToEnd)
                ? TURRET_KEEPOUT_START
                : TURRET_KEEPOUT_END;
        }

        if (SmartDashboard.getBoolean("TestMode", false)) {
            turretIO.setTurretAngle(SmartDashboard.getNumber("TurretAngle", 0.0));
        } else {
            turretIO.setTurretAngle(turretAngle);
        }
    }

    public void setHoodElevation(double elevation) {
        turretIO.setHoodAngle(elevation);
    }

    private double getAngleForTarget() {
        double distance = 0;

        double turretTestDistance = SmartDashboard.getNumber("TurretTestDistance", 0.0);
        if (turretTestDistance > 0) {
            distance = turretTestDistance;
        } else {
            distance = getTargetDistance();
        }

        double angle;

        if (aimTarget == TurretAimTarget.Hub) {
            // // y = 4E-11x4 - 8E-08x3 + 6E-05x2 - 0.0139x + 1.1316
            // angle = (4E-11 * Math.pow(distance, 4)) -
            //         (8E-08 * Math.pow(distance, 3)) +
            //         (6E-05 * Math.pow(distance, 2)) -
            //         (0.0139 * distance) + 1.1316;

            // y = -0.0666x2 + 0.561x - 0.7949

            angle = (-0.0666 * distance * distance) + (0.561 * distance) - 0.7949;

        } else {
            // FIX ME -- need real fomula
            angle = 0.4;
        }

        // sanity check the values
        angle = Math.min(angle, TurretConstants.HOOD_MAX_ANGLE);
        angle = Math.max(angle, TurretConstants.HOOD_MIN_ANGLE);

        return angle;
    }

    // private double getRatioForTarget() {
    //     // y = 5E-09x3 - 7E-06x2 + 0.0029x + 0.6728
    // }

    private double getVelocityForTarget() {
        double distance = 0;

        double turretTestDistance = SmartDashboard.getNumber("TurretTestDistance", 0.0);
        if (turretTestDistance > 0) {
            distance = turretTestDistance;
        } else {
            Pose2d robotPose = driveSubsystem.getPose();
            Pose2d targetPose = getTargetPoint(aimTarget);
            distance = robotPose.getTranslation().getDistance(targetPose.getTranslation());
        }

        double velocity;

        if (aimTarget == TurretAimTarget.Hub) {
            // // y = -4E-05x2 + 0.0706x + 24.273
            // velocity = (-4E-05 * distance * distance) + (0.0706 * distance) + 24.273;
            //y = 3.5535x + 21.142
            velocity = (3.5535 * distance) + 21.142;

        } else {
            // FIX ME -- need real fomula
            velocity = (distance + 1.2833) / 0.187;
        }

        // cap the speed to an achievable value
        velocity = Math.min(velocity, TurretConstants.MAX_FLYWHEEL_SPEED);

        // ensure we don't end up with a negative value or super slow speed
        velocity = Math.max(velocity, TurretConstants.MIN_FLYWHEEL_SPEED);

        return velocity;
    }

    public void setFlywheelVelocity(double mainVelocity, double upperVelocity) {
        turretIO.setFlywheelVelocity(mainVelocity, upperVelocity);
    }

    public void enableShooter(boolean enable) {
        if (SmartDashboard.getBoolean("TestMode", false)) {
            if (enable) {
                setHoodElevation(SmartDashboard.getNumber("TurretHoodElevation", 0.0));
                double ratio = SmartDashboard.getNumber("TurretTestFlywheelRatio", 1.0);
                double velocity = SmartDashboard.getNumber("TurretFlywheelVelocity", 0.0);
                setFlywheelVelocity(velocity, ratio * velocity);

                setAimTarget(SmartDashboard.getNumber("TurretAngle", 0.0));
            } else {
                setFlywheelVelocity(0, 0);
            }
            // boolean indexerTest = SmartDashboard.getBoolean("IndexerTest", false);
            // if (indexerTest) {
            //   hopper.indexerOn(true);
            //   hopper.kickerOn(true);
            // } else {
            //   hopper.indexerOff();
            //   hopper.kickerOff();
            // }
            // intake.setIntakeAngle(SmartDashboard.getNumber("IntakePosition", 1.0));
            // intake.setIntakeVelocity(SmartDashboard.getNumber("IntakeVelocity", 1.0));
        } else {
            if (enable) {
                setFlywheelVelocity(getVelocityForTarget(), getVelocityForTarget());
                setHoodElevation(getAngleForTarget());
            } else {
                setFlywheelVelocity(0, 0);
                // no need to move the hood on disable
            }
        }
    }

    public boolean canShoot() {
        return turretInputs.areFlywheelsAtVelocity && turretInputs.isTurretAtTarget;
    }

    private Pose2d getTargetPoint(TurretAimTarget target) {
        if (DriverStation.getAlliance().isEmpty()){
            // no aliance color found yet, probably simulation
            return new Pose2d();
        }

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
