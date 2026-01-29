package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public Pose2d turretTargetPoint = new Pose2d();

        public Rotation2d turretAngle = new Rotation2d();
        public double turretPIDTargetAngle = 0;
        public double turretPIDActualAngle = 0;

        public double hoodPIDTargetAngle = 0;
        public double hoodPIDActualAngle = 0;
    }
    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurretAngle(double turretAngle) {}
    public default void setHoodAngle(double hoodAngle) {}
    public default void setFlywheelsActive(boolean active) {}
}
