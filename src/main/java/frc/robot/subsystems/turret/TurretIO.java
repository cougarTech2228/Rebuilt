package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public interface TurretIO {

    @AutoLog
    public static class TurretIOInputs {
        public Pose2d turretTargetPoint = new Pose2d();

        public Rotation2d turretAngle = new Rotation2d();
        public double turretPIDTargetAngle = 0;
        public double turretPIDActualAngle = 0;
        public double turretMotorPIDTarget = 0;
        public double turretMotorRotations = 0;
        public double enc19t = 0;
        public double enc21t = 0;

        public double hoodTargetElevationPercent = 0;
        public double hoodPIDTargetAngle = 0;
        public double hoodPIDActualAngle = 0;
  

        public double hoodMotorVelocity = 0;
        public double hoodMotorVoltage = 0;
        public double hoodMotorCurrent = 0;

        public double flywheelMotorVelocity = 0;
        public double flywheelMotorVoltage = 0;
        public double flywheelMotorCurrent = 0;
        public double flywheelPIDTargetVelocity = 0;
    }
    public default void updateInputs(TurretIOInputs inputs) {}

    public default boolean isFlywheelAtVelocity() {return false;}


    public default void setTurretAngle(double turretAngle) {}
    public default void setHoodAngle(double hoodAngle) {}
    public default void setFlywheelVelocity(double velocity) {}
    public default void setFlywheelsActive(boolean active) {}
};
