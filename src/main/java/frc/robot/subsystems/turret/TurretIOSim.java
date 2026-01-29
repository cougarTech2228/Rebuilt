package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;

public class TurretIOSim implements TurretIO{

    private double turretAngleTarget;
    @Override
    public void setTurretAngle(double turretAngle) {
        turretAngleTarget = turretAngle;
    }

    private double turretHoodTarget;
    @Override
    public void setHoodAngle(double hoodAngle) {
        turretHoodTarget = hoodAngle;
    }
    
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turretPIDTargetAngle = turretAngleTarget;
        inputs.hoodPIDTargetAngle = turretHoodTarget;
        inputs.turretAngle = Rotation2d.fromDegrees(turretAngleTarget);
    }
}
