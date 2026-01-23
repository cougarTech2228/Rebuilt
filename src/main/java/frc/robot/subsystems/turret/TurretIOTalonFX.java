package frc.robot.subsystems.turret;


public class TurretIOTalonFX implements TurretIO {

    private double turretAngleTarget;

    @Override
    public void setTurretAngle(double turretAngle) {
        turretAngleTarget = turretAngle;
    }
    
}
