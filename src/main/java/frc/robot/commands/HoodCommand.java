 package frc.robot.commands;
import frc.robot.subsystems.turret.Turret;
import edu.wpi.first.wpilibj2.command.Command;

public class HoodCommand extends Command{
    private final Turret turret;
     


    public HoodCommand (Turret turret, double hoodAngle) {
        this.turret = turret;
        
        
        addRequirements(turret);
    }

}
