package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Turret;

public class ShootCommand extends Command{

    private final Hopper hopper;
    private final Turret turret; 
    private boolean initialized = false;   

    public ShootCommand(Hopper hopper, Turret turret) {
        this.hopper = hopper;
        this.turret = turret;
        addRequirements(hopper);
    }

    @Override
    public void initialize() {}
    

    @Override
    public void execute () {
        // if (turret.isAimed()) {
        //     turret.enableShooter(true);
        // }
    }

    @Override 
    public void end(boolean interrupted) {
        turret.enableShooter(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
