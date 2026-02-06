package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Turret;

public class ShootCommand extends Command{

    private final Hopper hopper;
    private final Turret turret; 
    private boolean initialized = false;   
    private boolean isIndexerOn = false;
    private boolean isRunning = false;

    public ShootCommand(Hopper hopper, Turret turret) {
        this.hopper = hopper;
        this.turret = turret;
    }

    @Override
    public void initialize() {
        turret.enableShooter(true);
        isRunning = true;
    }
    

    @Override
    public void execute () {
            if (turret.canShoot() && isIndexerOn == false) {
                hopper.indexerOn(false);
                isIndexerOn = true;
            }


        // if (turret.isAimed()) {
        //     turret.enableShooter(true);
        // }
    }

    @Override
     public void end(boolean interrupted) {
       isRunning = false;
      isIndexerOn = false;
        turret.enableShooter(false);
         hopper.indexerOff();
    }

    @Override
    public boolean isFinished() {
        return !isRunning;
    }
}
