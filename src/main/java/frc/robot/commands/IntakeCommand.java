// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.hopper.Hopper;
// import frc.robot.subsystems.intake.Intake;

// public class IntakeCommand extends Command{

//     private final Intake intake;
//     private final Hopper hopper;
//     private boolean initialized = false; 
    
//     public IntakeCommand(Intake intake, Hopper hopper) {
//         this.intake = intake;
//         this.hopper = hopper;
//         }    

//     @Override
//     public void initialize() {
//         if(hopperIsDeployed = false) {
//             hopper.deployHopper();
//         }
//     }

//     @Override
//     public void execute () {
//         intake.enableIntake();
//         hopper.enableHopper();
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }

//     @Override 
//     public void end(boolean interrupted) {
//         intake.disableIntake();
//         hopper.disableHopper();

//         if (hopperIsDeployed == true && safeToRetract() == true) {
//             hopper.retractHopper();
//         }
//     }
// }