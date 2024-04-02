package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Drivetrain;

public class ALignTrapShootCommand extends DriveWithController {

    private final PIDController m_xController = new PIDController(0, 0, 0);

    public ALignTrapShootCommand(Drivetrain drive) {
        super(
            drive,
            // X Move Velocity - Forward
            this::x,

            // Y Move Velocity - Strafe
            () ->0.0,

            // Rotate Angular velocity
            () ->  0.0,

            // Field Orientated
            () -> false,

            // Slow Mode
            () ->false,		
            

            // Disable X Movement
            () -> false,

            // Disable Y Movement
            () -> false,

            // Heading Override
            () -> null,
            () -> {
                // if (m_controller.getHID().getLeftBumper()) {
                //     if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue){
                //         return 90.0;
                //     } else if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
                //         return 90.0;
                //     }
                // }
                return null;
            
            }
    );
    }

    @Override
    public void initialize() {


       super.initialize();


    }

    public static double x() {
        return m_xController.calculate(LimelightHelpers.getTX("limelight"), 0);
    }
    
}
