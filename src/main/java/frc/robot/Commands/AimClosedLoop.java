package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AimClosedLoop extends Command {
    private final DriveSubsystem swerve;
    private final Shooter shooter;
    private final Vision vision;
    
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;

    private final PIDController aimPID = new PIDController(0.04, 0.0, 0.005); 
    
    private final InterpolatingDoubleTreeMap rpmMap = new InterpolatingDoubleTreeMap();

    public AimClosedLoop(DriveSubsystem swerve, Shooter shooter, Vision vision, 
                            DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        this.swerve = swerve;
        this.shooter = shooter;
        this.vision = vision;
        this.translationXSupplier = xSupplier;
        this.translationYSupplier = ySupplier;

        addRequirements(swerve, shooter);

        // Truth Table
        rpmMap.put(1.0, 2000.0);
        rpmMap.put(2.0, 3100.0);
        rpmMap.put(3.0, 4500.0);
        rpmMap.put(5.0, 5800.0); 
    }

    @Override
    public void execute() {
        double translationX = translationXSupplier.getAsDouble();
        double translationY = translationYSupplier.getAsDouble();
        
        double rotationSpeed = 0.0;
        double targetRpm = 1500.0;

        if (vision.hasTarget()) {
            
            rotationSpeed = aimPID.calculate(vision.getTx(), 0.0);
            
            double distance = vision.getDistanceToTargetMeters();
            targetRpm = rpmMap.get(distance);
        }

        // Aim Robot
        swerve.drive(translationX, translationY, rotationSpeed, true); 

        // Rev the shooter
        shooter.setTargetRpm(targetRpm);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the shooter when the button is released, or return to an idle RPM
        shooter.setTargetRpm(0);
    }
}