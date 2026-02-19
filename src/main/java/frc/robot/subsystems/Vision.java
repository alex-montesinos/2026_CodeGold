// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final String limelightName = "limelight";

    // Height from the floor to the center of your Limelight lens
    private static final double CAMERA_HEIGHT_METERS = 0.5; 
    
    // Height from the floor to the center of the 2026 target
    private static final double TARGET_HEIGHT_METERS = 2.0; 
    
    // The angle your camera is tilted up from perfectly horizontal
    private static final double CAMERA_PITCH_DEGREES = 30.0; 

    public Vision() {}

    public boolean hasTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTy() {
        return LimelightHelpers.getTY(limelightName);
    }

    public double getDistanceToTargetMeters() {
        if (!hasTarget()) {
            return 0.0; 
        }
        
        // a2: The vertical angle to the target from the Limelight
        double targetOffsetAngleDegrees = getTy();
        
        // a1 + a2: Total angle from the ground
        double angleToGoalDegrees = CAMERA_PITCH_DEGREES + targetOffsetAngleDegrees;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        return (TARGET_HEIGHT_METERS - CAMERA_HEIGHT_METERS) / Math.tan(angleToGoalRadians);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/TX (Aiming)", getTx());
        SmartDashboard.putNumber("Vision/TY", getTy());
        SmartDashboard.putNumber("Vision/Calculated Distance (m)", getDistanceToTargetMeters());
    }
}