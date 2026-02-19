// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final SparkFlex leaderMotor;
  private final SparkFlex followerMotor;
  private final SparkClosedLoopController closedLoopController;
  private final RelativeEncoder encoder;

  private double targetRpm = 0;
  private final double RPM_TOLERANCE = 100; // Spped margin of error

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.1, 0.0015, 0);

  /** Creates a new Shooter. */
  public Shooter() {
    leaderMotor = new SparkFlex(10, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    followerMotor = new SparkFlex(11, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    closedLoopController = leaderMotor.getClosedLoopController();
    encoder = leaderMotor.getEncoder();

    SparkFlexConfig leaderConfig = new SparkFlexConfig();
    SparkFlexConfig followerConfig = new SparkFlexConfig();

    leaderConfig.idleMode(IdleMode.kCoast) 
                .smartCurrentLimit(60);   

    followerConfig.idleMode(IdleMode.kCoast)
                  .smartCurrentLimit(60)
                  .follow(leaderMotor, true);

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);
  }

  public void setTargetRpm(double rpm) {
    this.targetRpm = rpm;
    
    if (rpm <= 0) {
        leaderMotor.set(0); 
    } else {
        double ffVoltage = feedforward.calculate(rpm);
        
        closedLoopController.setSetpoint(
            rpm, 
            ControlType.kVelocity, 
            ClosedLoopSlot.kSlot0, 
            ffVoltage, 
            ArbFFUnits.kVoltage
        );
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
