// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  SparkFlex motor;
  SparkClosedLoopController controller;
  RelativeEncoder encoder;

  private double targetRPM = 0.0;

  public static Shooter instance;
  public static Shooter getInstance() {
    if(instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  /** Creates a new Shooter. */
  private Shooter() {
    motor = new SparkFlex(ShooterConstants.kRollerPort, MotorType.kBrushless);

    motor.setCANTimeout(250);

    SparkFlexConfig config = new SparkFlexConfig();
    config.closedLoop
    .p(ShooterConstants.kP)
    .i(ShooterConstants.kI)
    .d(ShooterConstants.kD);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.setCANTimeout(0);

    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();
  }

  public void shoot() {
    controller.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  public boolean atSetpoint() {
    return Math.abs(getRPM() - targetRPM) < ShooterConstants.rpmtolerance;
  }

  public double getRPM() {
    return encoder.getVelocity();
  }

  public void setTargetRPMFromDistance(double distanceMeters) {
    double rpm = ShooterConstants.kDistanceToRPM * distanceMeters + ShooterConstants.kRPMIntercept;

    rpm = Math.max(ShooterConstants.MIN_RPM, Math.min(rpm, ShooterConstants.MAX_RPM));

    targetRPM = rpm;
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void stop() {
    targetRPM = 0;
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
