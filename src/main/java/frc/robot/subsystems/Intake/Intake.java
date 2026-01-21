// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  SparkMax roller, pivotLeader, pivotFollower;
  RelativeEncoder encoder;
  SparkClosedLoopController pivotController;

  public static Intake instance;

  public static Intake getInstance() {
    if(instance == null) {
      instance = new Intake();
    }
    return instance;
  }

  /** Creates a new IntakeSubsystem. */
  public Intake() {
    roller = new SparkMax(IntakeConstants.kRollerPort, MotorType.kBrushless);
    pivotLeader = new SparkMax(IntakeConstants.kPivotLeadPort, MotorType.kBrushless);
    pivotFollower = new SparkMax(IntakeConstants.kPivotFollowerPort, MotorType.kBrushless);

    roller.setCANTimeout(250);
    pivotLeader.setCANTimeout(250);
    pivotFollower.setCANTimeout(250);

    
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig
    .idleMode(IdleMode.kBrake)
    .follow(IntakeConstants.kPivotLeadPort, true)
    .closedLoop
      .p(IntakeConstants.kP)
      .i(IntakeConstants.kI)
      .d(IntakeConstants.kD);
    
    pivotFollower.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotConfig.disableFollowerMode().inverted(false);
    pivotLeader.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotController = pivotLeader.getClosedLoopController();
    encoder = pivotLeader.getEncoder();

    encoder.setPosition(0);
  }

  public void moveTo(double angle) {
    pivotController.setSetpoint(angle, ControlType.kPosition);
  }

  public void roller(boolean on) {
    roller.set(on ? IntakeConstants.rollerSpeed : 0);
  }

  public double getAngle() {
    return encoder.getPosition();
  }

  public void disable() {
    pivotLeader.stopMotor();
    roller.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
