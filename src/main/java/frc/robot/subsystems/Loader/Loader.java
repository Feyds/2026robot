// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Loader;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Loader extends SubsystemBase {

  SparkFlex motor;

  /** Creates a new Loader. */
  public Loader() {
    motor = new SparkFlex(LoaderConstants.kMotorPort, MotorType.kBrushless);
  }

  public void run(boolean on) {
    motor.set(on ? LoaderConstants.speed : 0);
  }

  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
