package frc.robot.controller;

import edu.wpi.first.wpilibj.PS5Controller;

public class Driver {
    public static Driver instance;
    private final PS5Controller controller;

    public Driver(int port) {
        controller = new PS5Controller(port);
    }

    public static Driver getInstance(int port) {
        if(instance == null) {
            instance = new Driver(port);
        }
        return instance;
    }

    public double getForwardSpeed() {
        return -controller.getLeftY();
    }

    public double getStrafeSpeed() {
        return -controller.getLeftX();
    }

    public double getRotationSpeed() {
        return -controller.getRightX();
    }

    public boolean isResetGyroButtonPressed() {
        return controller.getCrossButton();
    }

    public boolean isSlowMode() {
        return controller.getL1ButtonPressed();
    }
}
