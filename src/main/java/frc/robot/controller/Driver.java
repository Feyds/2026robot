package frc.robot.controller;

import edu.wpi.first.wpilibj.PS5Controller;

public class Driver {

    public static Driver instance;
    private final PS5Controller controller;

    public Driver(int port) {
        controller = new PS5Controller(port);
    }

    public static Driver getInstance(int port) {
        if (instance == null) {
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


    public boolean isIntakePressed() {
        return controller.getCrossButton();
    }

    public boolean isShootPressed() {
        return controller.getCircleButton();
    }

    public boolean isIntakeStowPressed() {
        return controller.getSquareButton();
    }

    public boolean isClimbPressed() {
        return controller.getTriangleButton();
    }


    public boolean isManualLoaderPressed() {
        return controller.getR1Button();
    }

    public boolean isEmergencyStopPressed() {
        return controller.getL1Button();
    }


    public boolean isSlowMode() {
        return controller.getL2Axis() > 0.4;
    }


    public boolean isResetGyroButtonPressed() {
        return controller.getPSButton();
    }
}