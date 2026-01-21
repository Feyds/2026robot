package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private static Limelight instance;
    NetworkTable table;

    private static final double aligntolerancedeg = 1.0;

    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static Limelight getInstance() {
        if(instance == null) {
            instance = new Limelight();
        }
        return instance;
    }

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public double getTx() {
        return table.getEntry("tx").getDouble(0.0);
    }

    public double getTy() {
        return table.getEntry("ty").getDouble(0.0);
    }

    public int getTagID() {
        return (int) table.getEntry("tid").getDouble(-1);
    }

    public double getDistanceMeters() {
        double[] pose = table
        .getEntry("botpose_targetspace")
        .getDoubleArray(new double[6]);

        double x = pose[0];
        double z = pose[2];

        return Math.sqrt(x*x + z*z);
    }

    public boolean isAligned() {
        return hasTarget() && Math.abs(getTx()) < aligntolerancedeg;
    }
}
