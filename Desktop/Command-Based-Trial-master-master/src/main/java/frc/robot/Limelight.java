package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ArmSubsystem;


public class Limelight {
    
    public final double limelightHeight;
    public final double targetHeight1;
    public final double targetHeight2;
    public final double postHeight1;
    final double limelightAngle;
    public final double armHeight;
    public final double postHeight2;
    double xOff, yOff, areaValue, validity, targetDistance;

    NetworkTable limelightTable;
    NetworkTableEntry xOffEntry, yOffEntry, areaValueEntry, validityEntry;

    /**
     * Constructs a limelight object
     * @param limelightHeight Vertical distance between the limelight and the ground in inches
     * @param targetHeight Verticl distance between the target and the ground in inches
     * @param limelightAngle Angle between the limelight and level (from limelight's height)
     */
    public Limelight (double limelightHeight, double targetHeight1, double targetHeight2, double postHeight1, double postHeight2, double limelightAngle, double armHeight) {

        this.postHeight1 = postHeight1;
        this.postHeight2 = postHeight2;
        this.limelightHeight = limelightHeight;
        this.targetHeight1 = targetHeight1;
        this.targetHeight2 = targetHeight2;
        this.limelightAngle = limelightAngle;
        this.armHeight = armHeight;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        xOffEntry = limelightTable.getEntry("tx");
        yOffEntry = limelightTable.getEntry("ty");
        areaValueEntry = limelightTable.getEntry("ta");
        validityEntry = limelightTable.getEntry("tv");

    }

    /**
     * Periodically updates the limelight variables
     * @param smartDashboardDisplay Boolean to display whether or not to display smart dashboard values
     * @param targetHeight difference height to find target distance
     * @return Returns the variable tragetDistance for use
     */
    public double updateLimelightVariables (Boolean smartDashboardDisplay, double targetHeight) {

        xOff = xOffEntry.getDouble(0.0);
        yOff = yOffEntry.getDouble(0.0);
        areaValue = areaValueEntry.getDouble(0.0);
        validity = validityEntry.getDouble(0.0);
        
        if (smartDashboardDisplay) {
            SmartDashboard.putNumber("X Offset", xOff);
            SmartDashboard.putNumber("Y Offset", yOff);
            SmartDashboard.putNumber("Area (Target Size)", areaValue);
            SmartDashboard.putNumber("Validity", validity);
        }

        targetDistance = (targetHeight) / Math.tan((limelightAngle + yOff) * Math.PI / 180);

        return targetDistance;

    }

    /**
     * Method to return the required arm extension of the robot
     * Just takes the pythagorean theorem of the distance and "height"
     * @param targetHeight height difference used to find length
     * @param distance distance from robot and target, given in function above
     * @return the arm extension length (to be inputted in values array)
     */
    public double returnLength(double targetHeight, double distance) {

        double length = Math.sqrt(Math.pow(distance, 2) + Math.pow((targetHeight), 2));
        return length;

    }

    /**
     * Method to return the angle to rotate the flippy arm to
     * takes the arctan of the target heights and given distance
     * @param targetHeight height difference to find angle
     * @param distance distance from robot to target, given by returnDistance
     * @return angle, in degrees, from the horizontal of the arm to the target
     */
    public double returnAngle(double targetHeight, double distance) {

        double angle = Math.atan((targetHeight)/distance);
        return angle * 180 / Math.PI;

    }

    public double weightPowersRanges (double angle) {

        //defines rpm variable to be output and defines the index variable as 1 to start iteration
        
        int index = 0;

        //iterates through the array of distances and breaks the loop once the distance is less than an input value
        while (angle < ArmSubsystem.weightDegrees[index] && index < ArmSubsystem.weightDegrees.length) {

            index = index + 1;
        
        }

        //Fail-safe to prevent code from breaking (stops potential 0-1 below)
        if (index == 0) {

            index = 1;

        }

        //plugs the corresponding inputs and outputs into the equation for rpm output
        double power = ArmSubsystem.weightPowers[(index - 1)] + (angle - ArmSubsystem.weightDegrees[(index - 1)]) * (ArmSubsystem.weightPowers[index] - ArmSubsystem.weightPowers[(index - 1)]) / (ArmSubsystem.weightDegrees[index] - ArmSubsystem.weightDegrees[(index - 1)]);

        //returns rpm for outside use
        return power;

    }

    public double flipEncoderRanges(double angle){

        //defines rpm variable to be output and defines the index variable as 1 to start iteration
        
        int index = 0;

        //iterates through the array of distances and breaks the loop once the distance is less than an input value
        while (angle < ArmSubsystem.degreesPosition[index] && index < ArmSubsystem.degreesPosition.length) {

            index = index + 1;
        
        }

        //Fail-safe to prevent code from breaking (stops potential 0-1 below)
        if (index == 0) {

            index = 1;

        }

        //plugs the corresponding inputs and outputs into the equation for rpm output
        double encoderRotate = ArmSubsystem.encoderPosition[(index - 1)] + (angle - ArmSubsystem.degreesPosition[(index - 1)]) * (ArmSubsystem.encoderPosition[index] - ArmSubsystem.encoderPosition[(index - 1)]) / (ArmSubsystem.degreesPosition[index] - ArmSubsystem.degreesPosition[(index - 1)]);

        //returns rpm for outside use
        return encoderRotate;


    }

    public double teleEncoderRanges(double length){

        //defines rpm variable to be output and defines the index variable as 1 to start iteration
        
        int index = 0;

        //iterates through the array of distances and breaks the loop once the distance is less than an input value
        while (length < ArmSubsystem.teleLength[index] && index < ArmSubsystem.teleLength.length) {

            index = index + 1;
        
        }

        //Fail-safe to prevent code from breaking (stops potential 0-1 below)
        if (index == 0) {

            index = 1;

        }

        //plugs the corresponding inputs and outputs into the equation for rpm output
        double encoderLength = ArmSubsystem.teleEncoder[(index - 1)] + (length - ArmSubsystem.teleLength[(index - 1)]) * (ArmSubsystem.teleEncoder[index] - ArmSubsystem.teleEncoder[(index - 1)]) / (ArmSubsystem.teleLength[index] - ArmSubsystem.teleLength[(index - 1)]);

        //returns rpm for outside use
        return encoderLength;
        
    }


    /**
     * Changes what pipeline to view 
     * @param number pipeline number to access
     */
    public void setPipeline (int number) {

        NetworkTableEntry pipelineEntry = limelightTable.getEntry("pipeline");
        pipelineEntry.setNumber(number);

    }

    /**
     * Method to return the current pipeline
     * @return double value for pipeline number
     */
    public double getPipeline () {

        return limelightTable.getEntry("getpipe").getDouble(0.0);

    }
}