package org.wfrobotics.test;

import java.util.logging.Level;

import org.wfrobotics.reuse.math.HerdVector;
import org.wfrobotics.reuse.utilities.ConsoleLogger;

// TODO cross broken? parallel should have mag zero

/** To enable unit tests: Tests.java->Properties->Run/debug settings->Tests->Edit->Arguments->VM args->-ea */
public class Tests
{
    static int settledSamples = 0;
    static final double width = 4;
    static final double depth = 3;

    static double maxWheelMagnitudeLast;
    static HerdVector wheelBR;
    static HerdVector wheelFR;
    static HerdVector wheelFL;
    static HerdVector wheelBL;
    static HerdVector rBR;
    static HerdVector rFR;
    static HerdVector rFL;
    static HerdVector rBL;

    public static void main(String[] args)
    {
        Vector robotV = Vector.NewFromMagAngle(1, 5);
        double robotS = -.1;
        debugOldChassisToWheelVectors(robotV, robotS);
        printDivider();

        HerdVector v = new HerdVector(robotV.getMag(), robotV.getAngle() + 90);
        double spin = robotS;
        debugNewChassisToWheelVectors(v, spin);

        //debugWheelVectorsToChassis();
        //printDivider();

        printDivider();

        debugLogging();
        printDivider();
    }

    public static HerdVector[] debugNewChassisToWheelVectors(HerdVector v, double spin)
    {
        double start, end;
        HerdVector w = new  HerdVector(spin, 90);
        System.out.format("Command (%.2f, %.2f, %.2f)\n\n", v.getMag(), v.getAngle(), spin);

        HerdVector rWidth = new HerdVector(width, 90);
        HerdVector rHeight = new HerdVector(depth, 0);

        rFR = rWidth.add(rHeight);
        rBR = rWidth.sub(rHeight);
        rFL = rWidth.rotate(180).add(rHeight);
        rBL = rWidth.rotate(180).sub(rHeight);

        double unitVectorCorrection = 1 / rBR.getMag();
        rFR = rFR.scale(unitVectorCorrection);
        rFL = rFL.scale(unitVectorCorrection);
        rBR = rBR.scale(unitVectorCorrection);
        rBL = rBL.scale(unitVectorCorrection);

        System.out.println("FR: " + rFR);
        System.out.println("FL: " + rFL);
        System.out.println("BR: " + rBR);
        System.out.println("BL: " + rBL);
        System.out.println();

        //        System.out.println("Cross FR: " + w.cross(rFR));
        //        System.out.println("Cross FL: " + w.cross(rFL));
        //        System.out.println("Cross BR: " + w.cross(rBR));
        //        System.out.println("Cross BL: " + w.cross(rBL));
        //        System.out.println();

        start = System.nanoTime();

        // v + w x r
        HerdVector wheelFR = v.add(w.cross(rFR));
        HerdVector wheelFL = v.add(w.cross(rFL));
        HerdVector wheelBR = v.add(w.cross(rBR));
        HerdVector wheelBL = v.add(w.cross(rBL));

        double maxMag = wheelFR.getMag();
        maxMag = (wheelFL.getMag() > maxMag) ? wheelFL.getMag(): maxMag;
        maxMag = (wheelBR.getMag() > maxMag) ? wheelBR.getMag(): maxMag;
        maxMag = (wheelBL.getMag() > maxMag) ? wheelBL.getMag(): maxMag;

        if (maxMag > 1)
        {
            wheelFR = wheelFR.scale(1 / maxMag);
            wheelFL = wheelFL.scale(1 / maxMag);
            wheelBR = wheelBR.scale(1 / maxMag);
            wheelBL = wheelBL.scale(1 / maxMag);
            Tests.maxWheelMagnitudeLast = maxMag;
        }
        else
        {
            Tests.maxWheelMagnitudeLast = 1;
        }

        end = System.nanoTime();

        System.out.println("FR: " + wheelFR);
        System.out.println("FL: " + wheelFL);
        System.out.println("BR: " + wheelBR);
        System.out.println("BL: " + wheelBL);
        System.out.println();

        double twist = -90;

        HerdVector oldFR = wheelFR.rotate(twist);
        HerdVector oldFL = wheelFL.rotate(twist);
        HerdVector oldBR = wheelBR.rotate(twist);
        HerdVector oldBL = wheelBL.rotate(twist);

        double mirrorY = -2;

        oldFR = oldFR.rotate(oldFR.getAngle() * mirrorY);
        oldFL = oldFL.rotate(oldFL.getAngle() * mirrorY);
        oldBR = oldBR.rotate(oldBR.getAngle() * mirrorY);
        oldBL = oldBL.rotate(oldBL.getAngle() * mirrorY);

        System.out.format("Translate (Twist %.0f, Mirror Y)\n", twist);
        System.out.println("FR old: " + oldFR);
        System.out.println("FL old: " + oldFL);
        System.out.println("BR old: " + oldBR);
        System.out.println("BL old: " + oldBL);
        System.out.println();

        System.out.println("Wheel Calcs: " + ((end - start)/1000) + " ns");
        System.out.format("Wheel Mag wanted: %.2f\n", Tests.maxWheelMagnitudeLast);

        Tests.wheelFR = wheelFR;
        Tests.wheelFL = wheelFL;
        Tests.wheelBR = wheelBR;
        Tests.wheelBL = wheelBL;


        HerdVector[] wheelCommands = new HerdVector[4];

        wheelCommands[0] = wheelFR;
        wheelCommands[1] = wheelFL;
        wheelCommands[2] = wheelBR;
        wheelCommands[3] = wheelBL;

        return wheelCommands;
    }

    public static void debugWheelVectorsToChassis()
    {
        double maxMag = maxWheelMagnitudeLast;
        double start = System.nanoTime();

        // Undo Mirror
        HerdVector reconstructWheelBR = new HerdVector(wheelBR.getMag(), -wheelBR.getAngle());
        HerdVector reconstructWheelFR = new HerdVector(wheelFR.getMag(), -wheelFR.getAngle());
        HerdVector reconstructWheelFL = new HerdVector(wheelFL.getMag(), -wheelFL.getAngle());
        HerdVector reconstructWheelBL = new HerdVector(wheelBL.getMag(), -wheelBL.getAngle());

        //        System.out.println("UnMirrored BR: " + reconstructWheelBR);
        //        System.out.println("UnMirrored FR: " + reconstructWheelFR);
        //        System.out.println("UnMirrored FL: " + reconstructWheelFL);
        //        System.out.println("UnMirrored BL: " + reconstructWheelBL);

        // Undo scale
        reconstructWheelBR = reconstructWheelBR.scale(maxMag);
        reconstructWheelFR = reconstructWheelFR.scale(maxMag);
        reconstructWheelFL = reconstructWheelFL.scale(maxMag);
        reconstructWheelBL = reconstructWheelBL.scale(maxMag);

        // v + w x r
        //        System.out.println("Unscaled BR: " + reconstructWheelBR);
        //        System.out.println("Unscaled FR: " + reconstructWheelFR);
        //        System.out.println("Unscaled FL: " + reconstructWheelFL);
        //        System.out.println("Unscaled BL: " + reconstructWheelBL);

        // Summing the wheel vectors, the w x r's components cancel, leaving the chassis command scaled by four
        HerdVector frankenstein = new HerdVector(reconstructWheelBR);
        frankenstein = frankenstein.add(reconstructWheelFR);
        frankenstein = frankenstein.add(reconstructWheelFL);
        frankenstein = frankenstein.add(reconstructWheelBL);
        frankenstein = frankenstein.scale(.25);

        double end = System.nanoTime();

        //        // w x r
        //        HerdVector reconstructWxBR = reconstructWheelBR.sub(v);
        //        HerdVector reconstructWxFR = reconstructWheelFR.sub(v);
        //        HerdVector reconstructWxFL = reconstructWheelFL.sub(v);
        //        HerdVector reconstructWxBL = reconstructWheelBL.sub(v);
        //
        //        System.out.println("Reconstruct BR: " + reconstructWxBR);
        //        System.out.println("Reconstruct FR: " + reconstructWxFR);
        //        System.out.println("Reconstruct FL: " + reconstructWxFL);
        //        System.out.println("Reconstruct BL: " + reconstructWxBL);

        HerdVector wXr = reconstructWheelBR.scale(maxMag);
        wXr = reconstructWheelBR.sub(frankenstein);

        System.out.println("Reconstructing Robot Command");
        System.out.format("Robot Command (%.2f, %.2f, %.2f)\n", frankenstein.getMag(), frankenstein.getAngle(), wXr.getMag());
        System.out.println("Wheel Calcs: " + ((end - start)/1000) + " ns");
    }

    public static void debugOldChassisToWheelVectors(Vector robotV, double robotS)
    {
        double start, end;
        // Cartesian Wheel Vectors
        double CHASSIS_WIDTH = width;
        double CHASSIS_DEPTH = depth;
        double CHASSIS_SCALE = Math.sqrt(CHASSIS_WIDTH * CHASSIS_WIDTH + CHASSIS_DEPTH * CHASSIS_DEPTH);

        double[][] POSITIONS = {
            { CHASSIS_WIDTH / CHASSIS_SCALE, -CHASSIS_DEPTH / CHASSIS_SCALE }, // back right
            { -CHASSIS_WIDTH / CHASSIS_SCALE, -CHASSIS_DEPTH / CHASSIS_SCALE }, // back left
            { CHASSIS_WIDTH / CHASSIS_SCALE, CHASSIS_DEPTH / CHASSIS_SCALE },  // front right
            { -CHASSIS_WIDTH / CHASSIS_SCALE, CHASSIS_DEPTH / CHASSIS_SCALE }}; // front left
        String[] names = {"FR","FL", "BR","BL"};
        Vector[] positions = new Vector[4];
        Vector velocity = Vector.NewFromMagAngle(robotV.getMag(), robotV.getAngle());  // Positive x-axis
        double spin = robotS;

        System.out.format("Width: %.2f, Depth: %.2f\n", CHASSIS_WIDTH, CHASSIS_DEPTH);
        System.out.format("Command (%.2f, %.2f, %.2f)\n\n", velocity.getMag(), velocity.getAngle(), spin);

        for (int index = 0; index < 4; index++)
        {
            positions[index] = new Vector(POSITIONS[index]);
            System.out.format("%s: (%.2f, %.2f)\n", names[index], positions[index].getMag(), positions[index].getAngle());
        }
        System.out.println();

        start = System.nanoTime();
        Vector[] scaled = oldScaleWheelVectors(velocity, spin, positions);

        end = System.nanoTime();
        for (int index = 0; index < scaled.length; index++)
        {
            System.out.format("%s: (%.2f, %.2f)\n", names[index], scaled[index].getMag(), scaled[index].getAngle());
        }
        System.out.println("Wheel Calcs: " + ((end - start)/1000) + " ns");
    }

    private static Vector[] oldScaleWheelVectors(Vector velocity, double spin, Vector[] positions)
    {
        Vector[] WheelsUnscaled = new Vector[4];
        Vector[] WheelsScaled = new Vector[4];
        double MaxWantedVeloc = 0;
        double VelocityRatio;

        for (int i = 0; i < 4; i++)
        {
            WheelsUnscaled[i] = new Vector(velocity.getX() - spin * positions[i].getY(),
                                            -(velocity.getY() + spin * positions[i].getX()));

            if (WheelsUnscaled[i].getMag() >= MaxWantedVeloc)
            {
                MaxWantedVeloc = WheelsUnscaled[i].getMag();
            }
        }

        VelocityRatio = oldGetVelocityLimit(MaxWantedVeloc);

        for (int i = 0; i < 4; i++)
        {
            WheelsScaled[i] = Vector.NewFromMagAngle(WheelsUnscaled[i].getMag() * VelocityRatio, WheelsUnscaled[i].getAngle());
        }

        return WheelsScaled;
    }

    public static double oldGetVelocityLimit(double MaxWantedVeloc)
    {
        double velocityMaxAvailable = 1;
        double velocityRatio = 1;

        // Determine ratio to scale all wheel velocities by
        velocityRatio = velocityMaxAvailable / MaxWantedVeloc;

        velocityRatio = (velocityRatio > 1) ? 1:velocityRatio;

        return velocityRatio;
    }

    public static void debugLogging()
    {
        double start;
        double end;
        HerdVector v = new HerdVector(1, 45);
        Level level = Level.OFF;

        System.out.println("Level: Default");
        start = System.nanoTime();
        ConsoleLogger.defcon1(1);
        ConsoleLogger.info(.33333);
        ConsoleLogger.error(v);
        ConsoleLogger.warning(v);
        ConsoleLogger.getInstance().reportState();
        end = System.nanoTime();
        System.out.println("Test Duration: " + ((end - start)/1000) + " ns");
        System.out.println("");

        level = Level.INFO;
        System.out.println("Level: " + level.toString());
        ConsoleLogger.setLevel(level);

        start = System.nanoTime();
        ConsoleLogger.defcon1(1);
        ConsoleLogger.info(.33333);
        ConsoleLogger.error(v);
        ConsoleLogger.warning(v);
        ConsoleLogger.getInstance().reportState();
        end = System.nanoTime();
        System.out.println("Test Duration: " + ((end - start)/1000) + " ns");
        assert (end - start)/1000 < 500;

        level = Level.WARNING;
        System.out.println("Level: " + level.toString() + "\n");
        ConsoleLogger.setLevel(level);

        start = System.nanoTime();
        ConsoleLogger.defcon1(1);
        ConsoleLogger.info(.33333);
        ConsoleLogger.error(v);
        ConsoleLogger.warning(v);
        ConsoleLogger.getInstance().reportState();
        end = System.nanoTime();
        System.out.println("Test Duration: " + ((end - start)/1000) + " ns");
        assert (end - start)/1000 < 500;

        level = Level.SEVERE;
        System.out.println("Level: " + level.toString());
        ConsoleLogger.setLevel(level);

        System.out.println("");
        start = System.nanoTime();
        ConsoleLogger.defcon1(1);
        ConsoleLogger.info(.33333);
        ConsoleLogger.error(v);
        ConsoleLogger.warning(v);
        ConsoleLogger.getInstance().reportState();
        end = System.nanoTime();
        System.out.println("Test Duration: " + ((end - start)/1000) + " ns");
        assert (end - start)/1000 < 500;
    }

    public static void printDivider()
    {
        System.out.println("-------------------------------\n");
        System.out.println("-------------------------------");
    }

    private static class Vector
    {
        private double mag;
        private double ang;

        public Vector()
        {
            mag = 0;
            ang = 0;
        }

        public Vector(double[] position)
        {
            setXY(position[0], position[1]);
        }

        public Vector(double x, double y)
        {
            setXY(x, y);
        }

        public static Vector NewFromMagAngle(double mag, double ang)
        {
            Vector r = new Vector();
            r.mag = mag;
            r.ang = ang;
            return r;
        }

        public void setXY(double x, double y)
        {
            mag = Math.sqrt(x * x + y * y);
            ang = Math.toDegrees(Math.atan2(y, x));
        }

        public double getX()
        {
            double realAngle = Math.toRadians(wrapToRange(ang, 0, 360));
            return Math.cos(realAngle) * mag;
        }

        public double getY()
        {
            double realAngle = Math.toRadians(wrapToRange(ang, 0, 360));
            return Math.sin(realAngle) * mag;
        }

        public double getMag()
        {
            return mag;
        }

        public double getAngle()
        {
            return ang;
        }

        public Vector clone()
        {
            return Vector.NewFromMagAngle(mag, ang);
        }

        public static final double wrapToRange(double value, double min, double max)
        {
            return wrapToRange(value - min, max - min) + min;
        }

        public static final double wrapToRange(double value, double max)
        {
            return ((value % max) + max) % max;
        }
    }
}
