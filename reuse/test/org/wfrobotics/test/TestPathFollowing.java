package org.wfrobotics.test;

import org.wfrobotics.reuse.config.RobotConfigPicker;
import org.wfrobotics.reuse.config.TankConfig;
import org.wfrobotics.reuse.subsystems.drive.SteeringController;
import org.wfrobotics.reuse.subsystems.drive.TankMaths;
import org.wfrobotics.reuse.utilities.ConsoleLogger;
import org.wfrobotics.robot.config.RobotConfig;

public class TestPathFollowing
{
    static final String pathProfile = "C:\\Users\\drlin\\Desktop\\Path.csv";

    static
    {
        RobotConfigPicker.setDefault(new FauxConfig());
    }

    public static void main(String[] args)
    {
        testMath(false);
        ConsoleLogger.getInstance().reportState();
        testSteeringController();
    }

    public static void testMath(boolean dump)
    {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();
        double vector = 12.0;
        double ticks = TankMaths.inchesToTicks(vector);
        double inches = TankMaths.ticksToInches(ticks);
        assert vector == inches;

        vector = 360.0;
        inches = TankMaths.inchesOfWheelTurning(vector);
        assert inches == config.WIDTH * Math.PI / config.SCRUB;

        vector = config.VELOCITY_MAX;
        double fps = TankMaths.ticksToFPS(vector);
        if (dump) { System.out.format("%s -> %.1f ft/s\n", vector, fps); }
        assert Math.abs(fps - 10.0) < 1.0;
        ticks = TankMaths.fpsToTicks(fps);
        assert Math.abs(ticks - vector) < 0.01 * vector;
        inches = TankMaths.ticksToInchesPerSecond(vector);
        if (dump) { System.out.format("%s -> %.1f in/s\n", vector, inches); }
        assert Math.abs(fps * 12.0 - inches) < 0.01 * vector;
        double ticks2 = TankMaths.inchesPerSecondToTicks(fps * 12.0);
        assert Math.abs(ticks - ticks2) < 2;

        assert TankMaths.ticksToPercent(config.VELOCITY_MAX) == 1.0;

        if (dump) { System.out.println(""); }
    }

    public static void testSteeringController()
    {
        final TankConfig config = RobotConfig.getInstance().getTankConfig();
        final double kP = 0.015;
        final double kI = 0.0;
        final SteeringController follower = new SteeringController(kP, kI);
        final double saturatedError = follower.kStopSteeringDegrees;
        final double saturatedSteer = kP * saturatedError * config.VELOCITY_MAX;
        final double dt = 0.05;

        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, -(saturatedError + 1), false) ==  saturatedSteer;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, -saturatedError, false) == saturatedSteer;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, -saturatedError / 2.0, false) == saturatedSteer / 2.0;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, 0.0, false) == 0.0;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, saturatedError / 2.0, false) == -saturatedSteer / 2.0;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, saturatedError, false) == -saturatedSteer;
        follower.reset(0.0);
        assert follower.correctHeading(dt, 0.0, saturatedError + 1, false) == -saturatedSteer;
    }

    private static class FauxConfig extends RobotConfig
    {
        public TankConfig getTankConfig()
        {
            TankConfig c = new TankConfig();
            c.VELOCITY_MAX = 10500.0;
            c.GEAR_RATIO_HIGH = (36.0 / 15.0) * (24.0 / 40.0);
            c.GEAR_RATIO_LOW = (36.0 / 15.0) * (40.0 / 24.0);
            c.SCRUB = .96;
            c.WHEEL_DIAMETER = 6.25;
            c.WIDTH = 24.0;
            return c;
        }
    }
}