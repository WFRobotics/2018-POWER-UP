package org.wfrobotics.reuse.commands.debug;

import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.math.physics.DriveCharacterization;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class CharacterizeDrivetrain extends CommandGroup
{
    List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
    List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

    public CharacterizeDrivetrain()
    {
        addSequential(new CollectVelocityData(velocityData, false, true));
        addSequential(new WaitCommand(10.0));
        addSequential(new CollectAccelerationData(accelerationData, false, true));
    }

    protected void end()
    {
        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}
