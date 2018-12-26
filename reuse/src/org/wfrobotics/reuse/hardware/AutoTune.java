package org.wfrobotics.reuse.hardware;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.function.Consumer;

import org.wfrobotics.reuse.config.TalonConfig.ClosedLoopConfig;
import org.wfrobotics.reuse.config.TalonConfig.Gains;
import org.wfrobotics.reuse.utilities.Reportable;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.ConnectionNotification;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.networktables.TableListener;
import edu.wpi.first.wpilibj.DriverStation;

/** @author STEM Alliance of Fargo Moorhead */
public class AutoTune implements TableListener, Consumer<ConnectionNotification>, Reportable
{
    private final static String TABLE_NAME = "AutoTune";
    private final static String MOTOR_NAME = "Motor";
    private final static String PID_NAME = "PID";

    private static interface UpdateGainsListener
    {
        void UpdateGains(Gains gains);
    }

    public static class SendableMotor implements TableListener, TableEntryListener, UpdateGainsListener
    {
        public List<TalonSRX> Motor;
        public String Name;
        public int Slot;
        public String Mode;
        public Map<Integer, SendablePID> PIDs;
        public Map<String, Double> Values;

        public ControlMode SetMode;
        public double SetValue;

        /**
         * Build a new motor
         * @param Motor
         * @param Name
         * @param parent
         * @param Listener
         */
        public SendableMotor(List<TalonSRX> Motor, String Name, NetworkTable parent)
        {
            this.Motor = new ArrayList<TalonSRX>();
            this.Motor.addAll(Motor);
            this.Name = Name;
            parent.getSubTable(Key()).getEntry("Name").setString(Name);
            PIDs = new HashMap<Integer, SendablePID>();
            Values = new HashMap<String, Double>();
        }

        public static int GetKeyFromTableName(String n)
        {
            return Integer.parseInt(n.replace(MOTOR_NAME, ""));
        }

        public String Key()
        {
            return MOTOR_NAME + Motor.get(0).getDeviceID();
        }

        public void UpdateGains(Gains gains)
        {
            for (TalonSRX Motors : Motor)
            {
                Motors.config_kP(gains.kSlot, gains.kP, 0);
                Motors.config_kI(gains.kSlot, gains.kI, 0);
                Motors.config_kD(gains.kSlot, gains.kD, 0);
                Motors.config_kF(gains.kSlot, gains.kF, 0);
                Motors.config_IntegralZone(gains.kSlot, gains.kIZone, 0);
            }
        }

        /**
         * Add a new PID slot for this motor
         * @param pidName
         * @param gains
         * @param Table
         */
        public void AddPID(Gains gains, NetworkTable Table)
        {
            SendablePID pid = new SendablePID(gains.kSlot, gains.name, gains, Table, this);
            PIDs.put(gains.kSlot, pid);
            Table.getSubTable(PID_NAME + gains.kSlot).addEntryListener(pid, EntryListenerFlags.kUpdate| EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kLocal);
            pid.Set(gains, Table.getSubTable(PID_NAME + gains.kSlot));
        }

        @Override
        public void tableCreated(NetworkTable parent, String key, NetworkTable table)
        {
            if(parent.getPath().endsWith(Key()))
            {
                if(key.startsWith(PID_NAME))
                {
                    int keyNum = SendablePID.GetKeyFromTableName(key);
                    if(!PIDs.containsKey(keyNum))
                    {
                        // new motor from the App
                        PIDs.put(keyNum, new SendablePID(keyNum, key, new Gains(key, keyNum,0,0,0,0,0), parent, this));
                        table.addEntryListener(PIDs.get(keyNum), EntryListenerFlags.kUpdate | EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kLocal);
                    }
                }
            }
        }

        @Override
        public void valueChanged(NetworkTable parent, String key, NetworkTableEntry entry, NetworkTableValue value, int flags)
        {
            if(parent.getPath().endsWith(Key()))
            {
                if(key.equals("Name"))
                {

                }
                else if(key.equals("Slot"))
                {

                }
                else if(key.equals("Mode"))
                {

                }
                else if(key.equals("Set_Slot"))
                {
                    if(DriverStation.getInstance().isTest())
                    {
                        for (TalonSRX talonSRX : Motor)
                        {
                            talonSRX.selectProfileSlot((int)value.getDouble(), 0);
                        }
                    }
                }
                else if(key.equals("Set_Mode"))
                {
                    int mode = (int)value.getDouble();

                    ControlMode[] Modes = ControlMode.values();

                    for(int i = 0; i < Modes.length; i++)
                    {
                        if(Modes[i].value == mode)
                            SetMode = Modes[i];
                    }
                }
                else if(key.equals("Set_Value"))
                {
                    SetValue = value.getDouble();

                    if(DriverStation.getInstance().isTest())
                    {
                        for (TalonSRX talonSRX : Motor)
                        {
                            talonSRX.set(SetMode, SetValue);
                        }
                    }
                }
                else
                {
                    Values.put(key, value.getDouble());
                }
            }
        }
    }

    public static class SendablePID implements TableEntryListener
    {
        public final int Slot;
        public String Name;
        public Map<String, Double> Values;
        public UpdateGainsListener Listener;

        /**
         * Build the PID using the default "PID#" name
         * @param Name
         * @param table
         * @param Listener
         */
        public SendablePID(String Name, NetworkTable table, UpdateGainsListener Listener)
        {
            Slot = GetKeyFromTableName(Name);
            this.Name = Name;
            Values = new HashMap<String, Double>();
            Set(table);
            this.Listener = Listener;
        }

        /**
         * Build the PID using a custom name and PID#
         * @param Key
         * @param Name
         * @param gains
         * @param Listener
         * @param parent
         */
        public SendablePID(int Key, String Name, Gains gains, NetworkTable parent, UpdateGainsListener Listener)
        {
            Slot = Key;
            this.Name = Name;
            parent.getSubTable(Key()).getEntry("Name").setString(Name);
            Values = new HashMap<String, Double>();
            Set(gains, parent.getSubTable(Key()));
            this.Listener = Listener;
        }

        public static int GetKeyFromTableName(String n)
        {
            return Integer.parseInt(n.replace(PID_NAME, ""));
        }

        public String Key()
        {
            return PID_NAME + Slot;
        }

        public void Set(Gains gains, NetworkTable parent)
        {
            //Slot = gains.kSlot; parent.getEntry("Slot").setDouble(gains.kSlot);
            Values.put("P", gains.kP); parent.getEntry("P").setDouble(gains.kP);
            Values.put("I", gains.kI); parent.getEntry("I").setDouble(gains.kI);
            Values.put("D", gains.kD); parent.getEntry("D").setDouble(gains.kD);
            Values.put("F", gains.kF); parent.getEntry("F").setDouble(gains.kF);
            Values.put("IZone", (double) gains.kIZone); parent.getEntry("IZone").setDouble(gains.kIZone);
        }

        public void Set(NetworkTable table)
        {
            Name = table.getEntry("Name").getString("");
            //Slot = (int) table.getEntry("Slot").getDouble(0);
            Values.put("P", table.getEntry("P").getDouble(0));
            Values.put("I", table.getEntry("I").getDouble(0));
            Values.put("D", table.getEntry("D").getDouble(0));
            Values.put("F", table.getEntry("F").getDouble(0));
            Values.put("IZone", table.getEntry("IZone").getDouble(0));
        }

        @Override
        public void valueChanged(NetworkTable parent, String key, NetworkTableEntry entry, NetworkTableValue value,
                                 int flags)
        {
            if(parent.getPath().endsWith(Key()))
            {
                if(key.equals("Name"))
                {
                    Name = value.getString();
                }
                else if(key.equals("Slot"))
                {
                    // Slot = (int) value.getDouble();
                }
                else
                {
                    if(value.isDouble())
                    {
                        Values.put(key, value.getDouble());
                    }
                }

                if(Listener != null)
                {
                    double P     = Values.containsKey("P") ? Values.get("P") : 0;
                    double I     = Values.containsKey("I") ? Values.get("I") : 0;
                    double D     = Values.containsKey("D") ? Values.get("D") : 0;
                    double F     = Values.containsKey("F") ? Values.get("F") : 0;
                    double IZone = Values.containsKey("IZone") ? Values.get("IZone") : 0;

                    Listener.UpdateGains(new Gains(Name, Slot,P,I,D,F,(int) IZone));
                }
            }
        }
    }

    // list of listeners to notify when receiving messages
    private HashMap<Integer,SendableMotor> motors = new HashMap<Integer,SendableMotor>();

    private static AutoTune instance = null;
    private NetworkTable Table;
    private boolean IsConnected;

    /**
     * Get the instance of the TargetServer at the default port {@value #DEFAULT_PORT}
     * @return
     */
    public static AutoTune getInstance()
    {
        if (instance == null)
        {
            instance = new AutoTune();
        }
        return instance;
    }

    private AutoTune()
    {
        Table = NetworkTableInstance.getDefault().getTable(TABLE_NAME);
        Table.addSubTableListener(this, true);

        IsConnected = false;
        NetworkTableInstance.getDefault().addConnectionListener(this, true);
    }

    @Override
    public void tableCreated(NetworkTable parent, String motor, NetworkTable table)
    {
        if(parent.getPath().endsWith(TABLE_NAME))
        {
            if(!motors.containsKey(SendableMotor.GetKeyFromTableName(motor)))
            {
                // new motor from the App
                // No longer supported
                // motors.put(motor, new SendableMotor(motor, table.getEntry("Name").getString(motor), Table));
                // table.addSubTableListener(motors.get(motor), true);
                // Table.getSubTable(motor).addEntryListener(motors.get(motor), EntryListenerFlags.kUpdate| EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kLocal);
            }
        }
    }

    /**
     * Add a new motor
     * @param t
     * @param motorName
     * @param listener
     * @return
     */
    public String AddMotor(List<TalonSRX> t, String motorName)
    {
        int addr = t.get(0).getDeviceID();
        String motorKey = MOTOR_NAME + addr;

        if(!motors.containsKey(addr))
        {
            // new motor from the robot
            motors.put(addr, new SendableMotor(t, motorName, Table));

            Table.getSubTable(motorKey).addSubTableListener(motors.get(addr), true);
            Table.getSubTable(motorKey).addEntryListener(motors.get(addr), EntryListenerFlags.kUpdate| EntryListenerFlags.kNew | EntryListenerFlags.kImmediate | EntryListenerFlags.kLocal);
        }

        return motorKey;
    }

    /** Register a tunable group of motors and all PID gains available for gain scheduling on the group */
    public void AddAllTheGains(List<TalonSRX> t, ClosedLoopConfig config)
    {
        for (Gains g : config.gains)
        {
            AddMotor(t, config.name, g);
        }
    }

    /**
     * Add a new motor and PID gains
     * @param t
     * @param motorName
     * @param gains
     * @param pidName
     */
    public void AddMotor(List<TalonSRX> t, String motorName, Gains gains)
    {
        int addr = t.get(0).getDeviceID();

        String motorKey = AddMotor(t, motorName);

        if(!motors.get(addr).PIDs.containsKey(gains.kSlot))
        {
            // new pid from the robot
            motors.get(addr).AddPID(gains, Table.getSubTable(motorKey));
        }
        else
        {
            // update existing pid from the robot
            SendablePID pid = motors.get(addr).PIDs.get(gains.kSlot);
            pid.Set(gains, Table.getSubTable(motorKey).getSubTable(pid.Key()));
        }
    }

    public void reportState()
    {
        if(IsConnected)
        {
            for(Entry<Integer, SendableMotor> t : motors.entrySet())
            {
                SendableMotor motor = t.getValue();

                Map<String, Double> values = new HashMap<String, Double>();
                values.put("Error", (double) motor.Motor.get(0).getClosedLoopError(0));
                values.put("Velocity", (double) motor.Motor.get(0).getSelectedSensorVelocity(0));

                for(Map.Entry<String, Double> value : values.entrySet())
                {
                    Table.getSubTable(motor.Key()).getEntry(value.getKey()).setNumber(value.getValue());
                }
            }

            Table.getEntry("IsTest").setBoolean(DriverStation.getInstance().isTest());
        }
    }

    public void reportState(TalonSRX motor, int Slot, String Mode, Map<String,Double> Values)
    {
        if(IsConnected)
        {
            SendableMotor m = motors.get(motor.getDeviceID());

            if(m != null)
            {
                String key = m.Key();

                for(Map.Entry<String, Double> value : Values.entrySet())
                {
                    Table.getSubTable(key).getEntry(value.getKey()).setNumber(value.getValue());
                }
                Table.getSubTable(key).getEntry("Slot").setNumber(Slot);
                Table.getSubTable(key).getEntry("Mode").setString(Mode);
            }
        }
    }

    @Override
    public void accept(ConnectionNotification t)
    {
        ConnectionInfo conn = t.conn;
        if(conn.remote_id.equals("AutoTune"))
        {
            IsConnected = t.connected;
        }
    }
}
