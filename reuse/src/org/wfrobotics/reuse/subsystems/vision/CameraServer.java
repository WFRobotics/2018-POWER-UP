package org.wfrobotics.reuse.subsystems.vision;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.List;

import org.wfrobotics.reuse.utilities.CrashTrackingRunnable;

import edu.wpi.first.wpilibj.Timer;

/** Handles connecting and servicing {@link Socket} with Kangaroo vision coprocessor */
public class CameraServer extends CrashTrackingRunnable
{
    /** Callback for receiving each new {@link CoprocessorData} */
    public static interface CameraListener
    {
        void Notify(CoprocessorData message);
    }

    public static CameraServer getInstance()
    {
        if (instance == null)
        {
            instance = new CameraServer();
        }
        return instance;
    }

    private final static int kDefaultPort = 5801;

    private static CameraServer instance = null;
    private ServerSocket serverSocket;
    private List<CameraListener> listeners = new ArrayList<CameraListener>();

    private CameraServer()
    {
        try
        {
            serverSocket = new ServerSocket(kDefaultPort);
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        new Thread(this, getClass().getSimpleName()).start();
    }

    /** The {@link CameraServer} thread kicks off a {@link Connection} when the coprocessor is detected */
    public void runCrashTracked()
    {
        System.out.println("Waiting for clients to connect at " + serverSocket.getInetAddress().getHostAddress() + ":" + serverSocket.getLocalPort());
        while (true)
        {
            try
            {
                Socket s = serverSocket.accept();
                Connection com = new Connection(s);
                new Thread(com, com.getClass().getSimpleName()).start();
            }
            catch (IOException e)
            {
                System.err.println("Cannot accept socket connection");
            }
            finally
            {
                try
                {
                    Thread.sleep(100);  // Save CPU in between looking for coprocessor
                }
                catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    /** Subscribe to receiving each new {@link CoprocessorData} from the coprocessor*/
    public synchronized void register(CameraListener toAdd)
    {
        listeners.add(toAdd);
    }

    /** Thread which handles communicating to a single connected {@link Socket} */
    private class Connection extends CrashTrackingRunnable
    {
        private final Socket socket;
        private final boolean javaTimestamps;

        private Connection(Socket client)
        {
            socket = client;
            javaTimestamps = System.getProperty("os.name").contains("Windows");
        }

        public void runCrashTracked()
        {
            if (socket == null)
            {
                return;
            }
            try
            {
                System.out.println("Connected to a client at " + socket.getInetAddress().getHostAddress());
                BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                String msg;

                while (socket.isConnected() && (msg = in.readLine()) != null)
                {
                    final CoprocessorData m = new CoprocessorData(msg, getTimestamp());
                    if (m.isValid())
                    {
                        for (CameraListener l : listeners)
                        {
                            l.Notify(m);
                        }
                    }
                }
                System.out.println("Socket disconnected");
            }
            catch (IOException e)
            {
                System.err.println("Could not talk to socket");
            }
            teardown();
        }

        private double getTimestamp()
        {
            if (javaTimestamps)
            {
                return System.currentTimeMillis();  // ms since epoch - java testing
            }
            return Timer.getFPGATimestamp();  // ms - RIO
        }

        private void teardown()
        {
            if (socket != null)
            {
                try
                {
                    socket.close();
                }
                catch(IOException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
