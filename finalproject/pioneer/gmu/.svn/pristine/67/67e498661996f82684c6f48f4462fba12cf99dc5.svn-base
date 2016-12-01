/*
 * SerialCommunication.java
 */

package gmu.robot.pioneer.serial;

import java.io.*;
import java.util.*;
import javax.comm.*;

/**
 * This class simplifies the use of the serial port. You need the javax.comm
 * package before being able to compile and run this code.
 * <P>
 * <A HREF="../COPYRIGHT.html">Copyright</A>
 * (c)2001 Evolutinary Computation Laboratory @ George Mason University
 *
 * @author Liviu Panait
 * @version 1.0
 */

public class SerialCommunication
    {
    boolean initialized = false;

    // the serial port
    SerialPort serialPort;

    // the input and the output streams to the serial port
    OutputStream outputStream;
    InputStream inputStream;

    public SerialCommunication( String port )
        {
        CommPortIdentifier cpi;

        initialized = true;

        try
            {
            cpi = CommPortIdentifier.getPortIdentifier( port );
            // System.err.println( "Communication port obtained successfully." );
            }
        catch( NoSuchPortException e )
            {
            System.err.println( "Failed obtaining communication port." );
            initialized = false;
            e.printStackTrace();
            return;
            }

        try
            {
            serialPort = (SerialPort)cpi.open( "SerialCommunication", 2000 );
            //System.err.println( "Serial port opened successfully." );
            }
        catch( PortInUseException e )
            {
            System.err.println( "Failed opening serial port." );
            initialized = false;
            e.printStackTrace();
            return;
            }

        try
            {
            // Setting the port parameters.
            serialPort.setSerialPortParams(9600,
                SerialPort.DATABITS_8,
                SerialPort.STOPBITS_1,
                SerialPort.PARITY_NONE);         
            //System.err.println( "Communication parameters set successfully." );
            }
        catch( UnsupportedCommOperationException e )
            {
            System.err.println( "Failed setting communication parameters." );
            initialized = true;
            e.printStackTrace();
            return;
            }

        try
            {
            outputStream = serialPort.getOutputStream();
            inputStream = serialPort.getInputStream();
            //System.err.println( "Input and output streams obtained successfully." );
            }
        catch( IOException e )
            {
            System.err.println( "Failed obtaining input or output stream." );
            initialized = false;
            e.printStackTrace();
            return;
            }

        }

    public OutputStream getOutputStream() throws IOException
        {
        if( initialized )
            {
            return outputStream;
            }
        else
            {
            throw new IOException( "Communication not initialized." );
            }
        }

    public InputStream getInputStream() throws IOException
        {
        if( initialized )
            {
            return inputStream;
            }
        else
            {
            throw new IOException( "Communication not initialized." );
            }
        }

    public void shutDown()
        {
        if( initialized )
            {
            serialPort.close();
            }
        }

    }
