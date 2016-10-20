package gmu.robot.pioneer;

import java.io.*;

public class NonClosingInputStream extends FilterInputStream
    {
    public NonClosingInputStream(InputStream stream) { super(stream); }
    public void close()
        {
        in = null; 
        } /* don't close the underlying stream */
    }
