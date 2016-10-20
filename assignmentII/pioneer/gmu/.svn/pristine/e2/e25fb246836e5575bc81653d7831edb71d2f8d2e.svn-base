package gmu.robot.pioneer;
import java.io.*;

/**
   A simple InputStream which does not block when its underlying stream blocks.  
   Additionally, if we close the NonBlockingInputStream, we are given the option 
   to close or not close the underlying stream as well.  NonBlockingInputStream
   overrides the basic read() method, not the more advanced ones.  This means that
   even if you call the more advanced read(...) methods, they'll just call the
   underlying read() method and will be as slow as it is.  NonBlockingInputStream
   also maintains a ring buffer of a fixed size, specified by the user, which stores the
   incoming bytes.  An underlying thread constantly does blocking reads on the underlying
   stream, filling the buffer as available.  Our read() method then just returns the oldest
   item in the buffer, removing it, or throws an exception if there's nothing to return. 
*/

public class NonBlockingInputStream extends InputStream
    {
    /* The underlying input stream that we read from.  Likely blocks. */
    InputStream stream;
    
    /* How long should we wait on a read? In milliseconds. */
    int timeout;
    
    /* The underlying buffer. We also lock on the buffer to establish synchronization to reading/writing it.  */
    int[] buffer;

    /* Where we read from.  If start==end, there's nothing in the buffer. */
    int start = 0;

    /* Where we write to in the buffer. */
    int end = 0;

    /* Returned by readNoException() after we waited up to timeout time and still couldn't read anything. */
    static final int FAIL = -2;

    /* Returned when we;re at the end of the stream. */
    static final int EOF = -1;
    
    /* The underlying thread which constantly does a blocking read on the underlying input stream.  Set to be daemon, so it quits when the program dies. */
    Thread readThread;
    
    /* A flag set internally to notify the readThread that it should quit. */
    boolean pleaseDie = false;
    
    /* If true, we will call close() when we receive an underlying exception. */
    int underlyingExceptionBehavior;

    /** Indicates that on an underlying exception, we should ignore the exception. */
    public final static int BEHAVIOR_IGNORE = 0;

    /** Indicates that on an underlying exception, we should ignore the exception and print it out. */
    public final static int BEHAVIOR_IGNORE_PRINT = 1;

    /** Indicates that on an underlying exception, we should close ourselves. */
    public final static int BEHAVIOR_CLOSE = 2; 

    /** Indicates that on an underlying exception, we should close ourselves and print it out. */
    public final static int BEHAVIOR_CLOSE_PRINT = 2; 
    
    /* If true, we will close the underlying stream when we close ourselves. */
    boolean closeUnderlyingStreamWhenClosed;

    /* If true, the stream has been closed. */
    boolean streamClosed = false;
    
    /* A lock object we use to set the streamClosed variable. */
    Object streamLock = new int[0];  // arrays are serializable, unlike bare Objects

    /** Creates a new NonBlockingInputStream with a given underlying STREAM,
        a TIMEOUT in milliseconds, and a fixed BUFFER SIZE.  If, when reading
        from the NonBlockingInputStream, we cannot get another byte without blocking,
        we will first wait for TIMEOUT time, and then try again.  If we still can't get
        another byte without blocking, we will signal a timeout failure.  A larger
        BUFFER SIZE gives us more efficiency with respect to variability in availability
        of bytes from the underlying stream.  It doesn't have to be too big.  The
        UNDERLYING EXCEPTION BEHAVIOR tells us what to do when the underlying stream has
        an exception.  If it's BEHAVIOR_IGNORE, then we do a timeout and try again.  If
        it's BEHAVIOR_IGNORE_PRINT, then we do a timeout and try again, but first print the exception
        to System.err.  If it's BEHAVIOR_CLOSE, then we close ourselves.  If it's BEHAVIOR_CLOSE_PRINT
        then we close ourselves, but first print the exception to System.err.
        One common underlying exception is Linux's "EAGAIN"
        exception, which indicates that reading from a device would block.  So you might
        want to set this value to "false".  Last, if CLOSE UNDERLYING STREAM WHEN CLOSED,
        then when we ourselves are closed, we also do a close() on the underlying stream. */
    public NonBlockingInputStream(InputStream stream, int timeout, int bufferSize, int underlyingExceptionBehavior, boolean closeUnderlyingStreamWhenClosed)
        {
        buffer = new int[bufferSize+1];
        this.stream = stream;
        this.timeout = timeout;
        this.underlyingExceptionBehavior = underlyingExceptionBehavior;
        this.closeUnderlyingStreamWhenClosed = closeUnderlyingStreamWhenClosed;
        
        // fire off the read-enqueue-thread
        readThread = new Thread(new Runnable() {  public void run() { doReadLoop(); } });
        readThread.setDaemon(true);
        readThread.start();
        }

    /* Returns true if the underlying stream is closed. */
    boolean isStreamClosed()
        {
        synchronized(streamLock) { return streamClosed; }
        }
        
    /** Returns the next byte from the stream, or -1 if EOF.  If we timeout when reading,
        an InterruptedIOException is thrown. */
    public int read() throws IOException
        {
        if (isStreamClosed()) return -1; 
        else 
            {
            int ret = dequeue(timeout);
            if (ret < 0 ) throw new InterruptedIOException("Read Would Block");
            return ret;
            }
        }
    
    // we're going to implement our own lightweight StringBuffer here.  There's
    // good reason to: the standard StringBuffers (sun's and gnu's) have O(n) mechanisms
    // for clearing themselves.  Which is really stupid and which is extremely costly for
    // the primary users of this package: small devices in our robot facility.  So we go
    // around it by making our own buffer and extending it appropriately.
    final static int READLINEBUFSIZE = 64;
    final static int MAXGOODREADLINEBUFSIZE = 1024;
    byte[] readLineBuffer = new byte[READLINEBUFSIZE];
    int readLineBufferSize = 0;
    
    /* Clears the readline buffer */
    void clear()
        {
        if (readLineBuffer.length > MAXGOODREADLINEBUFSIZE) readLineBuffer = new byte[READLINEBUFSIZE];
        readLineBufferSize = 0;
        }
        
    /* Appends a byte to our buffer */
    void append(byte b)
        {
        if (readLineBufferSize == readLineBuffer.length)
            {
            // increase
            byte[] newReadLineBuffer = new byte[readLineBuffer.length * 2];
            System.arraycopy(readLineBuffer, 0, newReadLineBuffer, 0, readLineBuffer.length);
            readLineBuffer = newReadLineBuffer;
            }
        readLineBuffer[readLineBufferSize++] = b;
        }
        
    /// gagh, I can't believe we had to do that.
            
    /** Reads a line from the stream, or throws an InterruptedIOException if a timeout occurs.
        A line is terminated at EOF, or if CRLF is true, then at a CR/LF pair, or if
        CRLF is false, then either a single CR or a single LF.  Terminators are stripped from the string.
        If we're already at EOF, null is returned.
    */
    public String readLine(boolean CRLF) throws IOException
        {
        boolean cr = false;
        if (isStreamClosed()) return null;
        clear();
        while (true)
            {
            int b = readNoException();
            if (b == EOF) return new String(readLineBuffer, 0, readLineBufferSize);
            else if (b == FAIL) throw new InterruptedIOException("ReadLine Would Block");
            else if ((b == 10 || b == 13) && !CRLF) return new String(readLineBuffer, 0, readLineBufferSize);
            else if (b == 13 && CRLF)
                {
                if (cr) append((byte) 13);  // already had a cr, put it in, stay true
                else cr = true;
                }
            else if (b == 10 && CRLF) 
                {
                if (cr) return new String(readLineBuffer, 0, readLineBufferSize); // we found our pair
                else append((byte) 10); // put it in
                }
            else 
                {
                if (cr) { cr = false; append((byte) 13); } // if we previously had a cr, put it in
                append((byte)b);
                }
            }
        }
    
    /** Returns the next byte from the stream, or -1 if EOF.  If we timeout when reading,
        we return -2 (NonBlockingInputStream.FAIL). */
    public int readNoException()
        {
        if (isStreamClosed()) return -1; 
        else return dequeue(timeout);
        }
    
    /** Returns the number of bytes available in the buffer.  This is NOT the number of bytes
        which can be read without blocking -- we may be able to read more -- it's just what
        we know for sure we can read without blocking because we have it stored in the buffer now. */
    public int available() throws IOException
        {
        synchronized(buffer)
            {
            if (end >= start) return end - start;
            else return (end + buffer.length - start + 1); // I think that's right
            }
        }
    
    /** Closes the stream.  If the stream was constructed with closeUnderlyingStreamWhenClosed=TRUE, then
        the underlying input stream is closed as well.  Closing the stream flushes it. */
    public void close() throws IOException
        {
        synchronized(streamLock)
            {
            if (streamClosed) return;  // we're outa here
            streamClosed = true;
            }
        pleaseDie = true;
        readThread.interrupt();
        try { readThread.join(); } catch (InterruptedException e) { /* hope it dies! */ }
        doClose();
        }
        
    /* Close procedures available to both the main and the underlying threads. */
    void doClose() throws IOException
        {
        synchronized(streamLock)
            {
            if (streamClosed) return;  // we're outa here
            streamClosed = true;
            }
        IOException except = null;
        if (closeUnderlyingStreamWhenClosed) try { stream.close(); } catch (IOException e) { except = e; }
        flush();
        if (except != null) throw except;
        }

    /** Flushes the buffer.  This is sort of like skip() except that the number of bytes skipped isn't specified -- it's
        whatever's in the buffer.*/
    public void flush()
        {
        synchronized(buffer) { start = end; }
        }
        

    /* The underlying read loop.  Reads a byte, then enqueues it.  If EOF, the stream is closed.  If an exception occurs,
       we do the appropriate behavior as specified in the constructor. */
    void doReadLoop()
        {
        while(!pleaseDie) try 
                              {
                              // read the byte
                              int b = stream.read();
                              if (b < 0) { close(); } // EOF
            
                              // enqueue the byte
                              synchronized(buffer)
                                  {
                                  if (end == start - 1 || (start == 0 && end==buffer.length - 1))  // end's right up against start, can't add any more (we sacrifice one byte of space)
                                      {
                                      //  must wait indefinitely
                                      try { buffer.wait(); } catch (InterruptedException e)  { return; }  // I just died
                                      }
                                  // now we must have some space
                                  buffer[end] = (byte)b;
                                  //System.err.println("--> " + b);
                                  end++;
                                  if (end >= buffer.length) end = 0;
                                  buffer.notify();  // let the dequeuer know we've made space if he's waiting
                                  }
                              }
            catch (IOException e)
                {
                if (underlyingExceptionBehavior == BEHAVIOR_CLOSE || underlyingExceptionBehavior == BEHAVIOR_CLOSE_PRINT)
                    {
                    if (underlyingExceptionBehavior == BEHAVIOR_CLOSE_PRINT) e.printStackTrace();
                    try { doClose(); } catch (IOException e2) { }
                    return;  // failed
                    }
                else
                    {
                    if (underlyingExceptionBehavior == BEHAVIOR_IGNORE_PRINT) e.printStackTrace();
                    // A common reason for an exception is Linux's "Resource temporarily unavailable" exception,
                    // which signifies EAGAIN.
                    // e.getMessage().equalsIgnoreCase("Resource temporarily unavailable"))  // EAGAIN
                    try { Thread.currentThread().sleep(timeout); } catch (InterruptedException e2) { } 
                    }
                }
        }

    /* Dequeues and returns the next byte.  If no byte, then FAIL is returned. */
    int dequeue()
        {
        int b = FAIL;
        synchronized(buffer)
            {
            if (start==end) return b;
            b = buffer[start];  
            if (b < 0) b += 256;  // bump to unsigned byte so we can consider -1 etc.
            start++;
            if (start >= buffer.length) start = 0;
            buffer.notify();  // let the enqueuer know we've made space if he's waiting
            }
        return b;
        }
    
    /* Dequeues and returns the next byte.  If no byte, then we wait for timeout time, then try dequeueing again.
       If still no byte, then FAIL is returned. */
    int dequeue(int timeout)
        {
        synchronized(buffer)
            {
            int b = dequeue();
            //System.out.println("<-- " + b);
            if (b==FAIL)
                {
                if (isStreamClosed()) return EOF;  // oh, THAT's why we failed...
                try { buffer.wait(timeout); }  // wait no longer than this
                catch (InterruptedException e) 
                    {
                    try { close(); } catch (IOException e2) { }
                    return EOF; 
                    } // eesh, what happened?
                b = dequeue();
                if (b==FAIL && isStreamClosed()) return EOF;  // oh, THAT's why we failed...
                }
            return b;
            }
        }       
    
    // try running this (in Beanshell, you can just say NonBlockingInputStream.test(); )
    // and then in another terminal, do a telnet localhost 5000
    // and try feeding it stuff.
    public static void test() throws Exception
        {
        java.net.ServerSocket ss = new java.net.ServerSocket(5000);
        java.net.Socket s = ss.accept();
        System.err.println("Got a live one!");
        NonBlockingInputStream i = new NonBlockingInputStream(s.getInputStream(), 1000, 32, BEHAVIOR_IGNORE, true);
        System.err.println("Stream constructed!");
        while(true)
            {
            int b = i.readNoException();
            if (b == FAIL) { System.err.println("Timeout!"); continue; }
            else if (b == EOF) { System.err.println("End of the line!"); break; }
            else System.out.print((char)((byte)b));
            }
            
        // close down
        try { s.close(); }
        catch (IOException e ) { }
        try { ss.close(); }
        catch (IOException e ) { }
        }
    }