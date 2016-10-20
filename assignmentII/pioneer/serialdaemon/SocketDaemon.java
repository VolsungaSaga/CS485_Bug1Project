import java.net.*;
import java.io.*;

public class SocketDaemon
    {
    public static void main(String[] args) throws Exception
	{
	int serverPort = Integer.parseInt(args[0]);
	int clientPort = Integer.parseInt(args[1]);
	String clientAddress = args[2];
	
	System.out.println("Connecting to Client");
	final Socket client = new Socket(clientAddress, clientPort);
	System.out.println("Connecting to Server");
	final Socket server = new ServerSocket(serverPort).accept();
	
	Thread thread = new Thread(new Runnable()
	    {
	    
	    public void run()
		{
		try
		    {
		    InputStream i = client.getInputStream();
		    OutputStream o = server.getOutputStream();
		    while(true)
			{
			int b = i.read();
			if (b == -1) break;
			o.write(b);
			}
		    }
		catch (Exception e) { }
		}
	    
	    });
	    
	thread.setDaemon(true);
	thread.start();
	
	InputStream i = server.getInputStream();
	OutputStream o = client.getOutputStream();
	while(true)
	    {
	    int b = i.read();
	    if (b == -1) break;
	    o.write(b);
	    }
	}
    }