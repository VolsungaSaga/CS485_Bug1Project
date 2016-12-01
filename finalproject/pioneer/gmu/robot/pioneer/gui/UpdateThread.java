package gmu.robot.pioneer.gui;
import java.awt.Component;
import java.util.*;

public class UpdateThread
    {
    Thread thread;
    long sleep = 100;
    boolean die = false;
    Object[] lock = new Object[0];
    ArrayList components = new ArrayList();
        
    public void setSleep(long val) { synchronized(lock) { sleep = val; } }
    public long getSleep() { synchronized(lock) { return sleep; } }
        
    public void add(Component c) 
        {
        synchronized(lock)
            {
            components.remove(c);
            components.add(c);
            }
        }
        
    public void remove(Component c)
        {
        synchronized(lock)
            {
            components.remove(c);
            }
        }
                
    public void removeAll()
        {
        synchronized(lock)
            {
            components = new ArrayList();
            }
        }
        
    public void start()
        {
        if (thread==null) 
            {
            thread = new Thread(new Runnable()
                {
                public void run()
                    {
                    long s = 0;
                    while(true)
                        {
                        synchronized(lock)
                            {
                            s = sleep;
                            if (die) break;
                            int len = components.size();
                            for(int x = 0 ; x< len; x++)
                                {
                                ((Component)components.get(x)).repaint();
                                }
                            }
                        try
                            {
                            Thread.sleep(s);
                            }
                        catch (InterruptedException e) { }
                        }
                    }
                });
            thread.start();
            }
        }
                
    public void stop()
        {
        if (thread!=null)
            {
            synchronized(lock) { die = true; }
            thread.interrupt();
            try { thread.join(); } catch (InterruptedException e) { }
            }
        }
    }