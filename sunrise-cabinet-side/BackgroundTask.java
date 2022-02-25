package lbrExampleApplications;

/* By Mohammad SAFEEA: Coimbra University-Portugal, 
 * Ensam University-France
 * 
 * KST 1.7
 * 
 * First upload 07-May-2017
 * 
 * Second update 26th-06-2018 
 * 
 * Last update 12-May-2019, introduced updates are marked with the comment,
 *  "modified 12th-May-2019" or the comment "newly-added 12th-May-2019"
 * 
 * This is a multi-threaded server program that is meant to be used with both
 *    KUKA iiwa 7 R 800
 * or KUKA iiwa 14 R 820.
 * The server of this application listens on the port 30001.
 * 
 * */

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Scanner;
import java.util.StringTokenizer;
import java.util.logging.Logger;

import sun.security.action.GetLongAction;

import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
//import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;

 

class BackgroundTask implements Runnable {

	
	
	private Controller kuka_Sunrise_Cabinet_1;
	//private MediaFlangeIOGroup daIO;
	public boolean terminateBool;
	private LBR _lbr;
	private int _port;
	private int _timeOut;
	private ServerSocket ss;
	private Socket soc;
	
    //private static final String stopCharacter="\n"+Character.toString((char)(10));
    private static final String stopCharacter=Character.toString((char)(10));
    private static final String ack="done"+stopCharacter;
    
	
	BackgroundTask(int daport, int timeOut,Controller kuka_Sunrise_Cabinet_1,LBR _lbr )
	{
		_timeOut=timeOut;
		_port=daport;
		this.kuka_Sunrise_Cabinet_1 = kuka_Sunrise_Cabinet_1;
		this._lbr = _lbr;
		//daIO=new MediaFlangeIOGroup(kuka_Sunrise_Cabinet_1);
		terminateBool=false;
		Thread t= new Thread(this);
		t.setDaemon(true);
		t.start();

	}

	
	public void run() {
		// TODO Auto-generated method stub
		
		try {
			ss= new ServerSocket(_port);
			try
			{
			ss.setSoTimeout(_timeOut);
			soc= ss.accept();
			}
			catch (Exception e) {
				// TODO: handle exception
				ss.close();
				ToolboxServer.terminateFlag=true;
				return;
			}
			Scanner scan= new Scanner(soc.getInputStream());
			// In this loop you shall check the input signal
			while((soc.isConnected()))
			{
				if(scan.hasNextLine())
				{			
					ToolboxServer.daCommand=scan.nextLine();
					 
					if(ToolboxServer.daCommand.startsWith("jf_"))
		        	{
		        		boolean tempBool=getTheJointsf(ToolboxServer.daCommand);
		        		ToolboxServer.daCommand="";
		        		// ToolboxServer.printMessage(ToolboxServer.daCommand);
		        		if(tempBool==false)
		        		{
		        			ToolboxServer.directSmart_ServoMotionFlag=false;
		        		}
		        		// this.sendCommand(ack); no acknowledgement in fast execution mode
		        	}
					// If the signal is equal to end, you shall turn off the server.
					else if(ToolboxServer.daCommand.startsWith("end"))
					{
						/* Close all existing loops:
						/  1- The BackgroundTask loop.
						 * 2- The main class, ToolboxServer loops:
						 * 		a- The while loop in run, using the flag: ToolboxServer.terminateFlag.
						 * 		b- The direct servo loop, using the flag: ToolboxServer.directServoMotionFlag.
						*/
						ToolboxServer.directSmart_ServoMotionFlag=false;
						ToolboxServer.terminateFlag=true;
						break;						
					}
					// Put the direct_servo joint angles command in the joint variable
					else if(ToolboxServer.daCommand.startsWith("jp"))
		        	{
		        		updateJointsPositionArray();
		        	}
					else if(ToolboxServer.daCommand.startsWith("vel"))
		        	{
		        		updateVelocityArrays();
		        	}
					else if(ToolboxServer.daCommand.startsWith("cArtixanPosition"))
		        	{
						if(ToolboxServer.daCommand.startsWith("cArtixanPositionCirc1"))
						{
			        		boolean tempBool=getEEFposCirc1(ToolboxServer.daCommand);
			        		// ToolboxServer.printMessage(ToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//ToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ToolboxServer.daCommand="";
						}
						else if(ToolboxServer.daCommand.startsWith("cArtixanPositionCirc2"))
						{
			        		boolean tempBool=getEEFposCirc2(ToolboxServer.daCommand);
			        		// ToolboxServer.printMessage(ToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//ToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ToolboxServer.daCommand="";
						}
						else
						{
			        		boolean tempBool=getEEFpos(ToolboxServer.daCommand);
			        		// ToolboxServer.printMessage(ToolboxServer.daCommand);
			        		if(tempBool==false)
			        		{
			        			//ToolboxServer.directServoMotionFlag=false;
			        		}
			        		this.sendCommand(ack);
			        		ToolboxServer.daCommand="";
						}
		        	}
					
					// This insturction is used to turn_off the direct_servo controller
		        	else if(ToolboxServer.daCommand.startsWith("stopDirectServoJoints"))
		        	{
		        		ToolboxServer.directSmart_ServoMotionFlag=false;
		        		this.sendCommand(ack);
		        		ToolboxServer.daCommand="";
		        	}
					else if(ToolboxServer.daCommand.startsWith("DcSe"))
		        	{
		        		updateEEFPositionArray();
		        	}
		        	else
		        	{
		        		// inquiring data from server
		        		dataAqcuisitionRequest();
		        	}
					
				}				
			}
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
		
		
		try {
			ToolboxServer.terminateFlag=true;
			soc.close();
			ss.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
	}

	private void updateVelocityArrays()
	{
		if(ToolboxServer.daCommand.startsWith("velJDC_"))
		{
			boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			this.sendCommand(ack);
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("velJDCExT_"))
		{
			boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsExternalTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("velJDCMT_"))
		{
			boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsMeasuredTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("velJDCEEfP_"))
		{
			boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendEEfPositionToClient(); // "modified 12th-May-2019"
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("velJDCJP_"))
		{
			boolean tempBool=getJointsVelocitiesForVelocityContrtolMode(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsPositionsToClient();
			ToolboxServer.daCommand="";
		}
	}
	
	private void updateEEFPositionArray()
	{
		//////////////////////////////////////////////////
		//Start of server update functions
		/////////////////////////////////////////////////////						
		
		if(ToolboxServer.daCommand.startsWith("DcSeCar_"))
		{
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			// this.sendCommand(ack);
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("DcSeCarExT_"))
		{
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsExternalTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("DcSeCarMT_"))
		{
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsMeasuredTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("DcSeCarEEfP_"))
		{
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendEEfPositionToClient(); // "modified 12th-May-2019"
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("DcSeCarJP_"))
		{
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
			ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsPositionsToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("DcSeCarEEfFrelEEF_")) 
		{
			// "newly-added 12th-May-2019"
			// Update Cartesian position of EEF and returns force force feedback described in EEF reference frame	
			boolean tempBool=getThePositions(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendEEFforcesToClient();
			ToolboxServer.daCommand="";
		}
		//////////////////////////////////////////////////
		//End of Servo joints update functions
		//////////////////////////////////////////////////////
	}
	
	private boolean getThePositions(String thestring) {
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
				int j=0;
				while(st.hasMoreTokens())
				{
					if(j<6)
					{
						//getLogger().warn(jointString);
						try
						{
							ToolboxServer.EEfServoPos[j]=Double.parseDouble(st.nextToken());
						}
						catch(Exception e)
						{
							return false;
						}						
					}					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
	}

	private boolean getJointsVelocitiesForVelocityContrtolMode(String thestring) {
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
				int j=0;
				while(st.hasMoreTokens())
				{
					if(j<7)
					{
						//getLogger().warn(jointString);
						try
						{
							ToolboxServer.jvel[j]=
							Double.parseDouble(st.nextToken());
						}
						catch(Exception e)
						{
							return false;
						}						
					}					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
	}

	
	private void updateJointsPositionArray()
	{
		//////////////////////////////////////////////////
		//Start of server update functions
		/////////////////////////////////////////////////////								
		if(ToolboxServer.daCommand.startsWith("jp_"))
		{
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			this.sendCommand(ack);
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("jpExT_"))
		{
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsExternalTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("jpMT_"))
		{
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsMeasuredTorquesToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("jpEEfP_"))
		{
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendEEfPositionToClient(); // "modified 12th-May-2019"
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("jpJP_"))
		{
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendJointsPositionsToClient();
			ToolboxServer.daCommand="";
		}
		else if(ToolboxServer.daCommand.startsWith("jpEEfFrelEEF_"))
		{
			// "newly-added 12th-May-2019"
			boolean tempBool=getTheJoints(ToolboxServer.daCommand);
			// ToolboxServer.printMessage(ToolboxServer.daCommand);
			if(tempBool==false)
			{
				ToolboxServer.directSmart_ServoMotionFlag=false;
			}
			ToolboxServer.svr.sendEEFforcesToClient();
			ToolboxServer.daCommand="";
		}

		
		//////////////////////////////////////////////////
		//End of Servo joints update functions
		//////////////////////////////////////////////////////
	}
	
	// respond to a data Acquisition Request
	private void dataAqcuisitionRequest()
	{
		// Inquiring data from server
    	if(ToolboxServer.daCommand.startsWith("getJointsPositions"))
    	{
    		ToolboxServer.svr.sendJointsPositionsToClient();
    		ToolboxServer.daCommand="";
    	}        	
    	// Write output of Mediaflange
    	else if(ToolboxServer.daCommand.startsWith("blueOn"))
    	{
    		ToolboxServer.mff.blueOn();
    		sendCommand(ack);
    		ToolboxServer.daCommand="";
    	}
    	else if(ToolboxServer.daCommand.startsWith("blueOff"))
    	{
    		ToolboxServer.mff.blueOff();
    		sendCommand(ack);
    		ToolboxServer.daCommand="";
    	}
    	else if(ToolboxServer.daCommand.startsWith("pin"))
    	{
        	if(ToolboxServer.daCommand.startsWith("pin1on"))
			{
        		ToolboxServer.mff.pin1On();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin1off"))
			{
				ToolboxServer.mff.pin1Off();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin11on"))
			{
				ToolboxServer.mff.pin11On();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin11off"))
			{
				ToolboxServer.mff.pin11Off();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin2on"))
			{
				ToolboxServer.mff.pin2On();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin2off"))
			{
				ToolboxServer.mff.pin2Off();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin12on"))
			{
				ToolboxServer.mff.pin12On();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("pin12off"))
			{
				ToolboxServer.mff.pin12Off();
				sendCommand(ack);
				ToolboxServer.daCommand="";
			}
    	}
    	// Read input of Mediaflange
    	if(ToolboxServer.daCommand.startsWith("getPin"))
    	{
			if(ToolboxServer.daCommand.startsWith("getPin10"))
			{
				ToolboxServer.mff.getPin10();
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("getPin16"))
			{
				ToolboxServer.mff.getPin16();
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("getPin13"))
			{
				ToolboxServer.mff.getPin13();
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("getPin3"))
			{
				ToolboxServer.mff.getPin3();
				ToolboxServer.daCommand="";
			}
			else if(ToolboxServer.daCommand.startsWith("getPin4"))
			{
				ToolboxServer.mff.getPin4();
				ToolboxServer.daCommand="";
			}
    	}
	}
	
    
	/* The following function is used to extract the 
	 joint angles from the command
	 */
	private boolean getTheJoints(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("jp"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<7)
					{
						//getLogger().warn(jointString);
						ToolboxServer.jpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}

	/* The following function is used to extract the 
	 joint angles from the command
	 */
	private boolean getTheJointsf(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("jf"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<7)
					{
						//getLogger().warn(jointString);
						ToolboxServer.jpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	
	private boolean getEEFpos(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ToolboxServer.EEFpos[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}

	
	private boolean getEEFposCirc2(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ToolboxServer.EEFposCirc2[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	
	
	private boolean getEEFposCirc1(String thestring)
	{
		
		
		StringTokenizer st= new StringTokenizer(thestring,"_");
		if(st.hasMoreTokens())
		{
			String temp=st.nextToken();
			if(temp.startsWith("cArtixanPosition"))
			{
				int j=0;
				while(st.hasMoreTokens())
				{
					String jointString=st.nextToken();
					if(j<6)
					{
						//getLogger().warn(jointString);
						ToolboxServer.EEFposCirc1[j]=Double.parseDouble(jointString);
					}
					
					j++;
				}
				ToolboxServer.daCommand="";
				return true;
				
			}
			else
			{
				return false;
			}
		}

		return false;
	}
	/* The following function is used to sent a string message
	 * to the server
	 */
	public boolean sendCommand(String s)
	{
		if(ss==null)return false;
		if(soc==null)return false;
		if(soc.isClosed())return false;
		
		try {
			soc.getOutputStream().write(s.getBytes("US-ASCII"));
			return true;
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		return false;
		
	}
	

	
}
	
