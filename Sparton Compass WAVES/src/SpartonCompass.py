#----------------------------------------------------------------------------------------------------------
# SPARTON COMPASS ROS Driver
#
#  
# BRIEF:        Main entry point for UM6 driver. Handles serial connection
#              details, as well as all ROS message stuffing, parameters,
#              topics, etc.
# author:      Snehal Jain < RE CENSAM, SMART >
#
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
# * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# *
# Please send comments, questions, or patches to snehal@smart.mit.edu
# ---------------------------------------------------------------------------------------------------------


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D, Quaternion
from sensor_msgs.msg import Imu

from tf_transformations import euler_from_quaternion, quaternion_from_euler

import time, math, calendar, string, serial
import roslib

def Radian2PI(theta):
    return (theta%2.*math.pi)
    
def RadianPI(theta):
    return (Radian2PI(theta+math.pi) - math.pi)
    
def Shutdown():
    global AHRS8 #Name of file
    global Dropline #set drop info
    print "Sparton shutdown time!"
    AHRS8.write(Dropline)
    AHRS8.flush() # flush data out before closing file
    rospy.loginfo('Closing Digital Compass Serial port')
    AHRS8.close()
    
if __name__ == '__main__':
    global AHRS8
    global Dropline
    rospy.init_node('SpartonDigitalCompassIMU')
    Pos_pub  = rospy.Publisher('imu/HeadingTrue', Pose2D)
    Imu_pub = rospy.Publisher('imu/data', Imu)
    SpartonPose2D=Pose2D()
# Initialize Pose2D message
    SpartonPose2D.x=float(0.0)
    SpartonPose2D.y=float(0.0)
    SpartonPose2D.theta = float(0.0)
    SpartonPose2D_D=SpartonPose2D
    
#------------------------------------------------------------------------------------------------------------
#Compass port parameters
#------------------------------------------------------------------------------------------------------------
    S_Compassport = rospy.get_param('~port','/dev/ttyUSB0')
    S_Compassrate = rospy.get_param('~baud',115200)
    S_Printmodulus = rospy.get_param('~printmodulus',1)   # printmodulus 60:10Hz 40:15~17  35:17~18Hz 30:21Hz 25:23~27Hz ,20: 30~35Hz,15:35~55Hz 10: 55~76 Hz,  5: 70~100 Hz, 1:70~100 Hz 
    
    S_Compass_Declination = rospy.get_param('~declination',0.2333333333334*(math.pi/180.0))
    S_Compass_EastAsZero = rospy.get_param('~UseEastAsZero',True)
    
    Checksum_error_limits = rospy.get_param('~Checksum_error_limits', 15)
    checksum_error_counter=0
#-------------------------------------------------------------------------------------------------------------
    imu_data = Imu()
    imu_data = Imu(header=rospy.Header(frame_id="SpartonCompassIMU"))
    
    # Initialize Imu message
    imu_data.orientation_covariance = [1e-6, 0, 0, 
                                       0, 1e-6, 0, 
                                       0, 0, 1e-6]
    
    imu_data.angular_velocity_covariance = [1e-6, 0, 0,
                                            0, 1e-6, 0, 
                                            0, 0, 1e-6]
    
    imu_data.linear_acceleration_covariance = [1e-6, 0, 0, 
                                               0, 1e-6, 0, 
                                               0, 0, 1e-6]
    # Instruction lines
    Dropline='\r\n\r\n print trigger 0 set drop \r\n'
    Checkline='\r\n printmask gyrop_trigger accelp_trigger or quat_trigger or time_trigger or set drop \r\n'
    ModulusLine='printmodulus %i set drop \r\n' % S_Printmodulus  
    Streamline='printtrigger printmask set drop \r\n'
    GSRline='$PSRFS,gyroSampleRate,get'

    rospy.on_shutdown(Shutdown)

    try:
        #Start communication
        AHRS8 = serial.Serial(S_Compassport, S_Compassrate, timeout = 0.5)
        AHRS8.write(Dropline)
        AHRS8.flush() # flush data out
        time.sleep(0.5)
        # readout all data, if any
        rospy.loginfo("[Dropline] Send Stop Continus mode to Digital Compass Got bytes %i" % AHRS8.inWaiting() ) 
        if (AHRS8.inWaiting() >0):
                #read out all datas, the response shuldbe OK (WHY? inWaiting gives the number of bytes to the data, right?)
                data=AHRS8.read(AHRS8.inWaiting())
                print("[Dropline - Diagnostic] Send to Digital Compass: Dropline Got: %s" % data) 

        else:
                rospy.logerr('[Dropline-Error] Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('[Dropline-Error] Received No data from DigitalCompass')
        AHRS8.write(Checkline) # send printmask
        time.sleep(0.5)
        data = AHRS8.readline()
        
        if (len(data) >0):
                rospy.loginfo("[Checkline] Send to Digital Compass: %s" % Checkline ) 
                rospy.loginfo("[Checkline - Diagnostic] Send to Digital Compass Got: %s" % data )
                
                AHRS8.write(GSRline) # find Fyro Sampling Rate
                Data_GSR = AHRS8.readline()
                rospy.loginfo("[GSRline] Send to Digital Compass: %s" %(GSRline))
                rospy.loginfo("[GSRline] Send to Digital Compass. Got: %s" %data)
                
                #Assigning $(Print Modulus) based on the data
                # S_Printmodulus = 400/data 
                # rospy.loginfo("GSR is %i and print modulus is %i" (%data, %S_Printmodulus))
                
                AHRS8.write(ModulusLine) # setup printmodule
                data = AHRS8.readline()
                rospy.loginfo("[Modulusline] Send to Digital Compass: %s " %(ModulusLine)) 
                rospy.loginfo("[Modulusline - Diagnostic] Send to Digital Compass. Got: %s" % data) 

                AHRS8.write(Streamline) # start the data streaming
                data = AHRS8.readline()
                rospy.loginfo("[Streamline] Send to Digital Compass: %s " % Streamline ) # should got OK here
                rospy.loginfo("[Streamline - Diagnostic] Send to Digital Compass. Got: %s" % data) # should got OK here
                rospy.loginfo('Digital Compass Setup Complete!')
                
        else:
                rospy.logerr('[Checkline-Error]Received No data from DigitalCompass. Shutdown!')
                rospy.signal_shutdown('[Checkline-Error]Received No data from DigitalCompass') 
        
        # Data Parsing of messages (based on wireshark's format)
        while not rospy.is_shutdown():
 
            data = AHRS8.readline()
            DataTimeSec=rospy.get_time()
            fields = data.split(',')
            # print fields[0]+fields[2]+fields[6]+fields[10]

            try:
                if len(fields)==14:
                        if 'P:apgpq' == (fields[0]+fields[2]+fields[6]+fields[10]):
                                Ax=float(fields[3])/1000.*9.806855 # convert to m/s^2 from mg/s
                                Ay=float(fields[4])/1000.*9.806855
                                Az=float(fields[5])/1000.*9.806855
                                Gx=float(fields[7]) * (math.pi/180.0) # convert to radians from degrees
                                Gy=float(fields[8]) * (math.pi/180.0)
                                Gz=float(fields[9]) * (math.pi/180.0)
                                w =float(fields[11])
                                x =float(fields[12])
                                y =float(fields[13])
                                z =float(fields[14])
                                
                                #imu_data.header.stamp = rospy.Time.now() # Should add an offset here
                                imu_data.header.stamp = rospy.Time.from_sec(DataTimeSec-len(data)/11520.) # this is timestamp with a bit time offset 10bit per byte @115200bps
                                imu_data.orientation = Quaternion()
                                # [w,x,y,z] (North-East Down) convert to [x,y,z,w] (East-North Up) according to ROS formats
                                imu_data.orientation.x = y
                                imu_data.orientation.y = x
                                imu_data.orientation.z = -z
                                imu_data.orientation.w = w
                                
                                ENU=  [y,x,-z,w] ;
                                angle_ENU=euler_from_quaternion(ENU, axes='sxyz'); 
                                
                                #add 90 degree and -declination , ROS= angle_ENU +90 degree - declination
                                angle_ROS=(angle_ENU[0],angle_ENU[1],angle_ENU[2]+math.pi/2.- D_Compass_declination);     #print angle_ROS
                                q_ROS=quaternion_from_euler(angle_ROS[0],angle_ROS[1],angle_ROS[2], axes='sxyz');         #print q_ROS

                                if D_Compass_UseEastAsZero 
                                    (imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w)=q_ROS
                                else:
                                    pass
                                # again note NED to ENU converstion
                                imu_data.angular_velocity.x = Gy
                                imu_data.angular_velocity.y = Gx
                                imu_data.angular_velocity.z = -Gz
                                # again note NED to ENU converstion
                                imu_data.linear_acceleration.x = Ay
                                imu_data.linear_acceleration.y = Ax
                                imu_data.linear_acceleration.z = -Az

                                Imu_pub.publish(imu_data)

                                #SpartonPose2D.y=1000./(float(fields[1])-SpartonPose2D.x) # put update rate here for debug the update rate
                                #SpartonPose2D.x=float(fields[1]) # put mSec tick here for debug the speed
                                #SpartonPose2D.theta = wrapToPI(math.radians(90.-float(fields[11])-D_Compass_offset))
                                #SpartonPose2D.theta = wrapToPI(yaw_ros)
                                SpartonPose2D.theta = wrapToPI(angle_ROS[2])
                                #print SpartonPose2D.theta/math.pi *180.
                                Pos_pub.publish(SpartonPose2D)
                                SpartonPose2D_D.theta =SpartonPose2D.theta/math.pi *180.
                                PosD_pub.publish(SpartonPose2D_D)
                                
                                # reset checksum_error_counter when you have good data
                                checksum_error_counter=0

                            else:
                                rospy.logerr("Received a sentence but not correct. USB may be out of sync or the print modulus is not defined properly. Sentence was: %s" % data)
                                checksum_error_counter+=1
                                if (checksum_error_counter > Checksum_error_limits)
                                    rospy.logfatal('Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                                    rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')

                else:
                        rospy.logerr("Received a sentence, could be correct. Yaw(16) and no Yaw(14) code check. Or missed sync by a few seconds. Sentence was: %s" % data)
                        # Usually when USB-Serial miss sync wiill cause error
                        checksum_error_counter+=1
                        if (checksum_error_counter > Checksum_error_limits ):
                            rospy.logfatal('Too much back to back checksumn error in Sparton Compass data. Shutdown!')
                            rospy.signal_shutdown('Too much back to back checksum error in Sparton Compass data')

            except ValueError as e:
                rospy.logwarn("Value error, likely due to missing fields in the data messages.Sentence was: %s" % data)

        # no loop, delay, ROSspin() here, we try to read all the data asap. This is for Ctrl+C system shutdown.
        D_Compass.write(myStr1) # stop data stream before close port
        D_Compass.flush() # flush data out

        rospy.loginfo('Closing Digital Compass Serial port')
        D_Compass.close()
    except rospy.ROSInterruptException:
        pass