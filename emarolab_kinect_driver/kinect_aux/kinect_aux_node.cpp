#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include <tf/transform_broadcaster.h> // !! added
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <math.h> 

// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 3.0 
#define MIN_TILT_ANGLE -60.0

#define PI 3.14159265359

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;
 
libusb_device_handle *dev(0);

Eigen::MatrixXf baxterCalibration(4,4); // homogeneus matrix
tf::Transform transformTilt;
tf::Transform transformFrame;

void openAuxDevice(int index = 0)
{
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
	if (cnt < 0)
	{
		ROS_ERROR("No device on USB");
		return;
	}
	
	int nr_mot(0);
	for (int i = 0; i < cnt; ++i)
	{
		struct libusb_device_descriptor desc;
		const int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		// Search for the aux
		if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
				{
					ROS_ERROR_STREAM("Cannot open aux " << index);
					return;
				}
				// Claim the aux
				libusb_claim_interface (dev, 0);
				break;
			}
			else
				nr_mot++;
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it
}

void publishState(void)
{
	uint8_t buf[10];
	const int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
	if (ret != 10)
	{
		ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
	
	const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
	const uint16_t uy = ((uint16_t)buf[4] << 8) | buf[5];
	const uint16_t uz = ((uint16_t)buf[6] << 8) | buf[7];
	
	const int16_t accelerometer_x = (int16_t)ux;
	const int16_t accelerometer_y = (int16_t)uy;
	const int16_t accelerometer_z = (int16_t)uz;
	const int8_t tilt_angle = (int8_t)buf[8];
	const uint8_t tilt_status = buf[9];
	
	// publish IMU
	sensor_msgs::Imu imu_msg;
	if (pub_imu.getNumSubscribers() > 0)
	{
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.linear_acceleration.x = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.y = (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.z = (double(accelerometer_z)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
			= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
		imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
		imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided
		pub_imu.publish(imu_msg);
	}
	
	// publish tilt angle and status
	if (pub_tilt_angle.getNumSubscribers() > 0)
	{
		std_msgs::Float64 tilt_angle_msg;
		tilt_angle_msg.data = double(tilt_angle) / 2.;
		pub_tilt_angle.publish(tilt_angle_msg);
	}
	if (pub_tilt_status.getNumSubscribers() > 0)
	{
		std_msgs::UInt8 tilt_status_msg;
		tilt_status_msg.data = tilt_status;
		pub_tilt_status.publish(tilt_status_msg);
	}
}


void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	uint8_t empty[0x1];
	double angle(angleMsg.data);

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
	angle = angle * 2;
	const int ret = libusb_control_transfer(dev, 0x40, 0x31, (uint16_t)angle, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting tilt angle, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}


	// compute the trasformation w.r.t. frame: camera_link due to this angle
	// compute angle 	
	double angle_rad = -( ( angleMsg.data * PI) / 180); // reverse segne from kinect aux publishing
 	double c = cos( angle_rad);
 	double s = sin( angle_rad);
	// compute simple rotation along y axis
	Eigen::MatrixXf rot(4,4);
	rot << c, 0, s, 0,
	       0, 1, 0, 0,
	       -s,0, c, 0, 
	       0, 0, 0, 1;

	// compute the rotation 
	Eigen::MatrixXf trans(4,4);
	trans = rot;

	// extrapolate rotation and translation from homogeneus matrix
	tf::Vector3 trans_T( trans(0,3), trans(1,3), trans(2,3));
	tf::Matrix3x3 trans_R(trans(0,0), trans(0,1), trans(0,2),trans(1,0), trans(1,1), trans(1,2),trans(2,0), trans(2,1), trans(2,2));

	// publish transformation between kinect and robot
	transformTilt.setOrigin( trans_T);
	transformTilt.setBasis( trans_R);

	trans = baxterCalibration; 
	tf::Vector3 trans_Tf( trans(0,3), trans(1,3), trans(2,3));
	tf::Matrix3x3 trans_Rf(trans(0,0), trans(0,1), trans(0,2),trans(1,0), trans(1,1), trans(1,2),trans(2,0), trans(2,1), trans(2,2));

	transformFrame.setOrigin( trans_Tf);
	transformFrame.setBasis( trans_Rf);


}

void setLedOption(const std_msgs::UInt16 optionMsg)
{
	uint8_t empty[0x1];
	const uint16_t option(optionMsg.data);
	
	const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting LED options, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}


int main(int argc, char* argv[])
{
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	

	// initialise kinect baxter calibration (with respect to tilt=0°)
	baxterCalibration <<  	-0.0356,    1.0949,    0.0698,   0.07,
    				 0.1829,   -0.1031,    1.0800,    0.08,//0.0714,
				 0.9805,    0.0551,   -0.1834,    0.160,//0.1204,
         			 0,         0,         0,         1.0;

	double correctionAngle = 0.0307;
	double c = cos(correctionAngle);
 	double s = sin(correctionAngle);
	Eigen::MatrixXf rotZ(4,4);
	rotZ << c, -s, 0, 0,
	       s, c, 0, 0,
	       0,0, 1, 0, 
	       0, 0, 0, 1;

	baxterCalibration = baxterCalibration * rotZ;

	
	ros::init(argc, argv, "kinect_aux");
	ros::NodeHandle n;
	
	int deviceIndex;
	n.param<int>("device_index", deviceIndex, 0);
	openAuxDevice(deviceIndex);
	if (!dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	
	pub_imu = n.advertise<sensor_msgs::Imu>("imu", 15);
	pub_tilt_angle = n.advertise<std_msgs::Float64>("cur_tilt_angle", 5);
	pub_tilt_status = n.advertise<std_msgs::UInt8>("cur_tilt_status", 5);
	
	sub_tilt_angle = n.subscribe("tilt_angle", 1, setTiltAngle);
	sub_led_option = n.subscribe("led_option", 1, setLedOption);
	 
	static tf::TransformBroadcaster br;
	
	ros::Rate rate(50);

	while (ros::ok()) //( n.ok())
	{
		publishState();
		br.sendTransform(tf::StampedTransform(transformFrame, ros::Time::now(), "head_camera", "cameraB_link_untilted"));
		br.sendTransform(tf::StampedTransform(transformTilt, ros::Time::now(), "cameraB_link_untilted", "cameraB_link"));

		ros::spinOnce();
		rate.sleep();
	}
	
	libusb_exit(0);
	return 0;
}
