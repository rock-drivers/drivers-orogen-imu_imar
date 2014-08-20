/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace imu_imar;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
    delete timestamp_estimator;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{   
    if (! TaskBase::configureHook())
        return false;
    
    timestamp_estimator = new aggregator::TimestampEstimator(
	base::Time::fromSeconds(20),
	base::Time::fromSeconds(1.0 / _sampling_rate.value()),
	base::Time::fromSeconds(0),
	INT_MAX);

     /** TO-DO: configure the IMU parameter values using the debug_port **/
     /** For the time being, it uses the default values: 57600 Baud-rate, 100 Hz 
      * and 50 bytes package size containing acc, gyros and roll, pitch and yaw **/
    
     
     /** Open the communication port which is the port to acquire the inertial values **/
     if (!imu_imar_driver.init_serial(_com_port.value().c_str(),_baudrate.value()))
     {
	 std::cerr << "Error opening device '" << _com_port.value() << "'" << std::endl;
	 return false;
     }
     
     reading.invalidate();
     reading.position.setZero();
     reading.velocity.setZero();
     reading.orientation = Eigen::Quaterniond::Identity();
     reading.angular_velocity.setZero();
     
     sensors.acc.setZero();
     sensors.gyro.setZero();
     sensors.mag.setZero();
     
    return true;
}
bool Task::startHook()
{
    int byteRead;
    unsigned char values[PKG_SIZE];
    
    if (! TaskBase::startHook())
        return false;
    
    /** Synchronize driver with the stream comming from the device **/
    while (!imu_imar_driver.cbIsSynchronized())
    {
	if ((byteRead = imu_imar_driver.read_serial(values, sizeof(values))))
	{
// 	    std::cout<<"byteRead: "<<byteRead<<"\n";
	    
	    /** Store the read values in the Circular Buffer **/
	    imu_imar_driver.cbWritePckg (byteRead, values);
// 	    std::cout<<"Writen "<<byteRead<<" bytes in Circular Buffer\n";

	    if (imu_imar_driver.cbIsFull())
	    {
		/**Synchronize with the starting byte**/
		imu_imar_driver.cbSynchronize();
	    }
	}
    }
    
    std::cout<<"iMAR Synchronized with device\n";
    timestamp_estimator->reset();
    
    return true;
}
void Task::updateHook()
{
    int byteRead, byteStored;
    unsigned char values[PKG_SIZE];
    
    
    
    /** The driver in synchronized, starting to read values **/
    if ((byteRead = imu_imar_driver.read_serial(values, sizeof(values))))
    {
	/** Time of receiving **/
	base::Time recvts = base::Time::now();
	
	/** Store the read values in the Circular Buffer **/
	imu_imar_driver.cbWritePckg (byteRead, values);
	    
	/** Check the status of the Driver Circular Buffer **/
	if ((byteStored = imu_imar_driver.cbNumberElements()) < PKG_SIZE)
	{
	    unsigned char mbyte[1];
	    
//  	    std::cout<<byteStored<<" elements in the Buffer\n";

	    /** Read a byte in the stream **/
	    if ((byteRead = imu_imar_driver.read_serial(mbyte, sizeof(mbyte))))
	    {
		/** Store the read values in the Circular Buffer **/
		imu_imar_driver.cbWritePckg (byteRead, mbyte);
	    }
	}
	else
	{    
	    int packet_counter = imu_imar_driver.getPacketCounter();
	    unsigned char buffer[PKG_SIZE];
	    unsigned int crc;
	    
// 	    std::cout<<byteStored<<" elements in the Buffer\n";
	    
	    base::Time ts = timestamp_estimator->update(recvts,packet_counter);
	    timeout_counter = 0;
	    
	    /** Compute the checksum **/
	    imu_imar_driver.cbCopyPckg(buffer, sizeof(buffer));
	    crc = imu_imar_driver.Crc16_0(buffer, sizeof(buffer));
//  	    printf("CRC: %X\n", crc>>24);
	    
	    /** Read the IMU inertial Values **/
	    if(imu_imar_driver.cbReadValues())
	    {
 		/** If checksum is correct **/
		if((crc>>24) == buffer[PKG_SIZE-1])
		{
// 		    imu_imar_driver.cbPrintValues();
		    
	// 	    imu_imar_driver.cbCalculateAccIntegration((float)2*PI*_sampling_rate.value());
		    
		    /** Write the values in the data object **/
		    sensors.time = ts;
		    sensors.acc = imu_imar_driver.getAccelerometers();
		    sensors.gyro = imu_imar_driver.getGyroscopes();
		    sensors.mag = base::Vector3d::Ones() * base::NaN<double>();
		    
		    reading.time = ts;
		    reading.orientation = imu_imar_driver.getAttitude();
		    reading.velocity = imu_imar_driver.getVelocity();
		    reading.position = imu_imar_driver.getPosition();
		    reading.angular_velocity = sensors.gyro;
		}
		
		
		/** Write in the Ports **/
		_orientation_samples.write(reading);
		_calibrated_sensors.write(sensors);
		
		_timestamp_estimator_status.write(timestamp_estimator->getStatus());
	    }
	    else
	    {
		std::cerr << "Error in stream at "<<ts<< std::endl;
		imu_imar_driver.cbReset();
 		this->startHook();
	    }
	}
    }
    
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
    
    imu_imar_driver.close_port();
    
    timestamp_estimator->reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    imu_imar_driver.close_port();
    
    delete timestamp_estimator;
    timestamp_estimator = NULL;
}

