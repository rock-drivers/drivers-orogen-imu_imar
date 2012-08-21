/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace imar;

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
     if (imar_driver.init_serial(_com_port.value().c_str(),_baudrate.value()) == ERROR)
     {
	 std::cerr << "Error opening device '" << _com_port.value() << "'" << std::endl;
	 return false;
     }
     
    return true;
}
bool Task::startHook()
{
    int byteRead;
    unsigned char values[PKG_SIZE];
    
    if (! TaskBase::startHook())
        return false;
    
    /** Synchronize driver with the stream comming from the device **/
    while (!imar_driver.cbIsSynchronized())
    {
	if ((byteRead = imar_driver.read_serial(values, sizeof(values))) != ERROR)
	{
// 	    std::cout<<"byteRead: "<<byteRead<<"\n";
	    
	    /** Store the read values in the Circular Buffer **/
	    imar_driver.cbWritePckg (byteRead, values);
// 	    std::cout<<"Writen "<<byteRead<<" bytes in Circular Buffer\n";

	    if (imar_driver.cbIsFull())
	    {
		/**Synchronize with the starting byte**/
		imar_driver.cbSynchronize();
	    }
	}
    }
    
    timestamp_estimator->reset();
    
    return true;
}
void Task::updateHook()
{
    int byteRead, byteStored;
    unsigned char values[PKG_SIZE];
    base::samples::IMUSensors sensors;
    base::samples::RigidBodyState reading;
    
    reading.invalidate();
    
    /** The driver in synchronized, starting to read values **/
    if ((byteRead = imar_driver.read_serial(values, sizeof(values))) != ERROR)
    {
	/** Time of receiving **/
	base::Time recvts = base::Time::now();
	
	/** Store the read values in the Circular Buffer **/
	    imar_driver.cbWritePckg (byteRead, values);
	    
	/** Check the status of the Driver Circular Buffer **/
	if ((byteStored = imar_driver.cbNumberElements()) < PKG_SIZE)
	{
	    unsigned char mbyte[1];

	    /** Read a byte in the stream **/
	    if ((byteRead = imar_driver.read_serial(mbyte, sizeof(mbyte))) != ERROR)
	    {
		/** Store the read values in the Circular Buffer **/
		imar_driver.cbWritePckg (byteRead, mbyte);
	    }
	}
	else
	{    
	    int packet_counter = imar_driver.getPacketCounter();
	    
	    base::Time ts = timestamp_estimator->update(recvts,packet_counter);
	    timeout_counter = 0;
	    
	    /** Read the IMU inertial Values **/
	    imar_driver.cbReadValues();
	
// 	    imar_driver.cbPrintValues();
	    
	    /** Write the values in the data object **/
	    sensors.time = ts;
	    sensors.acc = imar_driver.getAccelerometers();
	    sensors.gyro = imar_driver.getGyroscopes();
	    sensors.mag = base::Vector3d::Ones() * base::NaN<double>();
	    
	    reading.time = ts;
	    reading.orientation = imar_driver.getAttitude();
	    reading.angular_velocity = sensors.gyro;
	    
	    
	    /** Write in the Ports **/
	    _orientation_samples.write(reading);
	    _calibrated_sensors.write(sensors);
	    
	    _timestamp_estimator_status.write(timestamp_estimator->getStatus());
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
    
    imar_driver.close_port();
    
    timestamp_estimator->reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    imar_driver.close_port();
    
    delete timestamp_estimator;
    timestamp_estimator = NULL;
}

