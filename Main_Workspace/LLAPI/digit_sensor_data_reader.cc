// Digit Sensor Example
// Egwuchukwu Kalu
// 02/11/2023

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#include "lowlevelapi.h"

int main(int argc, char* argv[])
{
	FILE *fptr[20];
	const char* states[20] = {
		"Data/LeftHipRoll.csv","Data/LeftHipYaw.csv",
		"Data/LeftHipPitch.csv","Data/LeftKnee.csv",
		"Data/LeftToeA.csv","Data/LeftToeB.csv",
		"Data/RightHipRoll.csv","Data/RightHipYaw.csv",
		"Data/RightHipPitch.csv","Data/RightKnee.csv",
		"Data/RightToeA.csv","Data/RightToeB.csv",
		"Data/LeftShoulderRoll.csv","Data/LeftShoulderPitch.csv",
		"Data/LeftShoulderYaw.csv","Data/LeftElbow.csv",
		"Data/RightShoulderRoll.csv","Data/RightShoulderPitch.csv",
		"Data/RightShoulderYaw.csv","Data/RightElbow.csv"};
	for (int i = 0; i < 20; ++i)
		{	
			fptr[i] = fopen(states[i],"w");
			if(fptr[i] == NULL)
				{
					std::cerr << "Error: The data file " << states[i] << " cannot be created/modified.";
					return 1;
				}
			fprintf(fptr[i],"Time,q,dq,tau\n");
		}
			
  // The publisher address should be changed to the ip address of the robot
  const char* publisher_address = "127.0.0.1";
  llapi_init(publisher_address);

  // Define inputs and outputs (updated each iteration)
  llapi_command_t command = {0};
  llapi_observation_t observation;
  // Connect to robot (need to send commands until the subscriber connects)
  command.apply_command = true;
  while (!llapi_get_observation(&observation)) llapi_send_command(&command);

  // Get local copy of command limits (torque and damping)
  const llapi_limits_t* limits = llapi_get_limits();

  double start_time = observation.time;
  double time = start_time;
  while(1)
	{
		int return_val = llapi_get_observation(&observation); //Check if for errors
		 // Update observation
		if (return_val < 1) {
			// Error occurred
		} else if (return_val) {
			// New data received
		} else {
			// No new data
		}

		for(int i = 0; i < NUM_MOTORS;++i)
			{
				double q = observation.motor.position[i];
				double dq = observation.motor.velocity[i];
				double tau = observation.motor.torque[i];
				fprintf(fptr[i],"%f,%f,%f,%f\n",time,q,dq,tau);
			}
		time = observation.time - start_time;
		command.fallback_opmode = Damping;
		command.apply_command = true;

		llapi_send_command(&command);
		// Check if llapi has become disconnected
		if (!llapi_connected()) {
			// Handle error case. You don't need to re-initialize subscriber
			// Calling llapi_send_command will keep low level api open
		}
		// Sleep to keep to a reasonable update rate
		usleep(10000);
	}
  return 0;
  }
