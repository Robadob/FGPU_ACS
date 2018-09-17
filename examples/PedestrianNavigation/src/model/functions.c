/*
 * Copyright 2011 University of Sheffield.
 * Author: Dr Paul Richmond 
 * Contact: p.richmond@sheffield.ac.uk (http://www.paulrichmond.staff.shef.ac.uk)
 *
 * University of Sheffield retain all intellectual property and 
 * proprietary rights in and to this software and related documentation. 
 * Any use, reproduction, disclosure, or distribution of this software 
 * and related documentation without an express license agreement from
 * University of Sheffield is strictly prohibited.
 *
 * For terms of licence agreement please attached licence or view licence 
 * on www.flamegpu.com website.
 * 
 */

#include <random>
#include <ctime>
#include "dynamic/header.h"
#ifndef _FLAMEGPU_FUNCTIONS
#define _FLAMEGPU_FUNCTIONS

#include "header.h"
#include "CustomVisualisation.h"

#define SCALE_FACTOR 0.03125

#define I_SCALER (SCALE_FACTOR*0.35f)
#define MESSAGE_RADIUS d_message_pedestrian_location_radius
#define MIN_DISTANCE 0.0001f
#define PED_RADIUS 0.25f //in metres
#define PED_DIAMETER 0.5f //in metres
#define AGENT_MASS 65.0f //Average mass of 65 kilograms
#define INTERACTION_RANGE 4.0f //4metres
#define INTERACTION_STRENGTH 100.0f

#define COLLISION_WEIGHT	1000.0f //30.0f //10.0f//0.50f
#define GOAL_WEIGHT			120.0f //0.20f
#define MAX_SPEED 2.0f      //Metres per second
//#define NUM_EXITS 7

#define PI 3.1415f
#define RADIANS(x) (PI / 180.0f) * x

__FLAME_GPU_FUNC__ int getNewExitLocation(RNG_rand48* rand48){
	if (rnd<DISCRETE_2D>(rand48)<0.5f)
		return 1;
	else
		return 0;
}

/**
 * output_location FLAMEGPU Agent Function
 * Automatically generated using functions.xslt
 * @param agent Pointer to an agent structre of type xmachine_memory_agent. This represents a single agent instance and can be modified directly.
 * @param location_messages Pointer to output message list of type xmachine_message_location_list. Must be passed as an argument to the add_location_message function ??.
 */
__FLAME_GPU_FUNC__ int output_pedestrian_location(xmachine_memory_agent* agent, xmachine_message_pedestrian_location_list* pedestrian_location_messages){

    
	add_pedestrian_location_message(pedestrian_location_messages, agent->x, agent->y, 0.0, agent->vel_x, agent->vel_y);
  
    return 0;
}


/**
 * move FLAMEGPU Agent Function
 * Automatically generated using functions.xslt
 * @param agent Pointer to an agent structre of type xmachine_memory_agent. This represents a single agent instance and can be modified directly.
 * @param location_messages  location_messages Pointer to input message list of type xmachine_message__list. Must be passed as an argument to the get_first_location_message and get_next_location_message functions.* @param partition_matrix Pointer to the partition matrix of type xmachine_message_location_PBM. Used within the get_first__message and get_next__message functions for spatially partitioned message access.* @param rand48 Pointer to the seed list of type RNG_rand48. Must be passed as an arument to the rand48 function for genertaing random numbers on the GPU.
 */
__FLAME_GPU_FUNC__ int avoid_pedestrians(xmachine_memory_agent* agent, xmachine_message_pedestrian_location_list* pedestrian_location_messages, xmachine_message_pedestrian_location_PBM* partition_matrix, RNG_rand48* rand48){

	glm::vec2 agent_pos = glm::vec2(agent->x, agent->y);
	glm::vec2 agent_vel = glm::vec2(agent->vel_x, agent->vel_y);

	glm::vec2 repulsive_force = glm::vec2(0, 0);
	glm::vec2 physical_force = glm::vec2(0, 0);

	xmachine_message_pedestrian_location* current_message = get_first_pedestrian_location_message(pedestrian_location_messages, partition_matrix, agent->x, agent->y, 0.0);
	while (current_message)
	{
		glm::vec2 message_pos = glm::vec2(current_message->x, current_message->y);
		float separation = length(agent_pos - message_pos);

		//min distance used to check it is not the own agents message
		if (separation>MIN_DISTANCE){
			glm::vec2 to_agent = normalize(agent_pos - message_pos);
			glm::vec2 other_vel = glm::vec2(current_message->vel_x, current_message->vel_y);



			//Helbing's social forces implementation

			//Repulsive force for each pedestrian
			float A = INTERACTION_STRENGTH; //Interaction strength constant
											//float B = 0.08; //Interaction range constant
			float B = INTERACTION_RANGE; //Interaction range constant
										 //float lambda = 0.4f; //Must be less then 1, set to 0 for isotropic (force equal in all directions)
										 //float cosphi = dot( to_agent * -1.0f , agent_vel ); //Angle between heading direction and the location of the other ped

			repulsive_force += to_agent * A * exp((PED_DIAMETER - separation) / B);// * ( lambda + ((1- lambda)* (1 + cosphi)/2) ); 

																				   //Physical force, the force when pedestrians are touching each other
			if (PED_DIAMETER >= separation)
			{
				//Body force
				float k = 120000.0f; //Body force constant (large)

				float thetaTerm = PED_DIAMETER - separation;
				thetaTerm = thetaTerm >= 0 ? thetaTerm : 0; //Theta term must be zero or positive

															//Sliding friction force
				float kappa = 240000.0f; //sliding friction force constant (large)
				glm::vec2 t = glm::vec2(-to_agent.y, to_agent.x); //Tangential direction
				float deltav = dot(other_vel - agent_vel, t); //Tangential velocity difference
				physical_force += (k * thetaTerm * to_agent) + (kappa * thetaTerm * t * deltav)*COLLISION_WEIGHT;
			}
		}
		 current_message = get_next_pedestrian_location_message(current_message, pedestrian_location_messages, partition_matrix);
	}

	//Combine forces
	glm::vec2 agent_steer = repulsive_force + physical_force;
	agent_steer.x += (agent->goal ? -1 : 1)*GOAL_WEIGHT;
//Add a 3rd collision force with walls?

//Update velocity
	glm::vec2 dv = (agent_steer / AGENT_MASS); //Velocity difference due to force
	agent_vel *= 0.9f;
	agent_vel += dv; //Add velocity difference to overall velocity
	float speed = glm::length(agent_vel);

	//Apply global speed limit (is it necessary?)
	if (speed > MAX_SPEED)
	{
		agent_vel = glm::normalize(agent_vel) * MAX_SPEED;
		speed = MAX_SPEED;
	}

	//update position

	agent_vel = glm::mix(glm::vec2(agent->vel_x, agent->vel_y), agent_vel, 0.1f);
	agent_pos += agent_vel*TIME_SCALER;
	//printf("Pos(%.3f, %.3f) (%.3f, %.3f)\n", agent->x, agent->y, agent_pos.x, agent_pos.y);
	//printf("Vel(%.3f, %.3f) (%.3f, %.3f)\n", agent->vel_x, agent->vel_y, agent_vel.x, agent_vel.y);

	//animation
	agent->animate += (agent->animate_dir * speed * TIME_SCALER);
	if (agent->animate >= 1)
		agent->animate_dir = -1;
	if (agent->animate <= 0)
		agent->animate_dir = 1;

	//lod
	agent->lod = 1;

	//update
	agent->x = agent_pos.x;
	agent->y = agent_pos.y;
	agent->vel_x = agent_vel.x;
	agent->vel_y = agent_vel.y;

	//bound by wrapping
	//if (agent->x < -1.0f)
	//	agent->x += 2.0f;
	//if (agent->x > 1.0f)
	//	agent->x -= 2.0f;
	//if (agent->y < -1.0f)
	//	agent->y += 2.0f;
	//if (agent->y > 1.0f)
	//	agent->y -= 2.0f;

	//h_agent->vel_x = (h_agent->goal ? -1 : 1)*nrml_rng(gen);
	assert(!isnan(agent->x));
	assert(!isnan(agent->y));
	assert(!isnan(agent->vel_x));
	assert(!isnan(agent->vel_y));
	if (agent->goal)
	{
		//assert(agent_vel.x < 0);
		if (agent_pos.x < 1)
			return 1;
	}
	else
	{
		//assert(agent_vel.x > 0);
		if (agent_pos.x > 99)
			return 1;
	}

	return 0;
}

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> int_rng(0, 1);
std::uniform_real_distribution<> flt_rng(0,1);
std::normal_distribution<> nrml_rng( 1,0.2 );
xmachine_memory_agent * h_agent = nullptr;
const int CORRIDOR_LENGTH = 100;
const int CORRIDOR_WIDTH = 50;
__FLAME_GPU_INIT_FUNC__ void init_model()
{
	// Allocate a single agent struct on the host.
	h_agent = h_allocate_agent_agent();
	//Unchanging defaults
	h_agent->vel_y = 0;
	h_agent->animate = 0.5;
	h_agent->animate_dir = 1;
	h_agent->lod = 0;
	h_agent->steer_x = 0;
	h_agent->steer_y = 0;

	float sw = 0.1f*10000;
	set_STEER_WEIGHT(&sw);
	float aw = 1.0f * 50000;
	set_AVOID_WEIGHT(&aw);
	float er = 0.08f;
	set_EMMISION_RATE_EXIT1(&er);
	set_EMMISION_RATE_EXIT2(&er);
}
__FLAME_GPU_EXIT_FUNC__ void exit_model()
{
	// Clear host memory for single struct. The Utility function also deallocates any agent variable arrays.
	h_free_agent_agent(&h_agent);
	h_agent = nullptr;
}
/**
 * generate_pedestrians FLAMEGPU Agent Function
 * Automatically generated using functions.xslt
 */

__FLAME_GPU_STEP_FUNC__ void generate_pedestrians(){
	
	{
		static int t = 0;
		int _t = clock();
		int __t = _t - t;
		t = _t;
		float ts = __t / (float)CLOCKS_PER_SEC;
		set_TIME_SCALER(&ts);
	}
	if (get_agent_agent_default_count() > 1000)
		return;
	if (flt_rng(gen) > *get_EMMISION_RATE_EXIT1()+*get_EMMISION_RATE_EXIT2())
		return;
	//Init this agent
	h_agent->goal = int_rng(gen);
	h_agent->x = h_agent->goal ? CORRIDOR_LENGTH : 0;
	h_agent->y = flt_rng(gen) * CORRIDOR_WIDTH;
	float a = nrml_rng(gen);
	assert(a > 0);
	h_agent->vel_x = (h_agent->goal?-1:1)*a*1.5;
	// Copy agent data from the host to the device
	h_add_agent_agent_default(h_agent);
}

#endif //_FLAMEGPU_FUNCTIONS
