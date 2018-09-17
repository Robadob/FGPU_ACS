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
#ifndef _FLAMEGPU_FUNCTIONS
#define _FLAMEGPU_FUNCTIONS

#include "header.h"
#include "CustomVisualisation.h"

#define SCALE_FACTOR 0.03125

#define I_SCALER (SCALE_FACTOR*0.35f)
#define MESSAGE_RADIUS d_message_pedestrian_location_radius
#define MIN_DISTANCE 0.5f

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

    
	add_pedestrian_location_message(pedestrian_location_messages, agent->x, agent->y, 0.0);
  
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
	glm::vec2 navigate_velocity = glm::vec2(0.0f, 0.0f);
	glm::vec2 avoid_velocity = glm::vec2(0.0f, 0.0f);
	xmachine_message_pedestrian_location* current_message = get_first_pedestrian_location_message(pedestrian_location_messages, partition_matrix, agent->x, agent->y, 0.0);
	while (current_message)
	{
		glm::vec2 message_pos = glm::vec2(current_message->x, current_message->y);
		float separation = length(agent_pos - message_pos);
		if ((separation < MESSAGE_RADIUS)&&(separation>MIN_DISTANCE)){
			glm::vec2 to_agent = normalize(agent_pos - message_pos);
			float ang = acosf(dot(agent_vel, to_agent));
			float perception = 45.0f;

			//STEER
			if ((ang < RADIANS(perception)) || (ang > 3.14159265f-RADIANS(perception))){
				glm::vec2 s_velocity = to_agent;
				s_velocity *= powf(I_SCALER/separation, 1.25f)*STEER_WEIGHT;
				navigate_velocity += s_velocity;
			}

			//AVOID
			glm::vec2 a_velocity = to_agent;
			a_velocity *= powf(I_SCALER/separation, 2.00f)*AVOID_WEIGHT;
			avoid_velocity += a_velocity;						

		}
		 current_message = get_next_pedestrian_location_message(current_message, pedestrian_location_messages, partition_matrix);
	}

	//maximum velocity rule
	glm::vec2 agent_steer = navigate_velocity + avoid_velocity;
	//move
	agent_steer.x += (agent->goal ? -1 : 1);
	float current_speed = length(agent_vel) + 0.025f;//(powf(length(agent_vel), 1.75f)*0.01f)+0.025f;

	//apply more steer if speed is greater
	agent_vel += current_speed*normalize(agent_steer);
	float speed = length(agent_vel);
	//limit speed
	if (speed >= 2){
		agent_vel = normalize(agent_vel)*2.0f;
	}

	//update position
	agent_pos += agent_vel*TIME_SCALER;
	//printf("Pos(%.3f, %.3f) (%.3f, %.3f)\n", agent->x, agent->y, agent_pos.x, agent_pos.y);
	//printf("Vel(%.3f, %.3f) (%.3f, %.3f)\n", agent->vel_x, agent->vel_y, agent_vel.x, agent_vel.y);

	//animation
	agent->animate += (agent->animate_dir * speed * 0.5f * TIME_SCALER);
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
	float er = 0.1f;
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
