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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <cmath>
#include <GL/glew.h>
#include <GL/freeglut.h>

#include "GlobalsController.h"
#include "CustomVisualisation.h"

#ifdef _MSC_VER
// Disable _CRT_SECURE_NO_WARNINGS warnings
#pragma warning(disable:4996)
#endif

//globals, initialised to 0 then loaded from the relevant variable as specified in the model or initial states file (or init function)
float emmisionRateExit1 = 0;
float emmisionRateExit2 = 0;

float timeScaler = 0;

float steerWeight = 0;
float avoidWeight = 0;
float collisionWeight = 0;
float goalWeight = 0;

//imported functions from FLAME GPU
extern void set_EMMISION_RATE_EXIT1(float* h_EMMISION_RATE);
extern void set_EMMISION_RATE_EXIT2(float* h_EMMISION_RATE);

extern void set_TIME_SCALER(float* h_EMMISION_RATE);

extern void set_STEER_WEIGHT(float* h_weight);
extern void set_AVOID_WEIGHT(float* h_weight);
extern void set_COLLISION_WEIGHT(float* h_weight);
extern void set_GOAL_WEIGHT(float* h_weight);

extern const float * get_EMMISION_RATE_EXIT1();
extern const float * get_EMMISION_RATE_EXIT2();

extern const float * get_TIME_SCALER();

extern const float * get_STEER_WEIGHT();
extern const float * get_AVOID_WEIGHT();
extern const float * get_COLLISION_WEIGHT();
extern const float * get_GOAL_WEIGHT();

void initGlobalsController()
{
	// Fetch each value 
	timeScaler = *get_TIME_SCALER();
	steerWeight = *get_STEER_WEIGHT();
	avoidWeight = *get_AVOID_WEIGHT();
	collisionWeight = *get_COLLISION_WEIGHT();
	goalWeight = *get_GOAL_WEIGHT();

	emmisionRateExit1 = *get_EMMISION_RATE_EXIT1();
	emmisionRateExit2 = *get_EMMISION_RATE_EXIT2();
}

//global emmision rate

void increaseGlobalEmmisionRate()
{
	increaseEmmisionRateExit1();
	increaseEmmisionRateExit2();
}

void decreaseGlobalEmmisionRate()
{
	decreaseEmmisionRateExit1();
	decreaseEmmisionRateExit2();
}

/* EMMISION RATES */


//emmision rate exit 1
void increaseEmmisionRateExit1()
{
	emmisionRateExit1 += EMISSION_RATE_INCREMENT;
	set_EMMISION_RATE_EXIT1(&emmisionRateExit1);
}
void decreaseEmmisionRateExit1()
{
	emmisionRateExit1 -= EMISSION_RATE_INCREMENT;
	set_EMMISION_RATE_EXIT1(&emmisionRateExit1);
}
float getEmmisionRateExit1(){	return emmisionRateExit1;}
void setEmmisionRateExit1Text(char* text)
{	
	float rate_pm = emmisionRateExit1 * getFPS() * 60.0f * timeScaler;
	sprintf(text, "Emmision Rate Exit 1: %f", rate_pm);
}

//emmision rate exit 2
void increaseEmmisionRateExit2()
{
	emmisionRateExit2 += EMISSION_RATE_INCREMENT;
	set_EMMISION_RATE_EXIT2(&emmisionRateExit2);
}
void decreaseEmmisionRateExit2()
{
	emmisionRateExit2 -= EMISSION_RATE_INCREMENT;
	set_EMMISION_RATE_EXIT2(&emmisionRateExit2);
}
float getEmmisionRateExit2(){	return emmisionRateExit2;}
void setEmmisionRateExit2Text(char* text)
{	
	float rate_pm = emmisionRateExit2 * getFPS() * 60.0f * timeScaler;
	sprintf(text, "Emmision Rate Exit 2: %f", rate_pm);
}

//time scaler
void increaseTimeScaler()
{
	timeScaler += TIME_SCALER_INCREMENT;
	set_TIME_SCALER(&timeScaler);
}

void decreaseTimeScaler()
{
	timeScaler -= TIME_SCALER_INCREMENT;
	//prevent negative time scaler
	if (timeScaler < 0)
		timeScaler += TIME_SCALER_INCREMENT;
	set_TIME_SCALER(&timeScaler);
}

float getTimeScaler()
{
	return timeScaler;
}

void setTimeScalerText(char* text)
{
	sprintf(text, "Time Scaler: %f", timeScaler);
}

/* RULE WEIGHTS */
//steer
void increaseSteerWeight(){
	steerWeight += STEER_WEIGHT_INCREMENT;
	set_STEER_WEIGHT(&steerWeight);
}
void decreaseSteerWeight(){
	steerWeight -= STEER_WEIGHT_INCREMENT;
	if (steerWeight < 0)
		steerWeight = 0;
	set_STEER_WEIGHT(&steerWeight);
}
float getSteerWeight(){
	return steerWeight;
}
void setSteerWeightText(char* text){
	sprintf(text, "Steer Rule Weight: %f", steerWeight);
}
//avoid
void increaseAvoidWeight(){
	avoidWeight += AVOID_WEIGHT_INCREMENT;
	set_AVOID_WEIGHT(&avoidWeight);
}
void decreaseAvoidWeight(){
	avoidWeight -= AVOID_WEIGHT_INCREMENT;
	if (avoidWeight < 0)
		avoidWeight = 0;
	set_AVOID_WEIGHT(&avoidWeight);
}
float getAvoidWeight(){
	return avoidWeight;
}
void setAvoidWeightText(char* text){
	sprintf(text, "Avoid Rule Weight: %f", avoidWeight);
}
//collision
void increaseCollisionWeight(){
	collisionWeight += COLLISION_WEIGHT_INCREMENT;
	set_COLLISION_WEIGHT(&collisionWeight);
}
void decreaseCollisionWeight(){
	collisionWeight -= COLLISION_WEIGHT_INCREMENT;
	if (collisionWeight < 0)
		collisionWeight = 0;
	set_COLLISION_WEIGHT(&collisionWeight);
}
float getCollisionWeight(){
	return collisionWeight;
}
void setCollisionWeightText(char* text){
	sprintf(text, "Collision Rule Weight: %f", collisionWeight);
}
//goal
void increaseGoalWeight(){
	goalWeight += GOAL_WEIGHT_INCREMENT;
	set_GOAL_WEIGHT(&goalWeight);
}
void decreaseGoalWeight(){
	goalWeight -= GOAL_WEIGHT_INCREMENT;
	if (goalWeight < 0)
		goalWeight = 0;
	set_GOAL_WEIGHT(&goalWeight);
}
float getGoalWeight(){
	return goalWeight;
}
void setGoalWeightText(char* text){
	sprintf(text, "Goal Rule Weight: %f", goalWeight);
}
