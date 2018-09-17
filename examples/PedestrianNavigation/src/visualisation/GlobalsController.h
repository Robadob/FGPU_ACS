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
#ifndef __GLOBALS_CONTROLLER
#define __GLOBALS_CONTROLLER

#define EMISSION_RATE_INCREMENT 0.01f
#define TIME_SCALER_INCREMENT	0.00001f
#define STEER_WEIGHT_INCREMENT		0.001f
#define AVOID_WEIGHT_INCREMENT		0.001f
#define COLLISION_WEIGHT_INCREMENT	0.001f
#define GOAL_WEIGHT_INCREMENT		0.001f

void initGlobalsController();

void increaseGlobalEmmisionRate();
void decreaseGlobalEmmisionRate();

//emmision rates
void increaseEmmisionRateExit1();
void decreaseEmmisionRateExit1();
float getEmmisionRateExit1();
void setEmmisionRateExit1Text(char* text);
void increaseEmmisionRateExit2();
void decreaseEmmisionRateExit2();
float getEmmisionRateExit2();
void setEmmisionRateExit2Text(char* text);

//time
void increaseTimeScaler();
void decreaseTimeScaler();
float getTimeScaler();
void setTimeScalerText(char* text);

//rule weights
void increaseSteerWeight();
void decreaseSteerWeight();
float getSteerWeight();
void setSteerWeightText(char* text);
void increaseAvoidWeight();
void decreaseAvoidWeight();
float getAvoidWeight();
void setAvoidWeightText(char* text);
void increaseCollisionWeight();
void decreaseCollisionWeight();
float getCollisionWeight();
void setCollisionWeightText(char* text);
void increaseGoalWeight();
void decreaseGoalWeight();
float getGoalWeight();
void setGoalWeightText(char* text);

#endif //__GLOBALS_CONTROLLER
