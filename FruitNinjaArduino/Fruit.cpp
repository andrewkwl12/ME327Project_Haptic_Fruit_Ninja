#include <Arduino.h>
#include "Fruit.h"

//Default constructor
Fruit::Fruit(){
	this->dt = 0.02;
	this->inPlay = 0;
}

//Overloaded constructor
Fruit::Fruit(int dt){
	this->dt = float(dt/1000.0);
	this->inPlay = 0;
}

float Fruit::getX(){
	return this->x;
}

float Fruit::getY(){
	return this->y;
}

float Fruit::getXDot(){
	return this->xDot;
}

float Fruit::getYDot(){
	return this->yDot;
}

float Fruit::getDiam(){
	return this->D;
}

int Fruit::isInPlay(){
	return this->inPlay;
}

int Fruit::isIntersected(float x, float y){
	float xDiff = x - this->x;
	float yDiff = y - this->y;
	float dist = sqrt(pow(xDiff,2.0)+pow(yDiff,2.0));
	if(dist <= this->D/2.0){
		return 1;
	}else{
		return 0;
	}
}

void Fruit::setPos(float x, float y){
	this->x = x;
	this->y = y;
}

void Fruit::setVel(float xDot, float yDot){
	this->inPlay = 1;
	this->xDot = xDot;
	this->yDot = yDot;
}

void Fruit::setDt(int dt){
	this->dt = float(dt/1000.0);
}

void Fruit::setDiam(float D){
	this->D = D;
}

void Fruit::updatePos(){
	if(this->y >= 0){
		this->x = this->x + this->xDot*this->dt;
		this->y = this->y + this->yDot*this->dt;
		this->yDot = this->yDot - 9.8/2.0*dt;
	}else{
		this->inPlay = 0;
		this->x = 0;
		this->y = 0;
		this->xDot = 0;
		this->yDot = 0;
	}
}
