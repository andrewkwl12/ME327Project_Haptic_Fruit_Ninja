#ifndef Fruit_h
#define Fruit_h

class Fruit{
public:
	Fruit();
	Fruit(int dt);
	float getX();
	float getY();
	float getXDot();
	float getYDot();
	float getDiam();
	void setPos(float x, float y);
	void setVel(float xDot, float yDot);
	void setDt(int dt);
	void setDiam(float D);
	void updatePos();
	int isInPlay();
	int isIntersected(float x, float y);
private:
	float dt;
	float D;
	float x;
	float y;
	float xDot;
	float yDot;
	int inPlay;
	int intersected;
};

#endif