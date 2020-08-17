#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include <vector>
#include <tuple>
#include <limits>

// Override base class with your custom functionality

class Boid {
public:
	static std::vector<Boid> boid_list; //list of all agents
	static std::map< std::pair<int, int>, std::vector<Boid> > grid_table; //spatially partitioned agent map
	
	int m_id; //agent id
	olc::vf2d m_pos; //agent position

	float m_heading;
	float m_heading_target;

private:
	olc::vf2d m_vel;
	olc::vf2d m_accel;

	olc::Pixel m_col; //color of boid
	
	float m_max_force; //constants that determine motion characteristics
	float m_max_speed;
	float m_max_turnspeed;
	float m_repulse_strength;
	float m_equil_dist;
	float m_cohesion_strength;
	float m_alignment_strength;

	float m_vision; //vision range

	int m_gx; //grid coordinate position
	int m_gy;
public:
	Boid(int id) {
		int m_id = id;
		//random position
		olc::vf2d startpos(static_cast <float> (rand()) / static_cast <float> (RAND_MAX / BoidsApp::Nx), static_cast <float> (rand()) / static_cast <float> (RAND_MAX / BoidsApp::Ny));
		m_col = olc::BLUE;

		m_max_force = 0.6;
		m_max_speed = 100.0f * (0.75f + 0.5f * static_cast<float> (rand()) / static_cast<float> (RAND_MAX)); //give it some fuzz
		m_max_turnspeed = 3; //about 0.5 rev/s
		m_repulse_strength = 2400; //might wanna check this, chief
		m_equil_dist = 4;
		m_cohesion_strength = m_repulse_strength / m_equil_dist;
		m_alignment_strength = 1;

		m_vision = (float)BoidsApp::grid_size;
	}

	static void gen_table(){
		//take boid_list and apply spatial partitioning
	}

	void update(float fElapsedTime) {
		//update self
	}

	void draw() {
		//draw self (could be pulled out if necessary)
	}

	std::vector<Boid> find_neighbours(int rg = 1){
		//get neighbours using grid table
	}
};

std::vector<Boid> boid_list;
std::map< std::pair<int, int>, std::vector<Boid> > Boid::grid_table; //initialise static var

class BoidsApp : public olc::PixelGameEngine
{
public:
	BoidsApp()
	{
		// Name your application
		sAppName = "Boids v2";
	}

	static const int Nx = 1280;
	static const int Ny = 720;
	static const int grid_size = 50;
	static const int bar_width = 1;
	static int Gx; //number of grid squares across (determined in gen_grid func)
	static int Gy;


public:
	void place_agents() {
		//initialise grid
		
	}

	std::tuple<float, olc::vf2d> measure_distance(olc::vf2d vec1, olc::vf2d vec2) {
		//measure distance between points and create normal pointing FROM vec1 TO vec2
		//(there is a subtlety here due to toroidal world requirements)
		float best = std::numeric_limits<float>::infinity();
		float reflection_mag2;
		olc::vf2d reflection;
		for (int i = -1; i < 2; i++) {
			for (int j = -1; j < 2; j++) {
				reflection = (vec2 + olc::vf2d(Nx * i, Ny * j)) - vec1; //create nine mirror images and check them all
				reflection_mag2 = reflection.mag2();
				if (reflection_mag2 < best);
			}
		}

	}

	bool OnUserCreate() override
	{
		Gx = Nx / grid_size;
		Gy = Ny / grid_size;
		place_agents();
		Boid::gen_table();
		// Called once at the start, so create things here
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		//draw gridlines first
		for (int i = 0; i < Gx; i++) {
			DrawLine({ 0, i * grid_size }, { Ny, i * grid_size }, olc::GREY); //vert lines
		}
		for (int i = 0; i < Gy; i++) {
			DrawLine({ i * grid_size, 0 }, { i * grid_size, Nx }, olc::GREY); //horiz lines
		}

		// called once per frame, draws random coloured pixels
		for (int x = 0; x < ScreenWidth(); x++)
			for (int y = 0; y < ScreenHeight(); y++)
				Draw(x, y, olc::Pixel(rand() % 256, rand() % 256, rand() % 256));
		return true;
	}
};

int main()
{
	BoidsApp app;
	if (app.Construct(256, 240, 4, 4))
		app.Start();
	return 0;
}