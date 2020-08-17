#define OLC_PGE_APPLICATION
#define LOG(x) (std::cout << x << std::endl) //homebrew logger
#include "olcPixelGameEngine.h"
#include <vector>
#include <tuple>
#include <limits>
#include <math.h>

// Override base class with your custom functionality

constexpr int Nx = 960;
constexpr int Ny = 480;
constexpr int pixelscale = 1;

const int grid_size = 50;
const int bar_width = 1;
int Gx; //number of grid squares across (determined in gen_grid func)
int Gy;

class Boid {
public:
	static std::vector<Boid> boid_list; //list of all agents
	static std::map< std::pair<int, int>, std::vector<Boid> > grid_table; //spatially partitioned agent map
	
	int m_id; //agent id
	olc::vf2d m_pos; //agent position

	float m_heading;
	float m_heading_target;


	olc::vf2d m_vel;
	olc::vf2d m_accel;

	int m_size;
	olc::Pixel m_col; //color of boid
private:
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
		m_size = 5;
		//random position
		olc::vf2d startpos(static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Nx), static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Ny));
		m_col = olc::BLUE;

		m_max_force = 0.6;
		m_max_speed = 100.0f * (0.75f + 0.5f * static_cast<float> (rand()) / static_cast<float> (RAND_MAX)); //give it some fuzz
		m_max_turnspeed = 3; //about 0.5 rev/s
		m_repulse_strength = 2400; //might wanna check this, chief
		m_equil_dist = 4;
		m_cohesion_strength = m_repulse_strength / m_equil_dist;
		m_alignment_strength = 1;

		m_vision = (float)grid_size;

		//set grid coordinates
		/*self.gx,self.gy = self.pos//grid_size #current grid cell (goes from -1 to 8)
        if self.gx < 0:
            self.gx = Gx
        elif self.gx > Gx:
            self.gx = 0
        if self.gy < 0:
            self.gy = Gy
        elif self.gy > Gy:
            self.gy = 0*/

		m_gx = (int)(m_pos.x / grid_size);
		m_gy = (int)(m_pos.y / grid_size);
		//toroidal rollover
		if (m_gx < 0) { m_gx = Gx; }
		if (m_gx > 0) { m_gx = 0; }
		if (m_gy < 0) { m_gy = Gy; }
		if (m_gy > 0) { m_gy = 0; }

	}

	static void gen_table(){
		//take boid_list and apply spatial partitioning
	}

	void update(float fElapsedTime) {
		//update self
		m_pos += fElapsedTime * m_vel;
		m_vel += fElapsedTime * m_accel;
		if (m_vel.mag2() > m_max_speed * m_max_speed) {
			m_vel = m_max_speed * m_vel.norm();
		}
		LOG(m_vel);
		m_accel = olc::vf2d(0, 0);
		//clamp position
		if (m_pos.x < 0) { m_pos.x = Nx; }
		else if (m_pos.x > Nx) { m_pos.x = 0; }
		if (m_pos.y < 0) { m_pos.y = Ny; }
		else if (m_pos.y > Ny) { m_pos.y = 0; }
		//m_pos.x = ((int)m_pos.x) % Nx;
		//m_pos.y = ((int)m_pos.y) % Ny;
	}

	std::vector<Boid> find_neighbours(int rg = 1){
		//get neighbours using grid table
	}

	void calculate_movement() {
		m_accel = olc::vf2d(0, -10);
		//pass
	}
};


std::vector<Boid> Boid::boid_list;
std::map< std::pair<int, int>, std::vector<Boid> > Boid::grid_table; //initialise static var

class BoidsApp : public olc::PixelGameEngine
{
public:
	BoidsApp()
	{
		// Name your application
		sAppName = "Boids v2";
	}

public:
	void place_agents() {
		//initialise grid
		
	}

	std::tuple<float, olc::vf2d> measure_distance(olc::vf2d vec1, olc::vf2d vec2) {
		//measure distance between points and create normal pointing FROM vec1 TO vec2
		//(there is a subtlety here due to toroidal world requirements)
		float best_mag2 = std::numeric_limits<float>::infinity();
		olc::vf2d best_reflection;

		float reflection_mag2;
		olc::vf2d reflection;
		for (int i = -1; i < 2; i++) {
			for (int j = -1; j < 2; j++) {
				reflection = (vec2 + olc::vf2d(Nx * i, Ny * j)) - vec1; //create nine mirror images and check them all
				reflection_mag2 = reflection.mag2();
				if (reflection_mag2 < best_mag2) {
					best_mag2 = reflection_mag2;
					best_reflection = reflection;
				}
			}
		}
		return std::make_tuple((float)std::sqrt(best_mag2), reflection);
	}
	bool OnUserCreate() override
	{
		Gx = Nx / grid_size;
		Gy = Ny / grid_size;
		place_agents();
		Boid::gen_table();
		// Called once at the start, so create things here
		//debug loner

		Boid loner(0);
		loner.m_pos = olc::vf2d(Nx / 2, Ny / 2);
		loner.m_vel = olc::vf2d(40,40);
		Boid::boid_list.push_back(loner);
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		Clear(olc::BLACK);
		//draw gridlines first
		//for (int i = 0; i < Gy+1; i++) {
		//	DrawLine({ 0, i * grid_size }, { Nx, i * grid_size }, olc::GREY); //horiz lines
		//}
		//for (int i = 0; i < Gx+1; i++) {
		//	DrawLine({ i * grid_size, 0 }, { i * grid_size, Ny }, olc::GREY); //vert lines
		//}

		for (int i = 0; i < Boid::boid_list.size(); i++) {
			Boid* candidate = &Boid::boid_list[i];
			(*candidate).calculate_movement();
			(*candidate).update(fElapsedTime);
			FillCircle((*candidate).m_pos, (*candidate).m_size, (*candidate).m_col);
			//LOG((*candidate).m_pos);
			//draw_boid()

		}
		// called once per frame, draws random coloured pixels
		/*for (int x = 0; x < ScreenWidth(); x++)
			for (int y = 0; y < ScreenHeight(); y++)
				Draw(x, y, olc::Pixel(rand() % 256, rand() % 256, rand() % 256));*/
		return true;
	}
};

int main()
{
	BoidsApp app;
	if (app.Construct(Nx, Ny, pixelscale, pixelscale))
		app.Start();
	return 0;
}