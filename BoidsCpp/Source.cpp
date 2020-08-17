/*
----------
 BOIDS v2
----------
by Antiochian 18/08/2020

An implementation of the "Boids" flocking algorithm in C++ using the OLC PixelGameEngine library as a graphics wrapper


PROBLEM TRACKER:
problem:
	searches based on number of grid squares away, which is fine except that if one grid square is smaller than the rest
	(i.e. when Nx is not a clean mutliple of grid_size) then search range is smaller than it should be.
solution:
	Its not ideal, but we can err on the side of caution and search 1.5 grid squares rather than 0.5, as the later manual
	distance computation will "clean up" and eliminate anything too far away anyway

problem:
	i used global variables
solution:
	dont look at them and hope nobody else looks at them either
*/

#define OLC_PGE_APPLICATION
#define LOG(x) (std::cout << x << std::endl) //homebrew logger for debugging
#define _USE_MATH_DEFINES

#include "olcPixelGameEngine.h"

#include <iterator>
#include <vector>
#include <tuple>
#include <limits>
#include <math.h>
#include <chrono>

constexpr int Nx = 1280;
constexpr int Ny = 720;
constexpr int default_num_of_boids = 400;

constexpr int pixelscale = 1;

constexpr int grid_size = 50;

int Gx; //number of grid squares across (determined in gen_grid func)
int Gy;



class Boid {
public:
	static std::vector<Boid> boid_list; //list of all agents
	static std::map< std::pair<int, int>, std::vector<Boid*> > grid_table; //spatially partitioned agent map
	
	int m_id; //agent id

	olc::vf2d m_pos; //agent position and other quantities of motion
	olc::vf2d m_vel;
	olc::vf2d m_accel;

	int m_size;
	float m_vision; //vision range
	olc::Pixel m_col; //color

private:
	float m_max_force; //tweakable constants that determine motion characteristics
	float m_max_speed;
	float m_repulse_strength;
	float m_equil_dist;
	float m_cohesion_strength;
	float m_alignment_strength;

	int m_gx; //grid coordinate position
	int m_gy;

public:
	Boid(int id, olc::Pixel col, int size = 2) {
		m_id = id;
		m_size = size;
		m_col = col;
		if (m_id == 0) { m_col = olc::RED; } //highlight the ID=0 boid in red

		//fuzz max speed so the boids arent all the same
		m_max_speed = 100.0f * (0.75f + 0.5f * static_cast<float> (rand()) / static_cast<float> (RAND_MAX));

		//start off with random position
		olc::vf2d startpos(static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Nx), static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Ny));
		m_pos = startpos;
		//start off with random speed in a random heading
		float heading = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / (2*M_PI));
		float start_speed = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / m_max_speed);
		m_vel = start_speed * olc::vf2d(std::cos(heading), std::sin(heading));
		
		//tweakables
		m_max_force = 100;
		m_repulse_strength = 2.5; //might wanna check this, chief
		m_equil_dist = 4;
		m_cohesion_strength = m_repulse_strength / m_equil_dist;
		m_alignment_strength = 10;
		m_vision = (float)grid_size;

		//grid coordinates
		m_gx = (int)(m_pos.x / grid_size); //this could be modulo'd but the position is clamped seperately in update() so its not necessary
		m_gy = (int)(m_pos.y / grid_size);
	}
	
	static olc::vf2d rotate(olc::vf2d input, float angle) {
		//vestigal
		olc::vf2d output;
		output.x = input.x * std::cosf(angle) - input.y * std::sinf(angle);
		output.y = input.x * std::sinf(angle) + input.y * std::cosf(angle);
		return output;
	}

	static std::tuple<float, olc::vf2d> measure_distance(olc::vf2d vec1, olc::vf2d vec2) {
		//measure distance between points and create normal pointing FROM vec1 TO vec2
		//(there is a subtlety here due to toroidal world boundaries that means 9 possibilities have to be considered)
		float best_mag2 = std::numeric_limits<float>::infinity(); //squares are faster
		olc::vf2d best_reflection;

		float reflection_mag2;
		olc::vf2d reflection;
		for (int i = -1; i < 2; i++) {
			for (int j = -1; j < 2; j++) {
				reflection = (vec2 + olc::vf2d(Nx * i, Ny * j)) - vec1; //create nine mirror images and check them all
				reflection_mag2 = reflection.mag2(); //squares are faster
				if (reflection_mag2 < best_mag2) {
					best_mag2 = reflection_mag2;
					best_reflection = reflection;
				}
			}
		}
		float mag = std::sqrt(best_mag2);
		return std::make_tuple((float)mag, best_reflection);
	}
	static void gen_table(){
		//take boid_list and apply my homemade spatial partitioning algorithm
		Boid::grid_table.clear(); //format is {(gx,gy) : [boid1, boid2, ...]}
		float tx;
		float ty;
		std::pair<int, int> key;
		for (int i = 0; i < Boid::boid_list.size(); i++) {
			tx = boid_list[i].m_gx;
			ty = boid_list[i].m_gy;
			key = std::make_pair(tx, ty);
			Boid* ptr = &(Boid::boid_list[i]);
			
			if (Boid::grid_table.find(key) == grid_table.end()) {
				//if key not found then make new vec and add it to map
				std::vector<Boid*> t_vec({ ptr });
				//std::pair< std::pair<int,int>, std::vector<Boid*> > (key, t_vec)
				Boid::grid_table.insert(std::pair< std::pair<int, int>, std::vector<Boid*> >(key, t_vec));
			}
			else {
				Boid::grid_table[key].push_back(ptr); //add to grid
			}
		}
	}

	void update(float fElapsedTime) {
		//update self
		m_pos += fElapsedTime * m_vel;
		if (m_accel.mag2() > m_max_force * m_max_force) {
			m_accel = m_max_force * m_accel.norm();
		}
		m_vel += fElapsedTime * m_accel;
		if (m_vel.mag2() > m_max_speed * m_max_speed) {
			m_vel = m_max_speed * m_vel.norm();
		}
		m_accel = olc::vf2d(0, 0);
		//clamp position
		if (m_pos.x < 0) { m_pos.x = Nx; }
		else if (m_pos.x > Nx) { m_pos.x = 0; }
		if (m_pos.y < 0) { m_pos.y = Ny; }
		else if (m_pos.y > Ny) { m_pos.y = 0; }
		//m_pos.x = ((int)m_pos.x) % Nx;
		//m_pos.y = ((int)m_pos.y) % Ny;

		m_gx = m_pos.x / grid_size;
		m_gy = m_pos.y / grid_size;
	}

	std::vector<Boid*> find_neighbours(int rg){
		std::vector<Boid*> candidate_neighbours;
		std::vector<Boid*> neighbours;
		std::vector<Boid*> res;
		//get neighbours using grid table
		std::vector<int> gx_range; //range of gx to check
		std::vector<int> gy_range; //range of gy to check

		for (int i = m_gx - rg; i < m_gx + rg + 1; i++) { gx_range.push_back(i); }
		for (int i = m_gy - rg; i < m_gy + rg + 1; i++) { gy_range.push_back(i); }

		//get fast overcount using spatial partitioning
		std::pair<int, int> key;
		for (std::vector<int>::iterator i = gx_range.begin(); i != gx_range.end(); i++) {
			for (std::vector<int>::iterator j = gy_range.begin(); j != gy_range.end(); j++) {
				key = std::make_pair( *i, *j );
				res = Boid::grid_table[key]; //this will fill up the grid table with empty vecs but i think thats okay?
				candidate_neighbours.insert(candidate_neighbours.end(), res.begin(), res.end());
			}
		}

		//now check the  ACTUAL distance and ID numbers to prune as necessary
		for (int i = 0; i < candidate_neighbours.size(); i++) {
			//deleting elements from the middle of vectors is an inefficient operation, so we create 
			//a new vector instead and then discard the old candidates naturally when the scope is exited
			olc::vf2d candidate_pos = (*candidate_neighbours[i]).m_pos;
			int candidate_id = (*candidate_neighbours[i]).m_id;

			if ((candidate_id != m_id) && ( std::get<0>(Boid::measure_distance(candidate_pos, m_pos))< m_vision)) {
					neighbours.push_back(candidate_neighbours[i]);
					//add to list
			}
		}
		return neighbours;
	}

	void calculate_movement(float fElapsedTime) {
		std::vector<Boid*> neighbours = find_neighbours((int) (m_vision/grid_size) + 1);
		olc::vf2d avg_pos(0, 0);
		olc::vf2d avg_vel(0, 0);
		bool exist_neighbours = false; //flag to avoid no-neighbour case

		for (int i = 0; i < neighbours.size(); i++) {
			exist_neighbours = true;
			Boid* neighbour = neighbours[i];
			//apply repulsion
			float mag;
			olc::vf2d norm;
			std::tie(mag, norm) = Boid::measure_distance((*neighbour).m_pos, m_pos);
			if (mag != 0){
				m_accel += m_repulse_strength * norm / mag;
			}
			else {
				m_accel += m_repulse_strength * norm / 0.0001; //divide by zero fudge
			}
			//keep running averages
			avg_pos += (*neighbour).m_pos;
			avg_vel += m_max_speed*(*neighbour).m_vel.norm();
		}

		//only bother trying to apply these things if there are actual neighbours to deal with
		if (exist_neighbours) {
			olc::vf2d cohesion_target = avg_pos / neighbours.size();
			olc::vf2d alignment_target = avg_vel / neighbours.size();
			float mag;
			olc::vf2d normal;
			std::tie(mag, normal) = Boid::measure_distance(m_pos, cohesion_target);
			//m_accel += m_cohesion_strength * normal;
			m_accel += m_cohesion_strength * (cohesion_target - m_pos);
			m_accel += m_alignment_strength * (alignment_target - m_vel);
		}
	}
};


class BoidsApp : public olc::PixelGameEngine
{
public:
	static olc::Pixel bgcolor;
	static olc::Pixel gridcolor;
	static olc::Pixel boidcolor;
	static olc::Pixel highlightcolor;

private:
	int next_ID;
	int num_of_boids;

public:
	BoidsApp()
	{
		//Name
		sAppName = "Boids v2";
	}

	void place_agents() {
		//clear the old data to make room for the new
		Boid::boid_list.clear();
		for (int i = 0; i < num_of_boids; i++) {
			Boid agent(next_ID, boidcolor);
			next_ID++;
			Boid::boid_list.push_back(agent);
		}	
	}

	void reset_scene() {
		//reset state variables
		next_ID = 0;
		num_of_boids = default_num_of_boids;

		int t_seed = time(0);
		srand(t_seed);
		auto start = std::chrono::high_resolution_clock::now(); //timing data

		//actual reset happens here
		place_agents(); 
		Boid::gen_table();
		
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> t_time(finish - start);
		std::cout << num_of_boids << " agents generated in " << t_time.count() << " seconds" << std::endl;
		std::cout << "Seed: " << t_seed << std::endl;
	}

	void create_boid(olc::vf2d pos) {
		//adds new boid to scene on-the-fly
		num_of_boids += 1;
		Boid agent(next_ID, boidcolor);
		agent.m_pos = pos;
		next_ID++;
		Boid::boid_list.push_back(agent);
		Boid::gen_table();

	}

	bool OnUserCreate() override
	{
		//called on startup
		bgcolor = olc::Pixel(253, 246, 227);
		gridcolor = olc::Pixel(238, 232, 213);
		boidcolor = olc::Pixel(38, 139, 210);
		highlightcolor = olc::Pixel(220, 50, 47);
		
		Gx = Nx / grid_size;
		Gy = Ny / grid_size;
		reset_scene();
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override
	{
		//called every frame
		Boid::gen_table();
		Clear(bgcolor);
		//draw gridlines first
		for (int i = 0; i < Gy+1; i++) {
			DrawLine({ 0, i * grid_size }, { Nx, i * grid_size }, gridcolor, 0xF0F0F0F0); //dotted horiz lines
		}
		for (int i = 0; i < Gx+1; i++) {
			DrawLine({ i * grid_size, 0 }, { i * grid_size, Ny }, gridcolor, 0xF0F0F0F0); //dotted vert lines
		}



		for (int i = 0; i < Boid::boid_list.size(); i++) {
			Boid* candidate = &Boid::boid_list[i];
			(*candidate).calculate_movement(fElapsedTime);
			(*candidate).update(fElapsedTime);
			FillCircle((*candidate).m_pos, (*candidate).m_size, (*candidate).m_col); //draw boid

			if ((GetMouse(1).bHeld) && ((*candidate).m_id == 0)) { //draw debug trace on right mouseclick
				std::vector<Boid*> neighbours = (*candidate).find_neighbours((*candidate).m_vision / grid_size);
				for (std::vector<Boid*>::iterator it = neighbours.begin(); it != neighbours.end(); it++) {
					olc::vf2d targ_pos = (*(*it)).m_pos;
					DrawLine((*candidate).m_pos, targ_pos, highlightcolor, 0xF0F0F0F0);
				}
			}
		}

		if (GetMouse(0).bPressed) { //add new boid on left mouseclick
			std::cout << "Making new boid" << std::endl;
			create_boid(GetMousePos());
		}

		if (GetKey(olc::SPACE).bPressed) { //reset scene on spacebar press
			reset_scene();
		}

		return true;
	}
};

//initialise static var
std::vector<Boid> Boid::boid_list;
std::map< std::pair<int, int>, std::vector<Boid*> > Boid::grid_table; 

olc::Pixel BoidsApp::bgcolor;
olc::Pixel BoidsApp::gridcolor;
olc::Pixel BoidsApp::boidcolor;
olc::Pixel BoidsApp::highlightcolor;

int main()
{
	BoidsApp app;
	if (app.Construct(Nx, Ny, pixelscale, pixelscale))
		app.Start();
	return 0;
}