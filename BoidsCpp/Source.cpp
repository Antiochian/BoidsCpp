#define OLC_PGE_APPLICATION
#define LOG(x) (std::cout << x << std::endl) //homebrew logger
#define _USE_MATH_DEFINES

#include "olcPixelGameEngine.h"

#include <iterator>
#include <vector>
#include <tuple>
#include <limits>
#include <math.h>


/*
KNOWN ISSUES:
searches based on number of grid squares away, which is fine except that if one grid square is smaller than the rest 
(i.e. when Nx is not a clean mutliple of grid_size) then search range is smaller than it should be.

proposed solutoin:
Its not ideal, but it would be better to search 1.5 grid square rather than 0.5, as the later manual distance computation will eliminate anything to far away
*/
// Override base class with your custom functionality

constexpr int Nx = 1280;
constexpr int Ny = 720;

constexpr int num_of_boids = 200;
constexpr int pixelscale = 1;

const int grid_size = 50;
const int bar_width = 1;
int Gx; //number of grid squares across (determined in gen_grid func)
int Gy;

class Boid {
public:
	static std::vector<Boid> boid_list; //list of all agents
	static std::map< std::pair<int, int>, std::vector<Boid*> > grid_table; //spatially partitioned agent map
	
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
		m_id = id;

		m_size = 2;
		m_col = olc::BLUE;
		if (m_id == 0) { m_col = olc::RED; }

		m_max_speed = 100.0f * (0.75f + 0.5f * static_cast<float> (rand()) / static_cast<float> (RAND_MAX)); //give it some fuzz

		//start off with random position + speed
		olc::vf2d startpos(static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Nx), static_cast <float> (rand()) / static_cast <float> (RAND_MAX / Ny));
		olc::vf2d startvel(static_cast <float> (rand()) / static_cast <float> (RAND_MAX / m_max_speed), static_cast <float> (rand()) / static_cast <float> (RAND_MAX / m_max_speed));
		startvel = (2 * startvel) - olc::vf2d(m_max_speed, m_max_speed);
		m_pos = startpos;
		m_vel = startvel;


		m_heading = atan2f(m_vel.y, m_vel.x);
		m_heading_target = m_heading;

		m_max_force = 0.6;
		m_max_turnspeed = 3; //about 0.5 rev/s
		m_repulse_strength = 80; //might wanna check this, chief
		m_equil_dist = 2;
		m_cohesion_strength = m_repulse_strength / m_equil_dist;
		m_alignment_strength = 0.05;

		m_vision = (float)grid_size;

		m_gx = (int)(m_pos.x / grid_size);
		m_gy = (int)(m_pos.y / grid_size);
		//toroidal rollover
		if (m_gx < 0) { m_gx = Gx; }
		if (m_gx > Gx) { m_gx = 0; }
		if (m_gy < 0) { m_gy = Gy; }
		if (m_gy > Gy) { m_gy = 0; }

	}
	
	static olc::vf2d rotate(olc::vf2d input, float angle) {
		olc::vf2d output;
		output.x = input.x * std::cosf(angle) - input.y * std::sinf(angle);
		output.y = input.x * std::sinf(angle) + input.y * std::cosf(angle);
		return output;
	}

	static std::tuple<float, olc::vf2d> measure_distance(olc::vf2d vec1, olc::vf2d vec2) {
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
		float mag = std::sqrt(best_mag2);
		return std::make_tuple((float)mag, best_reflection);
	}
	static void gen_table(){
		//take boid_list and apply spatial partitioning
		Boid::grid_table.clear();
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

		//rotate by up to max turnspeed*fElapsedTime
		float rotate;
		if (std::abs(m_heading_target - m_heading) > m_max_turnspeed) {
			rotate = m_alignment_strength * m_max_turnspeed * fElapsedTime;
		}
		else {
			rotate = m_alignment_strength * (m_heading_target - m_heading) * fElapsedTime;
		}
		rotate = rotate *180 / M_PI;
		//float rotate = m_alignment_strength * min(m_heading_target - m_heading, m_max_turnspeed * fElapsedTime); //rotation amount
		m_vel = Boid::rotate(m_vel, rotate); //convert angle to radians before passing to rotate
		m_heading = atan2f(m_vel.y, m_vel.x);
		m_heading_target = m_heading;
		//if (m_id == 0){ std::cout << "G: (" << m_gx << ", " << m_gy << ")\n"; }
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
		//if ((m_gx == 0) || (m_gx == Gx)) {
		//	gx_range = { Gx - 1, Gx, 0, 1 }; //i guess this is inefficient if Gx == 1 but in that case spatial partitioning would be pointless anyway so its not worth accounting for
		//}
		//else if (m_gx == Gx - 1) {
		//	gx_range = { Gx - 2, Gx - 1, Gx, 0 };
		//}
		//else {
		//	for (int i = m_gx - 1; i < m_gx + 2; i++) { gx_range.push_back(i); }
		//}
		//if ((m_gy == 0)|| (m_gy == Gy)) {
		//	gy_range = { Gy - 1, Gy, 0, 1 }; //i guess this is inefficient if Gx == 1 but in that case spatial partitioning would be pointless anyway so its not worth accounting for
		//}
		//else if (m_gy == Gy - 1) {
		//	gy_range = { Gy - 2, Gy - 1, Gy, 0 };
		//}
		//else {
		//	for (int i = m_gy - rg; i < m_gy + rg + 1; i++) { gy_range.push_back(i); }
		//}

		std::pair<int, int> key;
		for (std::vector<int>::iterator i = gx_range.begin(); i != gx_range.end(); i++) {
			for (std::vector<int>::iterator j = gy_range.begin(); j != gy_range.end(); j++) {
				key = std::make_pair( *i, *j );
				res = Boid::grid_table[key]; //this will create empty vecs everywhere but maybe thats okay?
				candidate_neighbours.insert(candidate_neighbours.end(), res.begin(), res.end());
			}
		}

		//now check their ACTUAL distance and ID numbers to prune as necessary
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
		//if (m_id == 0) { std::cout << neighbours.size() << " neighbours" << std::endl; }
		/* We need to search both +/- rg as a Moore radius, remembering to clamp any rollover back to 0*/
		return neighbours;
	}

	void calculate_movement(float fElapsedTime) {
		std::vector<Boid*> neighbours = find_neighbours((int) (m_vision/grid_size) + 1);
		olc::vf2d avg_pos(0, 0);
		float avg_heading = 0;
		bool exist_neighbours = false;
		for (int i = 0; i < neighbours.size(); i++) {
			exist_neighbours = true;
			Boid* neighbour = neighbours[i];
			//draw blue lines here if you want
			//apply repulsion
			float mag;
			olc::vf2d norm;
			std::tie(mag, norm) = Boid::measure_distance((*neighbour).m_pos, m_pos);
			m_accel += m_repulse_strength  * norm / mag;
			//apply cohesion
			avg_pos += (*neighbour).m_pos;
			//align
			avg_heading += (*neighbour).m_heading;
		}
		if (exist_neighbours) {
			olc::vf2d cohesion_target = avg_pos / neighbours.size();
			float mag;
			olc::vf2d normal;
			std::tie(mag, normal) = Boid::measure_distance(m_pos, cohesion_target);
			m_accel += m_cohesion_strength * normal;
			m_heading_target = avg_heading / neighbours.size();
		}
		//m_accel = olc::vf2d(0, -10);
		//pass
	}
};


std::vector<Boid> Boid::boid_list;
std::map< std::pair<int, int>, std::vector<Boid*> > Boid::grid_table; //initialise static var

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
		//temp debug boids
		srand(0); //good for debugging, but change this to a proper RNG later
		for (int i = 0; i < num_of_boids; i++) {
			Boid agent(i);
			Boid::boid_list.push_back(agent);
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
		Boid::gen_table();
		Clear(olc::BLACK);
		//draw gridlines first
		for (int i = 0; i < Gy+1; i++) {
			DrawLine({ 0, i * grid_size }, { Nx, i * grid_size }, olc::GREY); //horiz lines
		}
		for (int i = 0; i < Gx+1; i++) {
			DrawLine({ i * grid_size, 0 }, { i * grid_size, Ny }, olc::GREY); //vert lines
		}

		for (int i = 0; i < Boid::boid_list.size(); i++) {
			Boid* candidate = &Boid::boid_list[i];
			(*candidate).calculate_movement(fElapsedTime);
			(*candidate).update(fElapsedTime);
			FillCircle((*candidate).m_pos, (*candidate).m_size, (*candidate).m_col); //draw boid
		}

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