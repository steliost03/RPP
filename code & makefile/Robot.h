//Copyright (C) 2017 Stylianos Tsiakalos

//For personal and/or educational use only. This limitation is due to the libraries used.

/////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef SCENE_ROBOT_H
#define SCENE_ROBOT_H

#include <VVRScene/scene.h>
#include <VVRScene/canvas.h>
#include <VVRScene/utils.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <GeoLib.h>
#include <cstdlib>
#include <cmath>
#include "clipper.hpp"



//ROBOT DATA
#define INITIAL_ROBOT_TYPE 0  //0->disk 1->line 2->rectangle
#define INITIAL_ROBOT_RADIUS 10 //disk only
#define INITIAL_ROBOT_LEN 20 //line and rectangle
#define INITIAL_ROBOT_WIDTH 10 //rectangle only

#define SHOW_ROBOT 1
#define SHOW_BOX 1

//assumes negative values
#define BOX_XRIGHT -375
#define BOX_XLEFT -700
#define BOX_YDOWN -500
#define BOX_YUP -200

#define MAX_ROBOT_RADIUS 200
#define MAX_ROBOT_LEN 100
#define MAX_ROBOT_WIDTH 100

#define CHANGE_INTERVAL_RADIUS 1
#define CHANGE_INTERVAL_LEN 1
#define CHANGE_INTERVAL_WIDTH 1

#define DISCRETIZATION_INTERVAL 30 //->for generalized voronoi approximation.


//////////////////////////////////////////////////////////////////////////////
#define USE_DEFAULT_DATA 1 //if 0->user must provide obstacle and boundary.///
//////////////////////////////////////////////////////////////////////////////

#define NO_OF_MAPS 30 //number of maps calculated,default value.
#define MAX_MAPS 180 //maximum number of maps allowed.

//environment boundaries
#define RECTANGULAR_BOUNDARIES 0 //if 0->the boundary becomes a convex polygon.
 //rectangular points
#define X_MAX 700
#define Y_MAX 400
 //convex polygon points:hardcoded coordinates.(clockwise)
 //large screen mode
#define ENV_CENTER_X 0
#define ENV_CENTER_Y 0
#define B_X_1 650
#define B_Y_1 400
#define B_X_2 700
#define B_Y_2 150
#define B_X_3 650
#define B_Y_3 -300
#define B_X_4 400
#define B_Y_4 -350
#define B_X_5 -350
#define B_Y_5 -300
#define B_X_6 -400
#define B_Y_6  0
#define B_X_7 -350
#define B_Y_7 400
#define B_X_8 150
#define B_Y_8 450
//small screen mode
#define B_X_1_S 450
#define B_Y_1_S 300

#define B_X_2_S 500
#define B_Y_2_S 150

#define B_X_3_S 450
#define B_Y_3_S -300

#define B_X_4_S 200
#define B_Y_4_S -350

#define B_X_5_S -150
#define B_Y_5_S -300

#define B_X_6_S -200
#define B_Y_6_S  0

#define B_X_7_S -150
#define B_Y_7_S 300

#define B_X_8_S 150
#define B_Y_8_S 350



//random center random obstacle max coordinates
#define RAND_OBST_XMAX 400
#define RAND_OBST_YMAX 250

//hardcoded open obstacles
#define X_HARDCODED_SIDE_1 500
#define Y_HARDCODED_SIDE_1 250
#define X_HARDCODED_SIDE_2 550
#define Y_HARDCODED_SIDE_2 300
#define Y_HARDCODED_SIDE_3 150

//hardcoded line obstacles
#define LINE_X_1 -320
#define LINE_Y_1 -250
#define LINE_X_2 -300
#define LINE_Y_2 -150

#define LINE2_X_1 -300
#define LINE2_X_2 -100
#define LINE2_Y 350

//hardcoded convex obstacles
#define TRIANGLE_X1 550
#define TRIANGLE_Y1 -100
#define TRIANGLE_X2 470
#define TRIANGLE_Y2 -250
#define TRIANGLE_X3 600
#define TRIANGLE_Y3 -250

#define POLY_X1 167
#define POLY_X2 129
#define POLY_X3 155
#define POLY_X4 198
#define POLY_X5 211
#define POLY_Y1 229
#define POLY_Y2 270
#define POLY_Y3 314
#define POLY_Y4 303
#define POLY_Y5 266

#define REC_X1 250
#define REC_Y1 320
#define REC_X2 350
#define REC_Y2 320
#define REC_X3 250
#define REC_Y3 280
#define REC_X4 350
#define REC_Y4 280


C2DPoint environment_center;
short globalcounter = 0;
int adjust = 0;	//is used for adjustment of object position and sizes for small screen mode.
int voronoi_discretization_interval = DISCRETIZATION_INTERVAL;
int no_of_maps_calculated = NO_OF_MAPS;
C2DPolygon perm_poly; //variable which is used for the random obstacle.
bool starting_message = false;
bool show_diagrams = false;
bool rectangular_boundary = RECTANGULAR_BOUNDARIES;
bool robot_visible = SHOW_ROBOT;
double box_center_x = (BOX_XRIGHT + BOX_XLEFT) / 2.0;
double box_center_y = (BOX_YUP + BOX_YDOWN) / 2.0;
bool legend_shown = false;
bool rand_ob_enabled = true;

bool minkowski_and_voronoi_calculated = false;
bool path_planning_mode = false;
bool first_point_placed = false;
bool second_point_placed = false;
bool rotational_path_planning_mode = false;
bool rotational_points_placed = false;
double starting_angle, ending_angle; //input from the user in rotational path planning mode.

bool only_show_minkw = false;
bool adjust_for_small_screens = false;
bool read_obs_from_file = false;
bool obs_read = false;
std::string filename;
char ob_type, mode;
bool env_boundary_infile = false;
bool custom_modification = false;
bool emerg_termination = false;
bool custom_boundary = false;


enum type{ disk, line, rectangle };
struct robot {

	type robot_type = disk;
	std::string type_text = "disk";
	double radius; //disk only
	double len; //line and rectangle
	double width; //rectangle only
	C2DPoint center_pos; //->for displaying purposes in lower left box, and for use in rotational path planning mode.

	C2DVector orientation = C2DVector(1, 0);
};


typedef struct graph_node graph_node; //graph_node syntactically equal to 'struct graph_node'.

//used for representation of graphs. Graphs are used for the calculation of the generalized voronoi diagram
//(and for the safest path).
struct graph_node{

	std::vector<graph_node*> neighbors; 
	int connectivity = 0;
	C2DPoint vertex;
	bool visited = false;

};

void ShowMenu();

void find_lines_intersection(C2DLine, C2DLine, C2DPoint&);
bool line_intersects_with_polygon(C2DLine,C2DPolygon);

void lines_to_polygon(std::vector<C2DLine>, C2DPolygon&);
void polygons_to_lines(std::vector<C2DPolygon>&,std::vector<C2DLine>&);
void check_if_already_added(C2DPoint,std::vector<graph_node>&);
void check_and_add_neighbors(C2DLine,std::vector<graph_node>&);
bool points_are_identical(C2DPoint, C2DPoint, double);
bool polygons_are_identical(C2DPolygon, C2DPolygon);
bool line_intersects_with_polygon(C2DLine, C2DPolygon);
bool lines_intersect(C2DLine, C2DLine);


bool point_is_on_line(C2DPoint, C2DLine);


//
double rectangle_robot_find_offset_distance(C2DLine, double, C2DVector,bool);
double pythagorean_theorem(double, double);
double degrees_to_radians(double);
double dotproduct(C2DVector,C2DVector);





class SceneRobotEnvironment : public vvr::Scene
{
public:
	SceneRobotEnvironment();


	const char* getName() const override {
		return "Robot";
	}

protected: // Overriden methods
	void draw() override;
	void reset() override;
	void mousePressed(int x, int y, int modif) override;
	void mouseMoved(int x, int y, int modif) override;
	void keyEvent(unsigned char key, bool up, int modif) override;
	void sliderChanged(int slider_id,float val) override;


private: // Methods

	void ReadObstaclesFromFile(std::string, char,char,bool);
	void DrawObstacles_CustomsIncluded(bool,bool);
	//called by ReadObstaclesFromFile
	void adjust_to_env_center(std::vector<C2DPoint>& obstacle);


	void CreateObstacles(bool init,int adjust = 0);
	void CreateObstacles_TextFile();

	void CalculateMinkowski(bool find_free_space = true); //Calculates and draws minkowski modified obstacles.
	//called by main CalculateMinkowski function.
	void CalculateMinkowski_convex_obstacle(C2DPolygon obstacle);
	void CalculateMinkowski_nonconvex_obstacle();
	void CalculateMinkowski_line_obstacle(C2DLine obstacle);
	void CalculateMinkowski_boundary_disk(C2DLine boundary_line,C2DPointSet& boundary_points);
	void CalculateMinkowski_boundary_general(C2DPolygon boundary);
	void Minkowski_draw();

	void compare_directions(C2DLine,bool&); //compares orientation of line ob with line robot.
	//

	void Minkowski_find_free_space();

	void ShowRobot(int adjust = 0);
	void modifyRobot(type new_robot_type = type(INITIAL_ROBOT_TYPE), double new_radius = INITIAL_ROBOT_RADIUS,
		double new_len = INITIAL_ROBOT_LEN, double new_width = INITIAL_ROBOT_WIDTH,C2DPoint new_center_pos = C2DPoint(box_center_x, box_center_y),
		bool output_msg = false,std::string dimchanged = "");

	void ChangeRobotOrientation(char = 'd',double set_angle = -999);

	void Generalized_Voronoi(); //calculates an approximation of the generalized voronoi diagram.
	void Discretize_Space(std::vector<C2DPoint>&);
	void Discretize_Space2(std::vector<C2DPoint>&);
	void Discretize_Line(std::vector<C2DPoint>&, C2DLine,C2DPointSet&,int depth=0);
	void Voronoi(unsigned,std::vector<C2DPoint>&,std::vector<C2DPolygon>&);
	void Voronoi_find_planes_intersection(std::vector<C2DPolygon>&,std::vector<C2DPolygon>&);
	void Voronoi_delete_unwanted_lines(std::vector<C2DLine>&);
	void Voronoi_draw();

	void find_safest_path(C2DPoint,C2DPoint,bool&);
	//called by find_safest_path:
	bool valid_point_selected(C2DPoint);
	bool line_is_valid(C2DLine);
	bool line_is_valid(C2DPoint, C2DPoint);
	bool neighbor_path_leads_to_target(graph_node*,std::vector<graph_node>&,graph_node*,int);
	int index_of_corresponding_distance(graph_node*,double,graph_node*);
	//
	void find_optimal_path(C2DPoint, C2DPoint);
	//called by find_optimal_path:
	void find_optimal_recursive(int depth, C2DPoint, int, std::vector<std::vector<C2DLine>>&, std::string, int direction_id = 0, int obs_index = 0, int obs_line = 0);
	void grow_line(C2DPoint,C2DLine&,bool,bool);
	bool no_obstacles_hit(C2DLine,C2DPoint* intersection_point,int* obs_index,int* obs_line_index);
	void manage_obstacle_lines();
	bool doesnt_hit_current_ob(C2DLine, C2DPolygon);
	bool line_is_in_freespace(C2DLine);
	int no_of_obs_crossed(C2DLine);
	bool line_crosses_real_ob(C2DLine);
	bool line_outside_bounds(C2DLine);
	bool obstacle_encountered(int);

	void SortByAngle3(C2DPointSet*);


	//
	void vector_to_pointset(std::vector<C2DPoint>, C2DPointSet*);
	void polygon_to_graph();
	void delete_graph_leaves();
	void convex_hull(std::vector<C2DPoint>, std::vector<C2DPoint>*);

	void minkw_addition(std::vector<C2DPoint>&, std::vector<C2DPoint>&, std::vector<C2DPoint>*);
	void add_robot_points(std::string, robot, std::vector<C2DPoint>*);
	void add_vertices(C2DPolygon, std::vector<C2DPoint>*);
	void add_vertices_line_ob(C2DLine, std::vector<C2DPoint>*);

	void rotational_pp(C2DPoint,C2DPoint,double,double);
	bool valid_point_selected_rotational(C2DPoint);
	void ShowRobot_in_env(C2DPoint, bool error_display, bool small_display = false,bool draw_center = false,bool colored = false);
	void compute_discrete_minkw();
	void minkowski_union();
	void draw_total_minkw();
	int find_path_orientations(std::vector<C2DLine>);
	void approx_minkw_for_lineobs(C2DLine line_ob);
	//called by find_path_orientations
	void check_point_orientation(C2DPoint,C2DVector&,bool&);
	void find_smallest_angle_change(C2DVector, std::vector<C2DVector>, C2DVector&);
	bool is_inside_space(C2DPoint, std::vector<C2DPolygon>, std::vector<C2DPolygon>, C2DLine& intersectingObline, C2DVector v = C2DVector(), bool check_dir = false);
	//called by is_inside_space
	void find_intersecting_line(C2DPolygon,C2DLine,C2DLine&);
	
	




private: // Data
	vvr::Canvas2D m_canvas;
	vvr::Canvas2D m_canvas_pp; //for path planning points and lines.
	vvr::Canvas2D m_canvas_robot; //for visualizing robot inside environment.
	robot m_robot;

	//data types in which the environment obstacles are held.
	std::vector<C2DPolygon> polygon_obstacles;
	std::vector<C2DLine> line_obstacles;
	std::vector<C2DPolygon> original_polygon_obstacle; //useful in the context of rotational path planning

	//environment boundary lines
	std::vector<C2DLine> boundary_lines; //for rectangular environment
	C2DPolygon boundary; //for convex polygon environment , or custom environment.
	//


	//modified obstacles after the calculation of minkowski sums
	std::vector<C2DPoint> minkw_points;
	std::vector<C2DPolygon> modified_polygon_obstacles;
	std::vector<C2DPolygon> modified_polygon_obstacles_union;
	C2DPolygon modified_boundary;
	std::vector<C2DPolygon> free_space;

	std::vector<C2DPoint> robot_points;
	std::vector<C2DPoint> line_ob_points;
	std::vector<C2DPoint> current_obstacle_edges;


	//voronoi graph representation
	std::vector<C2DLine> cell_lines;
	std::vector<graph_node> graph;
	C2DPointSet freespacepoints; //a separate data member to make the drawing of the discrete space easier.  (identical with discretized_space conceptually)

	//path planning Data
	C2DPoint point_1;
	C2DPoint point_2;
	std::vector<C2DLine> shortest_safest_path;
	//optimal path
	std::vector<C2DLine> optimal_path;
	std::vector<int> obs_encountered;

	//rotational path planning
	std::vector<C2DPolygon> current_map_obs_union;

	std::vector<C2DPolygon> minkw_resulting_obs; //***the resulting obstacles if we find the union of each map,then the intersection of the map unions.***
	std::vector<C2DPolygon> minkw_union_freespace;
	//test
	std::vector<C2DPolygon> minkw_resulting_bounds;

	C2DVector map_directions[MAX_MAPS];
	std::vector<C2DPolygon> maps_obstacles[MAX_MAPS];
	std::vector<C2DPolygon> maps_free_space[MAX_MAPS];
	std::vector<C2DPolygon> maps_obs_unions[MAX_MAPS];
	C2DPolygon maps_boundaries[MAX_MAPS];



	short obs_enc = 0;



};

#endif // SCENE_ROBOT_H
