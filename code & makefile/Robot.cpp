//Copyright (C) 2017 Stylianos Tsiakalos

//For personal and/or educational use only. This limitation is due to the libraries used.

/////////////////////////////////////////////////////////////////////////////////////////////////

#include "clipper.hpp"
#include "Robot.h"


using namespace std;
using namespace vvr;
using namespace ClipperLib;

//functions that converts data types from geolib to clipper and vice versa.
void geolib_2_clipper(C2DPolygon, Path&);
void geolib_2_clipper(C2DPoint, IntPoint&);
void clipper_2_geolib(Path, C2DPolygon&);
void clipper_2_geolib(IntPoint, C2DPoint&);



//CONVERSION FUNCTIONS//

//polygon overloads//
void geolib_2_clipper(C2DPolygon geolib_polygon, Path& clipper_polygon) {

	std::vector<C2DPoint> geolib_polygon_points;
	for (int w = 0; w < geolib_polygon.GetPointsCount(); w++) {

		geolib_polygon_points.push_back(*geolib_polygon.GetPoint(w));
	}

	for (int w = 0; w < geolib_polygon_points.size(); w++) {
		IntPoint temp;
		geolib_2_clipper(geolib_polygon_points[w], temp);
		clipper_polygon.push_back(temp);
	}


}

void clipper_2_geolib(Path clipper_polygon, C2DPolygon& geolib_polygon) {

	C2DPointSet geolib_polygon_points;

	for (int w = 0; w < clipper_polygon.size(); w++) {

		C2DPoint point_to_add;
		clipper_2_geolib(clipper_polygon[w], point_to_add);
		geolib_polygon_points.AddCopy(point_to_add);
	}

	geolib_polygon = C2DPolygon(geolib_polygon_points);


}

//point overloads//
void geolib_2_clipper(C2DPoint geolib_point,IntPoint &clipper_point) {

	//we scale the coordinate values because the clipper library accepts only integer type coordinates.
	clipper_point.X = 10*geolib_point.x;
	clipper_point.Y = 10*geolib_point.y;


}

void clipper_2_geolib(IntPoint clipper_point, C2DPoint& geolib_point) {

	//scale back coordinates to original values.
	double x_coordinate = clipper_point.X / 10.0;
	double y_coordinate = clipper_point.Y / 10.0;
	geolib_point = C2DPoint(x_coordinate, y_coordinate);
}


////////////////

void ShowMenu() {

	cout << "ROBOT PATH PLANNING SIMULATOR" << endl;
	cout << "***" << endl;
	cout << "Press 'h' for control instructions" << endl << endl;
	cout << "Press 'v' to adjust display for small screens (smaller environment)" << endl << endl;
	cout << "Press 'f' to show/hide Generalized Voronoi diagram and Minkowski sums" << endl << endl;
	cout << "Press 'm' to only show/hide Minkowski sums(toggle on/off)" << endl << endl;
	cout << "Press 'p' to show best path between two points (chosen by mouse click) (must have voronoi and minkowski calculated)" << endl << endl;
	cout << "Press 'e' to show best path between two points,with rotation included (use console)" << endl << endl;
	cout << "Press 's' to switch between visible/hidden robot" << endl << endl;
	cout << "Press '9' to enter obstacle/environment data from file (use console)"<<endl;
	cout << endl << "Press 'x'to display the starting menu at any time" << endl<<endl;
	cout << "Press 'c' to clear the canvas (leaves initial obstacles and environment)" << endl<<endl;
	cout<<"Press '6' to show slider info"<<endl;
	cout<< "---------------";
}



//finds the intersection point between two given lines (the determinant formula is used).
//this function assumes that an intersection point exists.
void find_lines_intersection(C2DLine line1, C2DLine line2,C2DPoint& intersection_point){

	double x1, y1, x2, y2, x3, y3, x4, y4;

	x1 = line1.GetPointFrom().x;
	y1 = line1.GetPointFrom().y;
	x2 = line1.GetPointTo().x;
	y2 = line1.GetPointTo().y;

	x3 = line2.GetPointFrom().x;
	y3 = line2.GetPointFrom().y;
	x4 = line2.GetPointTo().x;
	y4 = line2.GetPointTo().y;

	intersection_point.x = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4));
	intersection_point.y = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4));




}

//checks if two given lines have an intersection point.
//step 1: finds hypothetical intersection point coordinates using the determinant formula.
//step 2: finds extreme points of both lines.
//step 3: if intersection point falls between the extreme points of both lines,then the lines intersect at that point.
bool lines_intersect(C2DLine line1, C2DLine line2){

	double x1, y1, x2, y2, x3, y3, x4, y4;

	x1 = line1.GetPointFrom().x;
	y1 = line1.GetPointFrom().y;
	x2 = line1.GetPointTo().x;
	y2 = line1.GetPointTo().y;

	x3 = line2.GetPointFrom().x;
	y3 = line2.GetPointFrom().y;
	x4 = line2.GetPointTo().x;
	y4 = line2.GetPointTo().y;

	double intx, inty;
	intx = ((x1*y2 - y1*x2)*(x3 - x4) - (x1 - x2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4));
	inty = ((x1*y2 - y1*x2)*(y3 - y4) - (y1 - y2)*(x3*y4 - y3*x4)) / ((x1 - x2)*(y3 - y4) - (y1 - y2)*(x3 - x4));

	double xmax_l1, xmax_l2, xmin_l1, xmin_l2;
	double ymax_l1, ymax_l2, ymin_l1, ymin_l2;

	if (line1.GetPointFrom().x > line1.GetPointTo().x){
		xmax_l1 = line1.GetPointFrom().x;
		xmin_l1 = line1.GetPointTo().x;
	}
	else{
		xmax_l1 = line1.GetPointTo().x;
		xmin_l1 = line1.GetPointFrom().x;
	}

	if (line1.GetPointFrom().y > line1.GetPointTo().y){
		ymax_l1 = line1.GetPointFrom().y;
		ymin_l1 = line1.GetPointTo().y;
	}
	else{
		ymax_l1 = line1.GetPointTo().y;
		ymin_l1 = line1.GetPointFrom().y;
	}

	if (line2.GetPointFrom().x > line2.GetPointTo().x){
		xmax_l2 = line2.GetPointFrom().x;
		xmin_l2 = line2.GetPointTo().x;
	}
	else{
		xmax_l2 = line2.GetPointTo().x;
		xmin_l2 = line2.GetPointFrom().x;
	}

	if (line2.GetPointFrom().y > line2.GetPointTo().y){
		ymax_l2 = line2.GetPointFrom().y;
		ymin_l2 = line2.GetPointTo().y;
	}
	else{
		ymax_l2 = line2.GetPointTo().y;
		ymin_l2 = line2.GetPointFrom().y;
	}


	if (xmin_l1 <= intx && intx <= xmax_l1 && ymin_l1 <= inty && inty <= ymax_l1 && xmin_l2 <= intx && intx <= xmax_l2 && ymin_l2 <= inty && inty <= ymax_l2)
		return true;
	else
		return false;



}


//conversion from degrees to radians.
double degrees_to_radians(double degrees){

	if (degrees >= 180){
		degrees = 180 - degrees;
	}

	double radians;

	radians = (degrees*3.14) / 180.0;

	return radians;


}


//checks if a given line intersects with a given polygon.
//if atleast one line of the polygon intersects with the given line,then 'true' is returned.
bool line_intersects_with_polygon(C2DLine l,C2DPolygon p){

	std::vector<C2DLine> polygon_lines;
	std::vector<C2DPolygon> polygon;
	bool intersection_exists = false;
	polygon.push_back(p);
	polygons_to_lines(polygon,polygon_lines);

	for(int w=0;w<polygon_lines.size();w++){
		if (lines_intersect(l, polygon_lines[w]))
			intersection_exists = true;

	}

	return intersection_exists;
}


//checks if a given point is part of a given line.
bool point_is_on_line(C2DPoint point, C2DLine line){

	C2DPoint p1 = line.GetPointFrom();
	C2DPoint p2 = line.GetPointTo();
	//x1,x2,y1,y2 : line extreme point coordinates
	double y1 = p1.y;
	double x1 = p1.x;
	double y2 = p2.y;
	double x2 = p2.x;

	//x,y : given point coordinates
	double x = point.x;
	double y = point.y;

	double ymin, ymax;
	double xmin, xmax;
	if (x2 >= x1){
		xmax = x2;
		xmin = x1;
	}
	else{
		xmax = x1;
		xmin = x2;
	}

	//if line is vertical(special case), using tolerance of 1:
	if (abs(xmax - xmin) < 1){
		if (y2 >= y1){
			ymax = y2;
			ymin = y1;
		}
		else{
			ymax = y1;
			ymin = y2;
		}
		if (abs(x - xmax) < 1 && y >= ymin && y <= ymax)
			return true;
		else
			return false;
	}

	//general case:
	if (x >= xmin && x <= xmax){
		
		//evaluate line equation with point coordinates. 
		//If the result is a value smaller than 1(tolerance) ,
		//then the point is considered to be part of the line.
		double line_equation = (y - y1) - ((y2 - y1) / (x2 - x1))*(x - x1); 


		if (abs(line_equation) < 1)
			return true;
		else
			return false;
	}
	else
		return false;
}


//converts a given polygon to an std::vector of lines. This is used for the representation of the graph used
//for the safest path calculation.
void polygons_to_lines(std::vector<C2DPolygon>&cells, std::vector<C2DLine>& cell_lines) {

	for (int x = 0; x < cells.size(); x++){

		for (int y = 0; y < cells[x].GetLineCount(); y++){

			cell_lines.push_back(*cells[x].GetLine(y));
		}
	}
}


//std::vector of lines -> C2DPolygon representation
void lines_to_polygon(std::vector<C2DLine> lines, C2DPolygon& polygon){

	C2DPointSet polygon_points;

	for (int w = 0; w < lines.size(); w++){

		polygon_points.AddCopy(lines[w].GetPointFrom());
	}

	polygon = C2DPolygon(polygon_points, true);
}



//checks if two points have the same coordinates (within a specified tolerance value).
bool points_are_identical(C2DPoint point1, C2DPoint point2, double tolerance){

	if (abs(point1.x - point2.x) < tolerance && abs(point1.y - point2.y) < tolerance)
		return true;

	return false;



}

//checks if two given polygons are identical (points_are_identical function is utilized).
bool polygons_are_identical(C2DPolygon poly1, C2DPolygon poly2){

	bool polygonsareidentical = true;

	for (int w = 0; w < poly1.GetPointsCount(); w++){

		if (!points_are_identical(*poly1.GetPoint(w), *poly2.GetPoint(w),1))
			polygonsareidentical = false;



	}
	return polygonsareidentical;
}


//calculates dot product of two vectors via the algebraic definition.
double dotproduct(C2DVector v1,C2DVector v2){

	double dotpr;
	dotpr = v1.i*v2.i + v1.j*v2.j;
	return dotpr;
}


//sorts a pointset by their angle.
//used to convert a C2DPointSet to a C2DPolygon in the minkowski calculations.
//(pointset needs to be sorted correctly so that the resulting C2DPolygon is not bugged).
void SceneRobotEnvironment::SortByAngle3(C2DPointSet* pointset){

	//first find centroid
	double centroid_x, centroid_y;
	double sum_x = 0;
	double sum_y = 0;
	for (int w = 0; w < pointset->size(); w++){
		sum_x = sum_x + pointset->GetAt(w)->x;
		sum_y = sum_y + pointset->GetAt(w)->y;
	}
	centroid_x = sum_x / pointset->size();
	centroid_y = sum_y / pointset->size();

	C2DPoint centroid(centroid_x, centroid_y);

	//then sort all points with respect to that centroid.
	pointset->SortByAngleFromNorth(centroid);


}


//converts a polygon to graph representation.
void SceneRobotEnvironment::polygon_to_graph(){

	//graph is a global variable.Its purpose is to represent the
	//current calculated graph.
	graph.clear();

	//step 1: add nodes to graph.
	for(int x=0;x<cell_lines.size();x++){

		check_if_already_added(cell_lines[x].GetPointFrom(),graph);
		check_if_already_added(cell_lines[x].GetPointTo(),graph);


		}

	//part 2: add neighbors to every node
	for(int x=0;x<cell_lines.size();x++){

		check_and_add_neighbors(cell_lines[x],graph);
	}

}


//'trims' the graph. A large amount of leaves may appear because the
// generalized voronoi diagram is calculated in an approximate manner.
void SceneRobotEnvironment::delete_graph_leaves(){

	for(int w=0;w<graph.size();w++){
		if(graph[w].connectivity <=1){
			C2DLine cell_line_to_delete = C2DLine(graph[w].vertex,graph[w].neighbors[0]->vertex);
			for(int w=0;w<cell_lines.size();w++){
				if( (points_are_identical(cell_lines[w].GetPointFrom(),cell_line_to_delete.GetPointFrom(),1) ||
				points_are_identical(cell_lines[w].GetPointFrom(),cell_line_to_delete.GetPointTo(),1))&&
				(points_are_identical(cell_lines[w].GetPointTo(),cell_line_to_delete.GetPointTo(),1) ||
				points_are_identical(cell_lines[w].GetPointTo(),cell_line_to_delete.GetPointFrom(),1))){
					cell_lines.erase(cell_lines.begin()+w);
					break;
				}
			}
		}
	}
}



//adjust given obstacle coordinates to the center of the environment.
//this function is used for obstacles that are read from user input.
void SceneRobotEnvironment::adjust_to_env_center(std::vector<C2DPoint>& obstacle){

	for (int w = 0; w < obstacle.size(); w++){

		obstacle[w].x = obstacle[w].x - environment_center.x;
		obstacle[w].y = obstacle[w].y - environment_center.y;
	}


}


//group of points : std::vector -> C2DPointSet
void SceneRobotEnvironment::vector_to_pointset(std::vector<C2DPoint> vector, C2DPointSet* pointset){

	for (int w = 0; w < vector.size(); w++){

		pointset->AddCopy(vector[w]);
	}


}

//checks if a point has already been added as a node in the graph.
void check_if_already_added(C2DPoint target_point,std::vector<graph_node>& graph){

		bool already_added = false;
		for(int x=0;x<graph.size();x++){
			if (points_are_identical(target_point,graph[x].vertex,2)){
				already_added = true;

			}

		}
		if(already_added==false){
			graph_node new_node;
			new_node.vertex = target_point;
			graph.push_back(new_node);
		}


}


//adds the two extreme points of a given line as neighbors to each other (used in graph representation).
void check_and_add_neighbors(C2DLine target_line,std::vector<graph_node>& graph){

	C2DPoint pointfrom = target_line.GetPointFrom();
	C2DPoint pointto = target_line.GetPointTo();

	int pointfrom_index,pointto_index;
	//first find the indices of the points
	for(int x=0;x<graph.size();x++){

		if(points_are_identical(target_line.GetPointFrom(),graph[x].vertex,2))
			pointfrom_index = x;
		if(points_are_identical(target_line.GetPointTo(),graph[x].vertex,2))
			pointto_index = x;

	}

	//then add each other as neighbor and increase their connectivity

	graph[pointfrom_index].neighbors.push_back(&graph[pointto_index]);
	graph[pointto_index].neighbors.push_back(&graph[pointfrom_index]);
	graph[pointfrom_index].connectivity += 1;
	graph[pointto_index].connectivity +=1;
}


//discretization of robot for the purpose of calculating minkowski sums.
//the points chosen are different, depending on the type of the robot.
void SceneRobotEnvironment::add_robot_points(std::string robot_type, robot robot,std::vector<C2DPoint>* points){


	if (robot_type == "line"){

		//discretizing the line robot into 3 points.top,bottom and center.
		//center is moved to (0,0)
		C2DPoint line_center(0, 0);

		//find top and bottom based on the orientation.
		//top
		C2DLine line_to_top(line_center, robot.orientation);
		line_to_top.SetLength(robot.len / 2.0);
		points->push_back(line_to_top.GetPointTo());

		//bottom
		C2DVector reverse_orientation(-robot.orientation.i, -robot.orientation.j);
		C2DLine line_to_bottom(line_center, reverse_orientation);
		line_to_bottom.SetLength(robot.len / 2.0);
		points->push_back(line_to_bottom.GetPointTo());


	}

	else if (robot_type == "rectangle"){

		C2DVector orientation = robot.orientation; //this will be done each time we want to reset the orientation to the original one.

		//discretizing the rectangle robot into the corner points and the center point.

		//center is moved to (0,0)
		C2DPoint rectangle_center(0, 0);
		points->push_back(rectangle_center);

		//top midpoint(so that the upper corner points can be reached)
		C2DLine line_to_top(rectangle_center, orientation);
		line_to_top.SetLength(robot.len / 2.0);
		C2DPoint upper_midpoint = line_to_top.GetPointTo();

		//upper right corner point
		orientation.TurnRight();
		C2DLine line_to_right_corner(upper_midpoint, orientation);
		line_to_right_corner.SetLength(robot.width / 2.0);
		C2DPoint right_corner = line_to_right_corner.GetPointTo();
		points->push_back(right_corner);

		//upper left corner point
		orientation = robot.orientation;
		orientation.TurnLeft();
		C2DLine line_to_left_corner(upper_midpoint, orientation);
		line_to_left_corner.SetLength(robot.width / 2.0);
		C2DPoint left_corner = line_to_left_corner.GetPointTo();
		points->push_back(left_corner);

		//bottom midpoint(so that the lower corner points can be reached)
		orientation = robot.orientation;
		C2DVector opposite_orientation(-orientation.i, -orientation.j);
		C2DLine line_to_bottom(rectangle_center, opposite_orientation);
		line_to_bottom.SetLength(robot.len / 2.0);
		C2DPoint bottom_midpoint = line_to_bottom.GetPointTo();

		//lower right corner point
		orientation = robot.orientation;
		orientation.TurnRight();
		C2DLine line_to_lowerright_corner(bottom_midpoint, orientation);
		line_to_lowerright_corner.SetLength(robot.width / 2.0);
		C2DPoint lower_right_corner = line_to_lowerright_corner.GetPointTo();
		points->push_back(lower_right_corner);

		//lower left corner point
		orientation = robot.orientation;
		orientation.TurnLeft();
		C2DLine line_to_lowerleft_corner(bottom_midpoint, orientation);
		line_to_lowerleft_corner.SetLength(robot.width / 2.0);
		C2DPoint lower_left_corner = line_to_lowerleft_corner.GetPointTo();
		points->push_back(lower_left_corner);



	}


}


//finds offset distance for the boundary, for a chosen dimension of the robot.
double rectangle_robot_find_offset_distance(C2DLine boundaryline, double dimension_value, C2DVector orientation) {

	C2DPoint midpoint = boundaryline.GetMidPoint();
	C2DVector boundaryline_normal(boundaryline.GetPointFrom(), boundaryline.GetPointTo());
	boundaryline_normal.TurnRight();
	C2DLine boundaryline_norm_l(midpoint, boundaryline_normal);
	boundaryline_norm_l.GrowFromCentre(5000);

	C2DLine line_to_offsetlimitpoint(midpoint, orientation);
	line_to_offsetlimitpoint.SetLength(dimension_value);

	C2DPoint offsetlimitpoint_linestart = line_to_offsetlimitpoint.GetPointTo();
	boundaryline_normal.TurnLeft();
	C2DLine offset_limit_line(offsetlimitpoint_linestart, boundaryline_normal);
	offset_limit_line.GrowFromCentre(5000);

	C2DPoint offset_limit_point;
	find_lines_intersection(offset_limit_line, boundaryline_norm_l, offset_limit_point);

	boundaryline_normal.TurnRight();
	C2DLine offset_distance_line(midpoint, offset_limit_point);
	double result = offset_distance_line.GetLength();

	return result;


}

//
double pythagorean_theorem(double a, double b){

	double result;
	double asquared = a*a;
	double bsquared = b*b;

	result = asquared + bsquared;
	result = sqrt(result);
	return result;
}

//returns all vertices of polygon 
void SceneRobotEnvironment::add_vertices(C2DPolygon polygon, std::vector<C2DPoint>* vertices){

	vertices->clear();
	for (int w = 0; w < polygon.GetPointsCount(); w++){

		vertices->push_back(*polygon.GetPoint(w));
	}
}

//returns extreme points and midpoint of line
void SceneRobotEnvironment::add_vertices_line_ob(C2DLine line, std::vector<C2DPoint>* points){
	points->push_back(line.GetPointFrom());
	points->push_back(line.GetPointTo());
	points->push_back(line.GetMidPoint());
}

//Performs minkowski addition on two given set of points.
void SceneRobotEnvironment::minkw_addition(std::vector<C2DPoint>& set1, std::vector<C2DPoint>& set2, std::vector<C2DPoint>* resulting_set) {

	int i, j;

	for (i = 0; i < set1.size(); i++){

		for (j = 0; j < set2.size(); j++){

			C2DPoint resulting_point;
			resulting_point.x = set1[i].x + set2[j].x;
			resulting_point.y = set1[i].y + set2[j].y;
			resulting_set->push_back(resulting_point);
		}
	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Construct - Load  - Setup */

SceneRobotEnvironment::SceneRobotEnvironment()
{
	m_bg_col = Colour(0x44, 0x44, 0x44);
	m_hide_log = false;
	m_hide_sliders = false;
	m_fullscreen = true;
	srand(time(0));


	modifyRobot();


	reset();
}

void SceneRobotEnvironment::reset()
{
	Scene::reset();
}


/* UI Handling */
void SceneRobotEnvironment::mousePressed(int x, int y, int modif)
{
	Scene::mousePressed(x, y, modif);
	if (!path_planning_mode && !rotational_path_planning_mode){
		std::cout << "x = " << x;
		std::cout << "---y = " << y << endl;
	}
	const C2DPoint p(x, y);
	C2DPoint p0 = p;


	if (minkowski_and_voronoi_calculated || only_show_minkw){

		for (int w = 0; w < modified_polygon_obstacles.size(); w++){
			if (modified_polygon_obstacles[w].Contains(p)){
				cout << "Inside obstacle with index " << w << endl;
				break;
			}
		}
		for (int w = 0; w < modified_polygon_obstacles_union.size(); w++){
			if (modified_polygon_obstacles_union[w].Contains(p)){
				cout << "UNION : Inside obstacle with index " << w << endl;
				break;
			}
		}
		for (int w = 0; w < minkw_resulting_obs.size(); w++){
			if (minkw_resulting_obs[w].Contains(p)){
				cout << "minkw resulting : Inside obstacle with index " << w << endl;
			}
		}

	}


	if (path_planning_mode &&minkowski_and_voronoi_calculated){
		if (!valid_point_selected(p))
			cout << endl << "Invalid point,please try again (point must be inside free space)" << endl;
		else{

			if (!first_point_placed) {
				m_canvas_pp.clear();
				first_point_placed = true;
				cout << endl << "First point placed" << endl;
				point_1 = p0;
				m_canvas_pp.add(point_1);
				return;
			}
			if (first_point_placed && !second_point_placed) {
				second_point_placed = true;
				cout << "Second point placed" << endl;
				point_2 = p0;
				m_canvas_pp.add(point_2);
			}
			if (second_point_placed){
				first_point_placed = false;
				second_point_placed = false;
				path_planning_mode = false;
				bool path_is_impossible;
				find_safest_path(point_1, point_2, path_is_impossible);
				if (!path_is_impossible)
					find_optimal_path(point_1, point_2);
				point_1 = C2DPoint();
				point_2 = C2DPoint();
				if (!path_is_impossible){
					cout << endl << "PURPLE -> Shortest Safe Path" << endl;
					cout << "CYAN -> Optimal Path" << endl;
					cout << endl << "Press p again to create a new path." << endl;
					path_planning_mode = false;
				}

			}
		}
	}

	if (rotational_path_planning_mode){

		if (!valid_point_selected_rotational(p))
			cout << endl << "Invalid point,please try again (point must be inside free space)" << endl;
		else{

			if (!first_point_placed) {
				m_canvas_pp.clear();
				m_canvas_robot.clear();
				m_canvas.clear();
				first_point_placed = true;
				cout << endl << "First point placed" << endl;
				point_1 = p0;
				m_canvas_pp.add(point_1);
				return;
			}
			if (first_point_placed && !second_point_placed) {
				second_point_placed = true;
				rotational_points_placed = true;
				cout << "Second point placed" << endl;
				point_2 = p0;
				m_canvas_pp.add(point_2);
			}
			if (second_point_placed){
				first_point_placed = false;
				second_point_placed = false;
				rotational_pp(point_1, point_2, starting_angle, ending_angle);
				point_1 = C2DPoint();
				point_2 = C2DPoint();

			}




		}
	}
}


void SceneRobotEnvironment::mouseMoved(int x, int y, int modif)
{
	Scene::mouseMoved(x, y, modif);
	mousePressed(x, y, modif);
}

void SceneRobotEnvironment::sliderChanged(int slider_id,float val){

	if(slider_id == 0){
		voronoi_discretization_interval = 10 +90 * val;
		cout<<"Voronoi Discr.interval changed to "<<voronoi_discretization_interval<<endl;
	}
	if(slider_id == 1){
		no_of_maps_calculated = 10 + (MAX_MAPS-10)*val;
		cout<<"map calculation number changed to "<<no_of_maps_calculated<<endl;
	}
}
void SceneRobotEnvironment::keyEvent(unsigned char key, bool up, int modif) {

	Scene::keyEvent(key, up, modif);
	key = tolower(key);

	if (key == 'x')
		ShowMenu();

	if (key == 'c'){
		m_canvas_pp.clear();
		m_canvas_robot.clear();
		rotational_path_planning_mode = false;
		only_show_minkw = false;
		show_diagrams = false;
		minkowski_and_voronoi_calculated = false;
		first_point_placed = false;
		second_point_placed = false;
		rotational_points_placed = false;
		cout << endl << endl << "*Canvas Cleared*" << endl << endl;
	}

	if (key == 'b'){
		if (!custom_boundary){
			rectangular_boundary = !rectangular_boundary;
			if (rectangular_boundary)
				cout << "The environment boundaries are now RECTANGULAR" << endl << endl;
			else
				cout << "The environment boundaries are now POLYGONAL" << endl << endl;
		}
	}

	if (key == 'f') {
		if (polygon_obstacles.size() == 0 && line_obstacles.size() == 0){
			cout << endl << "No obstacles detected! This operation requires obstacles to exist" << endl;
			show_diagrams = false;
			return;
		}
		rotational_path_planning_mode = false;
		rotational_points_placed = false;
		first_point_placed = false;
		second_point_placed = false;
		only_show_minkw = false;
		show_diagrams = !show_diagrams;
		minkowski_and_voronoi_calculated = false;
		if (show_diagrams)
			cout << endl << "Working..Please wait" << endl;
	}

	if (key == 's') {
		robot_visible = !robot_visible;
	}

	if (key == 'h') {
		cout << endl;
		cout << "***" << endl;
		cout << "'SpaceBar'  ->" << " Generate new random obstacle" << endl << endl;
		cout << "'t' ->" << " Robot Type Instructions" << endl << endl;
		cout << "'d' ->" << " Robot Dimensions Instructions" << endl << endl;
		cout << "'o' ->" << " Robot Orientation Instructions" << endl << endl;
		cout << "'r' ->" << "Revert Robot settings to default" << endl << endl;
		cout << "'b' ->" << "Switch between rectangular/polygonal environment boundaries" << endl << endl;
		cout << "'u' -> " << "Disable/Enable random obstacle" << endl << endl;
		cout << "***" << endl;
	}

	if (key == 'u'){
		m_canvas.clear();
		m_canvas_pp.clear();
		m_canvas_robot.clear();
		rand_ob_enabled = !rand_ob_enabled;
		if (rand_ob_enabled)
			cout << endl << "*Random obstacle enabled (if it has not appeared, press Spacebar to generate). " << endl;
		else
			cout << endl << "*Random obstacle disabled." << endl;

	}

	if (key == 'o'){

		cout << endl;
		cout << "'q'-> Turn Left" << endl;
		cout << "'w'->Turn Right" << endl;

		cout << endl << "To manually set the orientation" << endl << ", press '~' (use console)" << endl << endl << endl;

	}

	if (key == 'q')
		ChangeRobotOrientation('l');
	if (key == 'w')
		ChangeRobotOrientation('r');

	if (key == '~') {
		double inputangle;
		cout << "Input angle in degrees (must be between 0 and 180): " << endl;
		cin >> inputangle;
		inputangle = (3.14*inputangle) / 180; //convert to radians.
		ChangeRobotOrientation('s', inputangle);
		cout << endl << "New angle set." << endl;
	}


	if (key == ' '){
		if (!custom_modification)
			CreateObstacles(true, adjust);
		else
			DrawObstacles_CustomsIncluded(env_boundary_infile,true);

	}

	if (key == 'r') {
		cout << endl << "All robot settings changed to default" << endl;
		modifyRobot();
		ChangeRobotOrientation();
	}

	if (key == 'd') {
		cout << endl;
		cout << "---Robot Type : " << m_robot.type_text << " ---" << endl;
		switch (m_robot.robot_type) {
		case(disk) :
			cout << "'=' -> Increase radius" << endl;
			cout << "'-' -> Decrease radius" << endl;
			break;
		case(line) :
			cout << "'=' -> Increase length" << endl;
			cout << "'-' -> Decrease length" << endl;
			break;
		case(rectangle) :
			cout << "'=' -> Increase length" << endl;
			cout << "'-' -> Decrease length" << endl;
			cout << "'+' -> Increase width" << endl;
			cout << "'_(underscore)' -> Decrease width" << endl;
			break;
		}
		cout << endl << " To manually set the dimension value"<<endl<<", press '\\' (use console)" << endl <<endl <<endl;
	}

	if (key == '\\') {

		cout << "- - -" << endl;
		switch (m_robot.robot_type) {

		case(disk) :
			double inputradius;
			do {
				cout << "Input radius (must be between 0 and "<<MAX_ROBOT_RADIUS<<" ) :";
				cin >> inputradius;
			} while (inputradius < 0 || inputradius > MAX_ROBOT_RADIUS);
			cout << endl;
			modifyRobot(m_robot.robot_type,inputradius,m_robot.len,m_robot.width,m_robot.center_pos,true,"Radius");
			break;

		case(line) :
			double inputlen;
			do {
				cout << "Input length (must be between 0 and " << MAX_ROBOT_LEN << ") :";
				cin >> inputlen;
			} while (inputlen < 0 || inputlen > MAX_ROBOT_LEN);
			cout << endl;
			modifyRobot(m_robot.robot_type, m_robot.radius, inputlen, m_robot.width, m_robot.center_pos, true, "Length");
			break;
		case(rectangle) :
			double inputlen2, inputwidth;
			do {
				cout << "Input length (must be between 0 and " << MAX_ROBOT_LEN << ") :";
				cin >> inputlen2;
			} while (inputlen2 < 0 || inputlen2 > MAX_ROBOT_LEN);
			cout << endl;
			do {
				cout << "Input width (must be between 0 and " << MAX_ROBOT_WIDTH << ") :";
				cin >> inputwidth;
			} while (inputwidth < 0 || inputwidth > MAX_ROBOT_LEN);
			cout << endl;
			modifyRobot(m_robot.robot_type, m_robot.radius, inputlen2, inputwidth, m_robot.center_pos, true, "Width");
			break;
		}
	}

	if (key == '=') {
		if (m_robot.robot_type == disk && m_robot.radius < MAX_ROBOT_RADIUS)
			modifyRobot(m_robot.robot_type, m_robot.radius + CHANGE_INTERVAL_RADIUS, m_robot.len, m_robot.width);
		else if (m_robot.robot_type == line && m_robot.len < MAX_ROBOT_RADIUS)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len + CHANGE_INTERVAL_LEN, m_robot.width);
		else if (m_robot.robot_type == rectangle&& m_robot.len < MAX_ROBOT_RADIUS)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len + CHANGE_INTERVAL_LEN, m_robot.width);


	}

	if (key == '-') {
		if (m_robot.robot_type == disk && m_robot.radius>15)
			modifyRobot(m_robot.robot_type, m_robot.radius - CHANGE_INTERVAL_RADIUS, m_robot.len, m_robot.width);
		else if (m_robot.robot_type == line && m_robot.len > 10)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len - CHANGE_INTERVAL_LEN, m_robot.width);
		else if (m_robot.robot_type == rectangle && m_robot.len > 10)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len - CHANGE_INTERVAL_LEN, m_robot.width);

	}

	if (key == '+') {
		if (m_robot.robot_type == rectangle && m_robot.width<MAX_ROBOT_WIDTH)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len, m_robot.width + CHANGE_INTERVAL_WIDTH);
	}
	if (key == '_') {
		if (m_robot.robot_type == rectangle && m_robot.width >15)
			modifyRobot(m_robot.robot_type, m_robot.radius, m_robot.len, m_robot.width - CHANGE_INTERVAL_WIDTH);
	}

	if (key == 't') {

		cout << endl;
		cout << "*** Change robot type ***"<<endl;
		cout << "'1'->" << " Disk" << endl;
		cout << "'2'->" << " Line" << endl;
		cout << "'3'->" << " Rectangle" << endl;

	}

	if (key == '1' || key == '2' || key == '3') {
		int temp = key - '0';
		temp--;
		type selected_type = type(temp);
		modifyRobot(selected_type);
		cout << endl;
		cout << "Robot type changed to " << m_robot.type_text << endl;



	}

	if(key == 'p') {
		if (path_planning_mode == false && minkowski_and_voronoi_calculated == false || rotational_path_planning_mode){
			cout << "---Minkowski sums and Voronoi diagram must be calculated first in order to go to path planning mode.--" << endl << endl;
			return;
		}
		path_planning_mode = !path_planning_mode;
		if (path_planning_mode)
			cout << endl << "***Path Planning mode ON : Select two points***" << endl;
		else
			cout << endl << "Path Planning mode OFF" << endl;


	}

	if (key == 'e'){
		if (polygon_obstacles.size() == 0 && line_obstacles.size() == 0){
			cout << endl << "No obstacles detected! This operation requires obstacles to exist" << endl;
			rotational_path_planning_mode = false;
			return;
		}
		m_canvas_pp.clear();
		only_show_minkw = false;
		first_point_placed = false;
		second_point_placed = false;
		if (m_robot.robot_type == disk){
			cout <<endl<< "*Combined motion path planning doesnt apply to disk robot*" << endl;
			return;
		}
		rotational_path_planning_mode = !rotational_path_planning_mode;
		rotational_points_placed = false;
		if (rotational_path_planning_mode){
			cout << endl << "***Path Planning mode ON : Select two points***" << endl;
			cout << endl << "Use the console to input angle at starting and ending points." << endl;
			cout << "Angles must be in entered in degrees,and must be between 0-180." << endl;
			do{
				cout << endl << "Enter the starting angle :";
				cin >> starting_angle;
				starting_angle = degrees_to_radians(starting_angle);
				cout << endl << "Enter the ending angle :";
				cin >> ending_angle;
			} while (starting_angle >= 180 || ending_angle >= 180);
			ending_angle = degrees_to_radians(ending_angle);
			cout << endl << "*Select two points with left mouse button*" << endl;
			cout << endl << "NOTE : please wait after placing the second pont" << endl;
		}
		else
			cout << endl << "Path Planning mode OFF" << endl;
	}



	if (key == 'm'){
		if (polygon_obstacles.size() == 0 && line_obstacles.size() == 0){
			cout <<endl<< "No obstacles detected! This operation requires obstacles to exist" << endl;
			only_show_minkw = false;
			return;
		}
		m_canvas_pp.clear();
		first_point_placed = false;
		second_point_placed = false;
		rotational_path_planning_mode = false;
		rotational_points_placed = false;
		only_show_minkw = !only_show_minkw;

	}

	if (key == 'v'){
		globalcounter = 0;
		if(adjust == 0){
			adjust = 200;
			cout<<endl<<"Display has been adjusted for small screens"<<endl;
		}
		else {
			adjust = 0;
			cout<<endl<<"Display adjust has been disabled"<<endl;
		}
		box_center_x = (BOX_XRIGHT + 0.8*adjust + BOX_XLEFT + 0.8*adjust) / 2.0;
		box_center_y = (BOX_YUP + adjust + BOX_YDOWN + adjust) / 2.0;
		m_robot.center_pos.x = box_center_x;
		m_robot.center_pos.y = box_center_y;
	}

	if(key == '6'){
		cout<<endl<<"1ST SLIDER"<<endl<<"Voronoi discretization interval. Max:100,MIN:10,DEFAULT:"<<DISCRETIZATION_INTERVAL<<endl;
		cout<<endl<<endl<<"2ND SLIDER"<<endl<<"No of maps calculated in combined motion path planning. Max:"<<MAX_MAPS<<",MIN:10,DEFAULT:"<<NO_OF_MAPS<<endl;
	}

	if (key == '9'){
		read_obs_from_file = !read_obs_from_file;
		if (read_obs_from_file){
			obs_read = false;
			cout << endl << "Input file instructions::" << endl;
			cout << endl << "*ALL entries must be separated by a newline" << endl;
			cout << endl << "*For each point,x coordinate must be written first,then y coordinate" << endl;
			cout << endl << "*To distinguish between different obstacles,use -9999 as a distinct entry" << endl;
			cout << endl << endl << "Note: All point coordinates are adjusted to the environment center." << endl << endl;
			cout << endl << "Input the name of the file/pathname (from console) :" << endl;
			cin >> filename;
			cout << "Input 'b' if the file contains points for the boundary, or 'o' if the file contains points for obstacles" << endl;
			char selection;
			cin >> selection;
			if (selection == 'b'){
				env_boundary_infile = true;
				rectangular_boundary = false;
				custom_boundary = true;
				boundary_lines.clear();
				boundary.Clear();

			}
			else{
				env_boundary_infile = false;
				do{
					cout << "Input 'l' if the obstacles are lines,or input 'p' if the obstacles are polygons" << endl;
					cin >> ob_type;
					cout << "Input 'r' if you want this text file to replace current obstacles,or 'a' if you want to add them to the existing ones" << endl;
					cin >> mode;

				} while (ob_type != 'l' && ob_type != 'p' || (mode != 'r' && mode != 'a'));

			}
		}

		}

}



/* Drawing */
void SceneRobotEnvironment::draw()
{
	m_canvas.clear();
	if (!rotational_path_planning_mode)
		m_canvas_robot.clear();

	if (!minkowski_and_voronoi_calculated && !rotational_path_planning_mode)
		m_canvas_pp.clear();


	if (!starting_message) {
		ShowMenu();
		starting_message = true;
	}
	enterPixelMode();
	if (USE_DEFAULT_DATA && !read_obs_from_file) {
		if (globalcounter == 0)
			CreateObstacles(true, adjust);
		else
			CreateObstacles(false, adjust);

	}
	else{
		if (!obs_read){
			ReadObstaclesFromFile(filename,ob_type,mode,env_boundary_infile);
			obs_read = true;
		}
		DrawObstacles_CustomsIncluded(env_boundary_infile,false);
	}


	if (robot_visible)
		ShowRobot(adjust);

	if (only_show_minkw){
		CalculateMinkowski();
		Minkowski_draw();


	}

	if (show_diagrams && !minkowski_and_voronoi_calculated) {
		CalculateMinkowski();
		Generalized_Voronoi();
		if (!legend_shown){
			cout <<endl<< "***" << endl;
			cout <<"ORANGE -> Minkowski sums" << endl;
			cout << endl << "GREEN->Generalized Voronoi approximation" << endl;
			cout << endl << "RED->Discretized Space" << endl;
			cout << "***" << endl << endl;
			legend_shown = true;
		}

	}



	if (minkowski_and_voronoi_calculated && show_diagrams){
		Minkowski_draw();
		Voronoi_draw();


	}

	if (rotational_path_planning_mode && rotational_points_placed){
		draw_total_minkw();
		//Voronoi_draw();

	}

	m_canvas.draw();
	m_canvas_pp.draw();
	m_canvas_robot.draw();
	globalcounter++;
	returnFromPixelMode();



}



//***main CalculateMinkowski function that calls all others.***
void SceneRobotEnvironment::CalculateMinkowski(bool find_freespace) {


	modified_polygon_obstacles.clear();
	modified_polygon_obstacles_union.clear();
	modified_boundary.Clear();
	free_space.clear();
	current_map_obs_union.clear();

	//DISK
	if (m_robot.robot_type == disk) {

		C2DPointSet new_boundary_points;



		//convex obstacles
		for (int w = 0; w < polygon_obstacles.size(); w++) {
			CalculateMinkowski_convex_obstacle(polygon_obstacles[w]);
		}
		//line obstacles
		for (int w = 0; w < line_obstacles.size(); w++) {
			CalculateMinkowski_line_obstacle(line_obstacles[w]);
		}

		//boundary modification
		if (rectangular_boundary) {
			for (int w = 0; w < boundary_lines.size(); w++) {
				CalculateMinkowski_boundary_disk(boundary_lines[w], new_boundary_points);
			}
		}
		else {

			for (int w = 0; w < boundary.GetLineCount(); w++) {
				CalculateMinkowski_boundary_disk(*boundary.GetLine(w), new_boundary_points);
			}
		}

		//boundary : pointset -> c2dpolygon
		if (rectangular_boundary) {

			C2DLine new_top(new_boundary_points[0], new_boundary_points[1]);
			C2DLine new_right(new_boundary_points[2], new_boundary_points[3]);
			C2DLine new_bottom(new_boundary_points[4], new_boundary_points[5]);
			C2DLine new_left(new_boundary_points[6], new_boundary_points[7]);


			C2DPointSet temp;
			C2DPoint p1, p2, p3, p4;
			find_lines_intersection(new_top, new_right, p1);
			find_lines_intersection(new_right, new_bottom, p2);
			find_lines_intersection(new_bottom, new_left, p3);
			find_lines_intersection(new_left, new_top, p4);
			temp.AddCopy(p1);
			temp.AddCopy(p2);
			temp.AddCopy(p3);
			temp.AddCopy(p4);

			modified_boundary = C2DPolygon(temp);

		}

		
		else{

			C2DLine first_line(new_boundary_points[0], new_boundary_points[1]);
			C2DLine second_line(new_boundary_points[2], new_boundary_points[3]);
			C2DPointSet temp;
			C2DPoint temp_intersection_point;
			find_lines_intersection(first_line, second_line, temp_intersection_point);
			temp.AddCopy(temp_intersection_point);

			for (int w = 5; w < new_boundary_points.size(); w = w + 2) {

				C2DLine previous_line(new_boundary_points[w - 3], new_boundary_points[w - 2]);
				C2DLine next_line(new_boundary_points[w - 1], new_boundary_points[w]);
				find_lines_intersection(previous_line, next_line, temp_intersection_point);
				temp.AddCopy(temp_intersection_point);
			}

			//last line is not covered by loop.
			C2DLine last_line(new_boundary_points[new_boundary_points.size() - 2], new_boundary_points[new_boundary_points.size() - 1]);
			find_lines_intersection(first_line, last_line, temp_intersection_point);
			temp.AddCopy(temp_intersection_point);


			modified_boundary = C2DPolygon(temp);


		}
	}

	
	//LINE OR RECTANGLE
	else{



		//convex obstacles
		for (int w = 0; w < polygon_obstacles.size(); w++)
			CalculateMinkowski_convex_obstacle(polygon_obstacles[w]);

		//line obstacles
		for (int w = 0; w < line_obstacles.size(); w++)
			CalculateMinkowski_line_obstacle(line_obstacles[w]);




		//boundary modification
		if (rectangular_boundary){

			C2DPointSet boundary_points;
			for (int w = 0; w < boundary_lines.size(); w++){
				//convert to polygon representation
				boundary_points.AddCopy(boundary_lines[w].GetPointFrom());


			}
			C2DPolygon poly_boundary = C2DPolygon(boundary_points, true);
			CalculateMinkowski_boundary_general(poly_boundary);

		}

		else
			CalculateMinkowski_boundary_general(boundary);




	}
	if(find_freespace)
		Minkowski_find_free_space(); //find the space in which the robot can move.



}

//Boundary lines modification,for disk robot only. (returns pointset,conversion to polygon happens outside this function)
void SceneRobotEnvironment::CalculateMinkowski_boundary_disk(C2DLine boundary_line,C2DPointSet& boundary_points){
		C2DVector normal(boundary_line.GetPointFrom(), boundary_line.GetPointTo());
		normal.TurnRight();
		normal.SetLength(m_robot.radius);
		C2DLine templine1(boundary_line.GetPointFrom(), normal);
		C2DLine templine2(boundary_line.GetPointTo(), normal);
		boundary_points.AddCopy(templine1.GetPointTo());
		boundary_points.AddCopy(templine2.GetPointTo());

}

//Boundary lines modification,for all other robot types.
void SceneRobotEnvironment::CalculateMinkowski_boundary_general(C2DPolygon boundary){

	if (m_robot.robot_type == line){

		std::vector<double> offset_distance; //the distance each side of the boundary will be deflated.
		std::vector<C2DLine> temp_new_lines;
		C2DPointSet modified_boundary_points;
		//for each line of the boundary: ((assuming counterclockwise arrangement)
		for (int w = 0; w < boundary.GetLineCount(); w++){

			//make the center of the line robot,the midpoint of the boundary line.
			C2DPoint line_robot_center = boundary.GetLine(w)->GetMidPoint();
			//find the bottom point of the line robot (taking its orientation into account)
			C2DVector opposite_orientation(-m_robot.orientation.i, -m_robot.orientation.j);
			C2DLine line_to_bottom(line_robot_center, opposite_orientation);
			line_to_bottom.SetLength(m_robot.len / 2.0);
			C2DPoint offset_limit_line_start = line_to_bottom.GetPointTo();
			//
			C2DVector boundaryline_normal(boundary.GetLine(w)->GetPointFrom(), boundary.GetLine(w)->GetPointTo());
			boundaryline_normal.TurnRight();
			C2DLine boundaryline_normal_l(boundary.GetLine(w)->GetMidPoint(), boundaryline_normal);
			boundaryline_normal_l.GrowFromCentre(5000);
			//
			boundaryline_normal.TurnLeft();
			C2DLine offset_limit_line(offset_limit_line_start, boundaryline_normal);
			offset_limit_line.GrowFromCentre(5000);
			//
			C2DPoint offset_limit_point;
			find_lines_intersection(offset_limit_line, boundaryline_normal_l,offset_limit_point);
			//
			C2DLine offset_distance_line(boundary.GetLine(w)->GetMidPoint(), offset_limit_point);
			double offsetdistance = offset_distance_line.GetLength();

			offset_distance.push_back(offsetdistance);


		}

		for (int w = 0; w < boundary.GetLineCount(); w++){

			C2DPoint startpoint = boundary.GetLine(w)->GetPointFrom();
			C2DPoint endpoint = boundary.GetLine(w)->GetPointTo();
			C2DVector normal(startpoint, endpoint);
			normal.TurnRight();
			C2DLine offset1(startpoint, normal);
			C2DLine offset2(endpoint, normal);
			offset1.SetLength(offset_distance[w]);
			offset2.SetLength(offset_distance[w]);

			C2DLine line_to_add(offset1.GetPointTo(), offset2.GetPointTo());
			temp_new_lines.push_back(line_to_add);

		}

		for (int w = 0; w < temp_new_lines.size()-1; w++){

			C2DPoint current_intersection_point;
			find_lines_intersection(temp_new_lines[w], temp_new_lines[w + 1], current_intersection_point);
			modified_boundary_points.AddCopy(current_intersection_point);


		}

		C2DPoint last_intersection_point;
		find_lines_intersection(temp_new_lines[0], temp_new_lines[temp_new_lines.size() - 1], last_intersection_point);
		modified_boundary_points.AddCopy(last_intersection_point);

		modified_boundary = C2DPolygon(modified_boundary_points, true);


	}

	/////////////////////////////////////////////////////////////////////////////////////////////

	else if (m_robot.robot_type == rectangle){


		//both the width and the length contribute in the offset (how much each side will be deflated).
		std::vector<double> offset_distance_width;
		std::vector<double> offset_distance_length;
		std::vector<double> offset_distance_total;

		std::vector<C2DLine> temp_new_lines;
		C2DPointSet modified_boundary_points;


		for (int w = 0; w < boundary.GetLineCount(); w++){

			//width contribution
			offset_distance_width.push_back(rectangle_robot_find_offset_distance(*boundary.GetLine(w), m_robot.width / 2.0, m_robot.orientation));
			//length contribution
			offset_distance_length.push_back(rectangle_robot_find_offset_distance(*boundary.GetLine(w), m_robot.len / 2.0, m_robot.orientation));

		}

		//pythagorean theorem to find the total offset
		for (int w = 0; w < offset_distance_width.size(); w++){

			offset_distance_total.push_back(pythagorean_theorem(offset_distance_width[w], offset_distance_length[w]));
		}

		for (int w = 0; w < boundary.GetLineCount(); w++){

			C2DPoint startpoint = boundary.GetLine(w)->GetPointFrom();
			C2DPoint endpoint = boundary.GetLine(w)->GetPointTo();
			C2DVector normal(startpoint, endpoint);
			normal.TurnRight();
			C2DLine offset1(startpoint, normal);
			C2DLine offset2(endpoint, normal);
			offset1.SetLength(offset_distance_total[w]);
			offset2.SetLength(offset_distance_total[w]);

			C2DLine line_to_add(offset1.GetPointTo(), offset2.GetPointTo());
			temp_new_lines.push_back(line_to_add);

		}

		for (int w = 0; w < temp_new_lines.size() - 1; w++){

			C2DPoint current_intersection_point;
			find_lines_intersection(temp_new_lines[w], temp_new_lines[w + 1], current_intersection_point);
			modified_boundary_points.AddCopy(current_intersection_point);


		}

		C2DPoint last_intersection_point;
		find_lines_intersection(temp_new_lines[0], temp_new_lines[temp_new_lines.size() - 1], last_intersection_point);
		modified_boundary_points.AddCopy(last_intersection_point);

		modified_boundary = C2DPolygon(modified_boundary_points, true);


	}


}

void SceneRobotEnvironment::Minkowski_draw() {

	for (int w = 0; w < free_space.size(); w++)
		vvr::draw(free_space[w], vvr::Colour::orange);

}

void SceneRobotEnvironment::Minkowski_find_free_space() {

	free_space.clear();

	Paths clipper_obstacles;
	Paths polygon_union;
	//convert to clipper data
	for (int w = 0; w < modified_polygon_obstacles.size(); w++) {

		Path temp;
		geolib_2_clipper(modified_polygon_obstacles[w], temp);
		clipper_obstacles.push_back(temp);
	}
	//find union
	Clipper c;
	//compute in pairs
	//first pair
	c.AddPath(clipper_obstacles[0], ptSubject, true);
	c.AddPath(clipper_obstacles[1], ptClip, true);
	c.Execute(ctUnion, polygon_union);

	for (int w = 2; w < clipper_obstacles.size(); w++){

		Clipper c;
		c.AddPaths(polygon_union, ptSubject, true);
		c.AddPath(clipper_obstacles[w], ptClip, true);
		c.Execute(ctUnion, polygon_union);
	}

	//useful for rotational path planning, and optimal path calculation
	for (int w = 0; w < polygon_union.size(); w++){

		C2DPolygon g_temp;
		clipper_2_geolib(polygon_union[w], g_temp);
		modified_polygon_obstacles_union.push_back(g_temp);
		current_map_obs_union.push_back(g_temp);
	}

	//find usable space (difference between union of obstacles and modified boundary) 
	Path modified_boundary_clipper;
	geolib_2_clipper(modified_boundary, modified_boundary_clipper);
	Clipper d;
	d.AddPaths(polygon_union, ptClip, true);
	d.AddPath(modified_boundary_clipper, ptSubject, true);
	d.Execute(ctDifference, polygon_union);


	//convert back to geolib data

	for (int w = 0; w < polygon_union.size(); w++){

		C2DPolygon g_temp;
		clipper_2_geolib(polygon_union[w], g_temp);
		free_space.push_back(g_temp);
	}

}
void SceneRobotEnvironment::CalculateMinkowski_line_obstacle(C2DLine obstacle){

	///////////////////////////////////////////////////////////
	//DISK//
		C2DPointSet collision_area_points;
		if (m_robot.robot_type == disk) {

			C2DVector temp_dir = C2DVector(obstacle.GetPointFrom(), obstacle.GetPointTo());
			C2DLine templine1(obstacle.GetPointTo(), temp_dir);
			templine1.SetLength(m_robot.radius);
			temp_dir = C2DVector(templine1.GetPointFrom(), templine1.GetPointTo());
			temp_dir.TurnRight();
			temp_dir.SetLength(m_robot.radius);
			C2DLine templine1_1(templine1.GetPointTo(), temp_dir);
			collision_area_points.AddCopy(templine1_1.GetPointTo());
			temp_dir.TurnLeft();
			temp_dir.TurnLeft();
			C2DLine templine1_2(templine1.GetPointTo(), temp_dir);
			collision_area_points.AddCopy(templine1_2.GetPointTo());

			//
			temp_dir = C2DVector(obstacle.GetPointTo(), obstacle.GetPointFrom());
			C2DLine templine2(obstacle.GetPointFrom(), temp_dir);
			templine2.SetLength(m_robot.radius);
			temp_dir = C2DVector(templine2.GetPointFrom(), templine2.GetPointTo());
			temp_dir.TurnRight();
			temp_dir.SetLength(m_robot.radius);
			C2DLine templine2_1(templine2.GetPointTo(), temp_dir);
			collision_area_points.AddCopy(templine2_1.GetPointTo());
			temp_dir.TurnLeft();
			temp_dir.TurnLeft();
			C2DLine templine2_2(templine2.GetPointTo(), temp_dir);
			collision_area_points.AddCopy(templine2_2.GetPointTo());


			C2DPolygon collision_area(collision_area_points, true);
			modified_polygon_obstacles.push_back(collision_area);


			}




	/////////////////////////////////////////////////////////////
	//LINE AND RECTANGLE//
	else  {


		//break obstacle line into its 2 limit points,then do regular minkowski addition

		//LINE ROBOT ONLY: first check if line has exactly the same direction with line obstacle.if thats the case,then create a minimal
		//minkowski polygon to represent a minimal safety distance.
		if (m_robot.robot_type == line){
			bool identical_dirs;
			compare_directions(obstacle,identical_dirs);
			if (identical_dirs){
				approx_minkw_for_lineobs(obstacle);
				return;
			}
		}
		robot_points.clear();
		line_ob_points.clear();
		minkw_points.clear();

		add_robot_points(m_robot.type_text,m_robot, &robot_points);
		add_vertices_line_ob(obstacle, &line_ob_points);
		minkw_addition(robot_points, line_ob_points, &minkw_points);

		C2DPointSet minkw_points_pointset;
		vector_to_pointset(minkw_points, &minkw_points_pointset);
		C2DPointSet convhullpoints_pointset;
		convhullpoints_pointset.ExtractConvexHull(minkw_points_pointset);
		C2DPointSet sorted_convhullpoints;
		SortByAngle3(&convhullpoints_pointset);


		C2DPolygon convhull_poly = C2DPolygon(convhullpoints_pointset, false);
		modified_polygon_obstacles.push_back(convhull_poly);




	}

	return;

}
void SceneRobotEnvironment::CalculateMinkowski_convex_obstacle(C2DPolygon obstacle) {

	/////////////////////////////////////////////
	//DISK
	if (m_robot.robot_type == disk) {
		C2DLineSet obstacle_lines;
		std::vector<C2DLine> perp_dirs_firstlines;
		std::vector<C2DLine> perp_dirs_secondlines;
		perp_dirs_firstlines.clear();
		perp_dirs_secondlines.clear();

		for (int w = 0; w < obstacle.GetLineCount();w++) {
			obstacle_lines.AddCopy(*obstacle.GetLine(w));
			}

	//find outward perpendicular direction for every line of obstacle,save in 'perp_dirs'.
	//assuming polygon points are recorded in a clockwise manner.
	for (int w = 0; w < obstacle.GetLineCount(); w++) {
		C2DVector temp_perp_dir = C2DVector(obstacle.GetLine(w)->GetPointFrom(), obstacle.GetLine(w)->GetPointTo());
		temp_perp_dir.TurnLeft();
		C2DLine first_line, second_line;
		first_line = C2DLine(obstacle.GetLine(w)->GetPointFrom(), temp_perp_dir);
		second_line = C2DLine(obstacle.GetLine(w)->GetPointTo(), temp_perp_dir);
		perp_dirs_firstlines.push_back(first_line);
		perp_dirs_secondlines.push_back(second_line);
	}


		C2DPointSet temp_minkw;
		temp_minkw.DeleteAll();

		C2DPoint limitline_point1 = perp_dirs_secondlines[0].GetPointTo();
		C2DPoint limitline_point2;
		for (int w = 1; w < obstacle.GetLineCount(); w++) {

			//make edges smoother
			limitline_point2 = perp_dirs_firstlines[w].GetPointTo();
			C2DLine limitline(limitline_point1,limitline_point2);
			C2DPoint limitline_midpoint = limitline.GetMidPoint();
			C2DLine offset_line(perp_dirs_firstlines[w].GetPointFrom(),limitline_midpoint);
			offset_line.SetLength(m_robot.radius);
			temp_minkw.AddCopy(offset_line.GetPointTo());


			perp_dirs_firstlines[w].SetLength(m_robot.radius);
			perp_dirs_secondlines[w].SetLength(m_robot.radius);
			//visualize
			temp_minkw.AddCopy(perp_dirs_firstlines[w].GetPointTo());
			temp_minkw.AddCopy(perp_dirs_secondlines[w].GetPointTo());

			limitline_point1 = perp_dirs_secondlines[w].GetPointTo();
		}
		//make last edge smoother
		limitline_point2 = perp_dirs_firstlines[0].GetPointTo();
		C2DLine limitline(limitline_point1,limitline_point2);
		C2DPoint limitline_midpoint = limitline.GetMidPoint();
		C2DLine offset_line(perp_dirs_firstlines[0].GetPointFrom(),limitline_midpoint);
		offset_line.SetLength(m_robot.radius);
		temp_minkw.AddCopy(offset_line.GetPointTo());

		C2DPolygon collision_area = C2DPolygon(temp_minkw, false);
		modified_polygon_obstacles.push_back(collision_area);


	}


	///////////////////////////////////////////////////////////////////////
	//LINE AND RECTANGLE//
	else {

		robot_points.clear();
		minkw_points.clear();
		current_obstacle_edges.clear();

		add_robot_points(m_robot.type_text, m_robot, &robot_points);


		current_obstacle_edges.clear();
		minkw_points.clear();
		add_vertices(obstacle, &current_obstacle_edges);
		minkw_addition(robot_points, current_obstacle_edges, &minkw_points);

		C2DPointSet minkw_points_pointset;
		vector_to_pointset(minkw_points, &minkw_points_pointset);
		C2DPointSet convhullpoints_pointset;
		convhullpoints_pointset.ExtractConvexHull(minkw_points_pointset);
		SortByAngle3(&convhullpoints_pointset);
		C2DPolygon convhull_poly = C2DPolygon(convhullpoints_pointset, false);
		modified_polygon_obstacles.push_back(convhull_poly);




	}


	return;
}

void SceneRobotEnvironment::compare_directions(C2DLine line_ob, bool& identical_dirs){

	C2DVector line_ob_dir(line_ob.GetPointFrom(), line_ob.GetPointTo());
	C2DVector reverse_line_ob_dir(line_ob.GetPointTo(), line_ob.GetPointFrom());

	if (line_ob_dir.j == 0 && line_ob_dir.i !=0){
		if (m_robot.orientation.j == 0 && m_robot.orientation.i != 0)
			identical_dirs = true;
		else
			identical_dirs = false;
		return;
	}

	if (line_ob_dir.i == 0 && line_ob_dir.j != 0){
		if (m_robot.orientation.i == 0 && m_robot.orientation.j != 0)
			identical_dirs = true;
		else
			identical_dirs = false;
		return;
	}

	double robot_angle = m_robot.orientation.j / m_robot.orientation.i;
	double ob_angle = line_ob_dir.j / line_ob_dir.i;
	if (abs(robot_angle - ob_angle) < 0.175) //0.175 radians = approx 10 degrees.
		identical_dirs = true;
	else
		identical_dirs = false;
	return;
}


//default and random obstacle creation.
void SceneRobotEnvironment::CreateObstacles(bool init,int adjust)
{

	polygon_obstacles.clear();
	line_obstacles.clear();
	boundary_lines.clear();
	boundary.Clear();

	if (init) {



		//random center obstacles
		if (rand_ob_enabled){
			int topleft_y, topleft_x, bottomright_x, bottomright_y;
			topleft_y = rand() % (RAND_OBST_YMAX - adjust);
			topleft_x = rand() % (RAND_OBST_XMAX - adjust);
			topleft_x = -topleft_x;
			bottomright_x = rand() % (RAND_OBST_XMAX - adjust);
			bottomright_y = rand() % (RAND_OBST_YMAX - adjust);
			bottomright_y = -bottomright_y;


			C2DPoint topleft_r(topleft_x, topleft_y);
			C2DPoint bottomright_r(bottomright_x, bottomright_y);
			const C2DRect bounding_rect = C2DRect(topleft_r, bottomright_r);
			C2DPolygon testpoly;

			short internal_counter = 0;
			do {
				testpoly.CreateRandom(bounding_rect, 3, 10);
				if (internal_counter > 100){

					break;
				}
				internal_counter++;

			} while (!testpoly.IsConvex() || testpoly.GetArea() < 500);



			perm_poly = testpoly;

			vvr::draw(perm_poly, vvr::Colour::black, true);


		}



	}

	//environment boundaries
	if (rectangular_boundary) {
		C2DPoint topleft = C2DPoint(-X_MAX+350+adjust, Y_MAX-0.5*adjust);
		C2DPoint bottomright = C2DPoint(X_MAX-adjust, -Y_MAX+100);
		C2DPoint topright = C2DPoint(X_MAX-adjust, Y_MAX-0.5*adjust);
		C2DPoint bottomleft = C2DPoint(-X_MAX+350+adjust, -Y_MAX+100);
		C2DLine l1(topleft, topright);
		C2DLine l2(topright, bottomright);
		C2DLine l3(bottomright, bottomleft);
		C2DLine l4(bottomleft, topleft);

		environment_center.y = (topright.y + bottomright.y)/2.0;
		environment_center.x = (topright.x + topleft.x)/2.0;

		boundary_lines.push_back(l1);
		boundary_lines.push_back(l2);
		boundary_lines.push_back(l3);
		boundary_lines.push_back(l4);
		m_canvas.add(l1,vvr::Colour::white);
		m_canvas.add(l2,vvr::Colour::white);
		m_canvas.add(l3,vvr::Colour::white);
		m_canvas.add(l4,vvr::Colour::white);
	}
	else{
		//polygonal boundaries
		if(adjust == 0){
		C2DPoint bound1(B_X_1, B_Y_1);
		C2DPoint bound2(B_X_2, B_Y_2);
		C2DPoint bound3(B_X_3, B_Y_3);
		C2DPoint bound4(B_X_4, B_Y_4);
		C2DPoint bound5(B_X_5, B_Y_5);
		C2DPoint bound6(B_X_6, B_Y_6);
		C2DPoint bound7(B_X_7, B_Y_7);
		C2DPoint bound8(B_X_8, B_Y_8);
		C2DPointSet bound_temp;
		bound_temp.AddCopy(bound1);
		bound_temp.AddCopy(bound2);
		bound_temp.AddCopy(bound3);
		bound_temp.AddCopy(bound4);
		bound_temp.AddCopy(bound5);
		bound_temp.AddCopy(bound6);
		bound_temp.AddCopy(bound7);
		bound_temp.AddCopy(bound8);

		boundary = C2DPolygon(bound_temp);
		vvr::draw(boundary, vvr::Colour::white);
	}
	else{
		C2DPoint bound1(B_X_1_S, B_Y_1_S);
		C2DPoint bound2(B_X_2_S, B_Y_2_S);
		C2DPoint bound3(B_X_3_S, B_Y_3_S);
		C2DPoint bound4(B_X_4_S, B_Y_4_S);
		C2DPoint bound5(B_X_5_S, B_Y_5_S);
		C2DPoint bound6(B_X_6_S, B_Y_6_S);
		C2DPoint bound7(B_X_7_S, B_Y_7_S);
		C2DPoint bound8(B_X_8_S, B_Y_8_S);
		C2DPointSet bound_temp;
		bound_temp.AddCopy(bound1);
		bound_temp.AddCopy(bound2);
		bound_temp.AddCopy(bound3);
		bound_temp.AddCopy(bound4);
		bound_temp.AddCopy(bound5);
		bound_temp.AddCopy(bound6);
		bound_temp.AddCopy(bound7);
		bound_temp.AddCopy(bound8);

		boundary = C2DPolygon(bound_temp);
		vvr::draw(boundary, vvr::Colour::white);


	}

	}





	//hardcoded line obstacles
	C2DPoint lp1(LINE_X_1+adjust, LINE_Y_1);
	C2DPoint lp2(LINE_X_2+adjust, LINE_Y_2);
	C2DLine line_obstacle3(lp1, lp2);

	C2DPoint l2p1(LINE2_X_1+adjust, LINE2_Y-adjust);
	C2DPoint l2p2(LINE2_X_2+adjust, LINE2_Y-adjust);
	C2DLine line_obstacle4(l2p1, l2p2);


	line_obstacles.push_back(line_obstacle3);
	line_obstacles.push_back(line_obstacle4);

	//hardcoded obstacles(other)
	C2DPoint trp1(TRIANGLE_X1-adjust, TRIANGLE_Y1);
	C2DPoint trp2(TRIANGLE_X2-adjust, TRIANGLE_Y2);
	C2DPoint trp3(TRIANGLE_X3-adjust, TRIANGLE_Y3);
	C2DTriangle tr(trp1, trp2, trp3);
	C2DPointSet temp;
	temp.AddCopy(tr.GetPoint1());
	temp.AddCopy(tr.GetPoint2());
	temp.AddCopy(tr.GetPoint3());
	C2DPolygon tr_p = C2DPolygon(temp, true);

	C2DPoint polyp1(POLY_X1, POLY_Y1-0.5*adjust);
	C2DPoint polyp2(POLY_X2, POLY_Y2-0.5*adjust);
	C2DPoint polyp3(POLY_X3, POLY_Y3-0.5*adjust);
	C2DPoint polyp4(POLY_X4, POLY_Y4-0.5*adjust);
	C2DPoint polyp5(POLY_X5, POLY_Y5-0.5*adjust);
	C2DPointSet temp2;
	temp2.AddCopy(polyp1);
	temp2.AddCopy(polyp2);
	temp2.AddCopy(polyp3);
	temp2.AddCopy(polyp4);
	temp2.AddCopy(polyp5);
	C2DPolygon h_p = C2DPolygon(temp2, true);


	C2DPoint recp1(REC_X1, REC_Y1-0.5*adjust);
	C2DPoint recp2(REC_X2, REC_Y2-0.5*adjust);
	C2DPoint recp3(REC_X3, REC_Y3-0.5*adjust);
	C2DPoint recp4(REC_X4, REC_Y4-0.5*adjust);
	temp.DeleteAll();
	temp.AddCopy(recp1);
	temp.AddCopy(recp2);
	temp.AddCopy(recp3);
	temp.AddCopy(recp4);
	C2DPolygon rec_p = C2DPolygon(temp, true);



	polygon_obstacles.push_back(tr_p);
	polygon_obstacles.push_back(rec_p);
	polygon_obstacles.push_back(h_p);

	polygon_obstacles.push_back(perm_poly);

	m_canvas.add(line_obstacle3);
	m_canvas.add(line_obstacle4);
	m_canvas.add(tr, vvr::Colour::black, true);
	vvr::draw(rec_p, vvr::Colour::black, true);
	vvr::draw(h_p, vvr::Colour::black, true);

	if (rand_ob_enabled)
		vvr::draw(perm_poly, vvr::Colour::black, true);


}


void SceneRobotEnvironment::modifyRobot(type new_robot_type, double new_radius, double new_len ,double new_width,C2DPoint new_center_pos,bool output_msg,string dimchanged ){


		m_robot.robot_type = new_robot_type;
		m_robot.radius = new_radius;
		m_robot.center_pos = new_center_pos;
		m_robot.len = new_len;
		m_robot.width = new_width;

		double dimchangedVal;
		if (dimchanged == "Radius")
			dimchangedVal = new_radius;
		else if (dimchanged == "Length")
			dimchangedVal = new_len;
		else if (dimchanged == "Width")
			dimchangedVal = new_width;

		switch (new_robot_type){
		case(disk) :
			m_robot.type_text = "disk";
			break;
		case(line) :
			m_robot.type_text = "line";
			break;
		case(rectangle) :
			m_robot.type_text = "rectangle";
			break;
		}

		if (output_msg){
		
			cout << endl << dimchanged << " : " << dimchangedVal;
		}

}
void SceneRobotEnvironment::ShowRobot(int adjust) {



	if (m_robot.robot_type == disk) {
		C2DCircle diskrobot(m_robot.center_pos, m_robot.radius);
		m_canvas.add(diskrobot, Colour::blue, true);

	}
	if (m_robot.robot_type == line) {
		C2DPoint line_center = m_robot.center_pos;

		C2DLine line_to_top(line_center, m_robot.orientation);
		line_to_top.SetLength(m_robot.len / 2.0);
		C2DVector opposite_orientation(-m_robot.orientation.i, -m_robot.orientation.j);
		C2DLine line_to_bottom(line_center, opposite_orientation);
		line_to_bottom.SetLength(m_robot.len / 2.0);

		C2DPoint top_point = line_to_top.GetPointTo();
		C2DPoint bottom_point = line_to_bottom.GetPointTo();
		C2DLine linerobot(bottom_point, top_point);
		m_canvas.add(linerobot, Colour::blue);

	}

	if (m_robot.robot_type == rectangle) {
		C2DLine rec_top, rec_bottom, rec_right, rec_left;
		C2DPoint toprightcorner, topleftcorner, bottomrightcorner, bottomleftcorner;

		C2DPoint rectangle_center = m_robot.center_pos;

		C2DLine line_to_top(rectangle_center, m_robot.orientation);
		line_to_top.SetLength(m_robot.len / 2.0);

		C2DVector temp_orientation = m_robot.orientation;
		temp_orientation.TurnRight();
		C2DLine line_to_topright_corner(line_to_top.GetPointTo(), temp_orientation);
		line_to_topright_corner.SetLength(m_robot.width / 2.0);
		toprightcorner = line_to_topright_corner.GetPointTo();

		temp_orientation = m_robot.orientation;
		temp_orientation.TurnLeft();
		C2DLine line_to_topleft_corner(line_to_top.GetPointTo(), temp_orientation);
		line_to_topleft_corner.SetLength(m_robot.width / 2.0);
		topleftcorner = line_to_topleft_corner.GetPointTo();

		C2DVector opposite_orientation(-m_robot.orientation.i, -m_robot.orientation.j);
		C2DLine line_to_bottom(rectangle_center, opposite_orientation);
		line_to_bottom.SetLength(m_robot.len / 2.0);

		temp_orientation = opposite_orientation;
		temp_orientation.TurnRight();
		C2DLine line_to_bottomleft_corner(line_to_bottom.GetPointTo(), temp_orientation);
		line_to_bottomleft_corner.SetLength(m_robot.width / 2.0);
		bottomleftcorner = line_to_bottomleft_corner.GetPointTo();

		temp_orientation = opposite_orientation;
		temp_orientation.TurnLeft();
		C2DLine line_to_bottomright_corner(line_to_bottom.GetPointTo(), temp_orientation);
		line_to_bottomright_corner.SetLength(m_robot.width / 2.0);
		bottomrightcorner = line_to_bottomright_corner.GetPointTo();

		rec_top = C2DLine(topleftcorner, toprightcorner);
		rec_right = C2DLine(toprightcorner, bottomrightcorner);
		rec_bottom = C2DLine(bottomrightcorner, bottomleftcorner);
		rec_left = C2DLine(bottomleftcorner, topleftcorner);


		m_canvas.add(rec_top, Colour::blue);
		m_canvas.add(rec_bottom, Colour::blue);
		m_canvas.add(rec_right, Colour::blue);
		m_canvas.add(rec_left, Colour::blue);



	}

	if (SHOW_BOX){

		C2DPoint box_topright(BOX_XRIGHT+0.8*adjust, BOX_YUP+adjust);
		C2DPoint box_topleft(BOX_XLEFT+0.8*adjust, BOX_YUP+adjust);
		C2DPoint box_bottomright(BOX_XRIGHT+0.8*adjust, BOX_YDOWN+adjust);
		C2DPoint box_bottomleft(BOX_XLEFT+0.8*adjust, BOX_YDOWN+adjust);
		C2DLine box_top(box_topleft, box_topright);
		C2DLine box_right(box_topright, box_bottomright);
		C2DLine box_bottom(box_bottomright, box_bottomleft);
		C2DLine box_left(box_bottomleft, box_topleft);


		m_canvas.add(box_top, Colour::white);
		m_canvas.add(box_bottom, Colour::white);
		m_canvas.add(box_right, Colour::white);
		m_canvas.add(box_left, Colour::white);
	}




}

//obstacle and environment data , input from file.
void SceneRobotEnvironment::ReadObstaclesFromFile(string filename,char obstacle_type,char mode,bool env_boundary){

	if (mode == 'r'){
		polygon_obstacles.clear();
		line_obstacles.clear();
	}
	if (!env_boundary){
		//environment boundaries
		if (rectangular_boundary) {
			C2DPoint topleft = C2DPoint(-X_MAX + 350 + adjust, Y_MAX - 0.5*adjust);
			C2DPoint bottomright = C2DPoint(X_MAX - adjust, -Y_MAX + 100);
			C2DPoint topright = C2DPoint(X_MAX - adjust, Y_MAX - 0.5*adjust);
			C2DPoint bottomleft = C2DPoint(-X_MAX + 350 + adjust, -Y_MAX + 100);
			C2DLine l1(topleft, topright);
			C2DLine l2(topright, bottomright);
			C2DLine l3(bottomright, bottomleft);
			C2DLine l4(bottomleft, topleft);

			environment_center.y = (topright.y + bottomright.y) / 2.0;
			environment_center.x = (topright.x + topleft.x) / 2.0;

			boundary_lines.push_back(l1);
			boundary_lines.push_back(l2);
			boundary_lines.push_back(l3);
			boundary_lines.push_back(l4);

		}
		else{
			//polygonal boundaries
			if (adjust == 0){
				C2DPoint bound1(B_X_1, B_Y_1);
				C2DPoint bound2(B_X_2, B_Y_2);
				C2DPoint bound3(B_X_3, B_Y_3);
				C2DPoint bound4(B_X_4, B_Y_4);
				C2DPoint bound5(B_X_5, B_Y_5);
				C2DPoint bound6(B_X_6, B_Y_6);
				C2DPoint bound7(B_X_7, B_Y_7);
				C2DPoint bound8(B_X_8, B_Y_8);
				C2DPointSet bound_temp;
				bound_temp.AddCopy(bound1);
				bound_temp.AddCopy(bound2);
				bound_temp.AddCopy(bound3);
				bound_temp.AddCopy(bound4);
				bound_temp.AddCopy(bound5);
				bound_temp.AddCopy(bound6);
				bound_temp.AddCopy(bound7);
				bound_temp.AddCopy(bound8);

				boundary = C2DPolygon(bound_temp);

			}
			else{
				C2DPoint bound1(B_X_1_S, B_Y_1_S);
				C2DPoint bound2(B_X_2_S, B_Y_2_S);
				C2DPoint bound3(B_X_3_S, B_Y_3_S);
				C2DPoint bound4(B_X_4_S, B_Y_4_S);
				C2DPoint bound5(B_X_5_S, B_Y_5_S);
				C2DPoint bound6(B_X_6_S, B_Y_6_S);
				C2DPoint bound7(B_X_7_S, B_Y_7_S);
				C2DPoint bound8(B_X_8_S, B_Y_8_S);
				C2DPointSet bound_temp;
				bound_temp.AddCopy(bound1);
				bound_temp.AddCopy(bound2);
				bound_temp.AddCopy(bound3);
				bound_temp.AddCopy(bound4);
				bound_temp.AddCopy(bound5);
				bound_temp.AddCopy(bound6);
				bound_temp.AddCopy(bound7);
				bound_temp.AddCopy(bound8);

				boundary = C2DPolygon(bound_temp);



			}

		}


		//////////////////////////////////////////
		std::vector<std::vector<C2DPoint>> obstacles;
		short id = 0; //0 for x coordinate,1 for y coordinate
		std::vector<C2DPoint> current_obstacle;
		double current_coordinate_x;
		double current_coordinate_y;
		C2DPoint current_point;

		ifstream input_file(filename);
		if (!input_file){
			cout << endl << "Error : The file provided is empty,or the pathname was wrong" << endl;
			read_obs_from_file = false;
			return;
		}
		while (input_file){

			double data;
			input_file >> data;
			if (id == 0){
				current_coordinate_x = data;
				id = 1;
			}
			else{
				current_coordinate_y = data;
				id = 0;
				current_point.x = current_coordinate_x;
				current_point.y = current_coordinate_y;
				current_obstacle.push_back(current_point);
				cout << endl << "point read ::" <<current_point.x<<" - "<<current_point.y<< endl;
			}

			if (data == -9999){

				obstacles.push_back(current_obstacle);
				id = 0;
				cout << endl << "new item detected" << endl;
				current_obstacle.clear();
			}

			if (input_file.eof()){
				obstacles.push_back(current_obstacle);

			}


		}


		if (obstacle_type == 'p'){
			for (int w = 0; w < obstacles.size(); w++){
				adjust_to_env_center(obstacles[w]);
				C2DPointSet obstacle_points;
				vector_to_pointset(obstacles[w], &obstacle_points);
				C2DPolygon new_obstacle = C2DPolygon(obstacle_points, true);
				polygon_obstacles.push_back(new_obstacle);
			}
		}
		else{
			for (int w = 0; w < obstacles.size(); w++){
				adjust_to_env_center(obstacles[w]);
				C2DPoint startpoint = obstacles[w][0];
				C2DPoint endpoint = obstacles[w][1];
				C2DLine new_obstacle = C2DLine(startpoint, endpoint);
				line_obstacles.push_back(new_obstacle);


			}
		}


		input_file.close();
	}

	//custom boundary.
	else{

		//////////////////////////////////////////
		std::vector<C2DPoint> boundary_points;
		short id = 0; //0 for x coordinate,1 for y coordinate
		C2DPoint current_boundary_point;
		double current_coordinate_x;
		double current_coordinate_y;
	

		ifstream input_file(filename);
		if (!input_file){
			cout << endl << "Error : The file provided is empty,or the pathname was wrong" << endl;
			read_obs_from_file = false;
			return;
		}
		while (input_file){

			double data;
			input_file >> data;
			if (id == 0){
				current_coordinate_x = data;
				id = 1;
			}
			else{
				current_coordinate_y = data;
				id = 0;
				current_boundary_point.x = current_coordinate_x;
				current_boundary_point.y = current_coordinate_y;
				cout << endl << "point read ::" << current_boundary_point.x << " - " << current_boundary_point.y << endl;
				boundary_points.push_back(current_boundary_point);
			}



		}
		C2DPointSet boundary_points_pointset;
		vector_to_pointset(boundary_points,&boundary_points_pointset);
		//find new environment center
		double new_env_center_y = 0;
		double new_env_center_x = 0;
		for(int w=0;w<boundary_points.size();w++){

			new_env_center_x = new_env_center_x + boundary_points[w].x;
			new_env_center_y = new_env_center_y + boundary_points[w].y;
		}

		new_env_center_x = new_env_center_x / boundary_points.size();
		new_env_center_y = new_env_center_y / boundary_points.size();

		environment_center.x = new_env_center_x;
		environment_center.y = new_env_center_y;

		boundary = C2DPolygon(boundary_points_pointset, true);



	}

	custom_modification = true;
	DrawObstacles_CustomsIncluded(env_boundary_infile,false);

	}

void SceneRobotEnvironment::DrawObstacles_CustomsIncluded(bool custom_boundary,bool draw_randoms){

	if (draw_randoms && rand_ob_enabled){
		for (int w = 0; w < polygon_obstacles.size(); w++){

			if (polygons_are_identical(polygon_obstacles[w], perm_poly)){
				polygon_obstacles.erase(polygon_obstacles.begin() + w);
			}

		}

		//random center obstacles
		if (rand_ob_enabled){
			int topleft_y, topleft_x, bottomright_x, bottomright_y;
			topleft_y = rand() % (RAND_OBST_YMAX - adjust);
			topleft_x = rand() % (RAND_OBST_XMAX - adjust);
			topleft_x = -topleft_x;
			bottomright_x = rand() % (RAND_OBST_XMAX - adjust);
			bottomright_y = rand() % (RAND_OBST_YMAX - adjust);
			bottomright_y = -bottomright_y;


			C2DPoint topleft_r(topleft_x, topleft_y);
			C2DPoint bottomright_r(bottomright_x, bottomright_y);
			const C2DRect bounding_rect = C2DRect(topleft_r, bottomright_r);
			C2DPolygon testpoly;

			short internal_counter = 0;
			do {
				testpoly.CreateRandom(bounding_rect, 3, 10);
				if (internal_counter > 100){

					break;
				}
				internal_counter++;

			} while (!testpoly.IsConvex() || testpoly.GetArea() < 500);



			perm_poly = testpoly;

			vvr::draw(perm_poly, vvr::Colour::black, true);
			polygon_obstacles.push_back(perm_poly);

		}





	}
	for (int w = 0; w < line_obstacles.size(); w++)
		m_canvas.add(line_obstacles[w]);


	for (int w = 0; w < polygon_obstacles.size(); w++)
		vvr::draw(polygon_obstacles[w],vvr::Colour::black,true);

	if (!custom_boundary){
		if (rectangular_boundary){
			for (int w = 0; w < boundary_lines.size(); w++)
				m_canvas.add(boundary_lines[w],vvr::Colour::white);
		}
		else{
			vvr::draw(boundary,vvr::Colour::white);
		}
	}

	else{
		vvr::draw(boundary, vvr::Colour::white);
	}

	


	}


void SceneRobotEnvironment::ChangeRobotOrientation(char direction,double set_angle){


	if (direction == 'd') //revert to default
		m_robot.orientation = C2DVector(1, 0);

	if (direction == 'r') //turn right
		m_robot.orientation.TurnRight(0.05);

	if (direction == 'l') //turn left
		m_robot.orientation.TurnLeft(0.05);

	if (direction == 's') //set angle mode
	{
		m_robot.orientation = C2DVector(1, 0);
		m_robot.orientation.TurnLeft(set_angle);
	}

	modified_polygon_obstacles.clear();
	robot_points.clear();
	line_ob_points.clear();
	current_obstacle_edges.clear();




}

//Generalized Voronoi diagram calculation.
void SceneRobotEnvironment::Generalized_Voronoi() {

	cell_lines.clear();


	std::vector<C2DPoint> discretized_space;
	std::vector<C2DPolygon> planes;
	std::vector<C2DPolygon> cells;

	//step one: discretize space
	Discretize_Space2(discretized_space);

	//step two: voronoi on resulting points
	//for every point,find the intersection of all planes.
	//the intersection of all planes for one target point will result in a cell.
	for (int w = 0; w < discretized_space.size(); w++){
		planes.clear();
		Voronoi(w, discretized_space,planes);
		Voronoi_find_planes_intersection(planes,cells);
	}

	//step three : delete voronoi edges that have starting or ending points inside obstacles or outside boundaries.
	polygons_to_lines(cells, cell_lines);
	Voronoi_delete_unwanted_lines(cell_lines);
	//transform polygonal voronoi representation into graph
	polygon_to_graph();
	delete_graph_leaves();
	//
	for (int w = 0; w < 20; w++) {
		polygon_to_graph();
		delete_graph_leaves();
	}



	cout <<endl<< "Done" << endl;
	minkowski_and_voronoi_calculated = true;


}

void SceneRobotEnvironment::Voronoi_draw() {

	for (int x = 0; x < cell_lines.size(); x++){
		m_canvas.add(cell_lines[x], vvr::Colour::green);
	}


	vvr::draw(freespacepoints, vvr::Colour::red);


}


void SceneRobotEnvironment::Discretize_Space2(std::vector<C2DPoint>& discretized_space){

	std::vector<C2DLine> free_space_lines;
	C2DLineSet freespacelines;

	//break free space into lines
	//for every polygon
	for (int w = 0; w < free_space.size(); w++){
		//record lines
		for (int x = 0; x < free_space[w].GetLineCount(); x++){
				free_space_lines.push_back(*free_space[w].GetLine(x));

		}
	}

	freespacepoints.DeleteAll();

	//for every line-> discretize into points
	for (int w = 0; w < free_space_lines.size(); w++){

		Discretize_Line(discretized_space, free_space_lines[w],freespacepoints);
	}

	vvr::draw(freespacepoints, vvr::Colour::red);


}

void SceneRobotEnvironment::Discretize_Line(std::vector<C2DPoint>& discretized_space, C2DLine target_line,C2DPointSet& freespacepoints,int depth){

	if (depth == 0){
		discretized_space.push_back(target_line.GetPointFrom());
		discretized_space.push_back(target_line.GetPointTo());
		freespacepoints.AddCopy(target_line.GetPointFrom());
		freespacepoints.AddCopy(target_line.GetPointTo());
	}

	discretized_space.push_back(target_line.GetMidPoint());
	freespacepoints.AddCopy(target_line.GetMidPoint());

	C2DLine newline_mid_to_end(target_line.GetMidPoint(),target_line.GetPointTo());
	C2DLine newline_mid_to_start(target_line.GetMidPoint(), target_line.GetPointFrom());
	if (newline_mid_to_start.GetLength() > voronoi_discretization_interval) {
		Discretize_Line(discretized_space, newline_mid_to_end, freespacepoints, depth + 1);
		Discretize_Line(discretized_space, newline_mid_to_start, freespacepoints, depth + 1);
	}


}

void SceneRobotEnvironment::Discretize_Space(std::vector<C2DPoint>& discretized_space) {

	std::vector<C2DLine> free_space_lines;
	std::vector<C2DLine> free_space_large_lines;
	C2DLineSet freespacelines;

	//break free space into lines
		//for every polygon
	for (int w = 0; w < free_space.size(); w++){
		//record lines
		for (int x = 0; x < free_space[w].GetLineCount(); x++){
			free_space_lines.push_back(*free_space[w].GetLine(x));

		}
	}


	C2DPointSet freespacepoints;

	//for every line-> discretize into points
	for (int w = 0; w < free_space_lines.size(); w++) {

		C2DPoint current_point = free_space_lines[w].GetPointFrom();
		discretized_space.push_back(current_point);
		freespacepoints.AddCopy(current_point);
		C2DPoint last_point = free_space_lines[w].GetPointTo();
		discretized_space.push_back(last_point);
		freespacepoints.AddCopy(last_point);

		C2DVector linevector = C2DVector(current_point, last_point);

		bool exceeded_line = false;
		while (!exceeded_line){

			C2DLine displacement(current_point, linevector);
			displacement.SetLength(voronoi_discretization_interval);
			current_point = displacement.GetPointTo();
			C2DVector current_linevector(current_point,last_point);
			if (current_linevector.Dot(linevector) < 0)
				exceeded_line = true;
			else{
				discretized_space.push_back(current_point);
				freespacepoints.AddCopy(current_point);
			}

		}


	}


	vvr::draw(freespacepoints, vvr::Colour::red);

}

//for a given point , calculates the voronoi cell with the plane intersection method , with all other points. The plane intersection (the final step) is implemented
//in a different function.
void SceneRobotEnvironment::Voronoi(unsigned selected_point_index,std::vector<C2DPoint>& discretized_space,std::vector<C2DPolygon>& planes){

	//calculate planes (from bisectors) for every possible pair of selected_point - other points.
	for (int w = 0; w < discretized_space.size(); w++){

		if (w == selected_point_index) //avoid degenerate case.
			continue;

		//find bisector line between pair of points.
		C2DLine line_between_points(discretized_space[selected_point_index], discretized_space[w]);
		C2DPoint bisector_midpoint = line_between_points.GetMidPoint();
		C2DVector bisector_direction(bisector_midpoint, discretized_space[selected_point_index]);

		bisector_direction.TurnRight();
		bisector_direction.SetLength(1);
		C2DLine bisector(bisector_midpoint, bisector_direction);
		bisector.GrowFromCentre(100000);



		//form plane (in an approximate way, representing it with a polygon)
		C2DPointSet plane_points;
		plane_points.AddCopy(bisector.GetPointFrom());
		plane_points.AddCopy(bisector.GetPointTo());
		bisector_direction = C2DVector(bisector_midpoint, discretized_space[selected_point_index]);
		C2DLine topline(bisector.GetPointTo(), bisector_direction);
		topline.SetLength(100000);
		C2DLine bottomline(bisector.GetPointFrom(), bisector_direction);
		bottomline.SetLength(100000);
		plane_points.AddCopy(topline.GetPointTo());
		plane_points.AddCopy(bottomline.GetPointTo());

		C2DPolygon new_plane = C2DPolygon(plane_points,true);
		planes.push_back(new_plane);

	}

}

//finds the plane intersections,for the planes of one point. Uses the clipper library to so.
void SceneRobotEnvironment::Voronoi_find_planes_intersection(std::vector<C2DPolygon>& planes,std::vector<C2DPolygon>& cells){

	Paths clipper_planes;
	Paths planes_intersection;
	//convert planes to clipper data
	for (int x = 0; x < planes.size(); x++){
		Path temp;
		geolib_2_clipper(planes[x], temp);
		clipper_planes.push_back(temp);
	}

	//find intersection
	Clipper c;
	//compute in pairs
	//first pair
	c.AddPath(clipper_planes[0], ptSubject, true);
	c.AddPath(clipper_planes[1], ptClip, true);
	c.Execute(ctIntersection, planes_intersection);



	for (int w = 2; w < clipper_planes.size(); w++){

		Paths planes_intersection_temp;
		Clipper c;
		c.AddPaths(planes_intersection, ptClip, true);
		c.AddPath(clipper_planes[w], ptSubject, true);
		c.Execute(ctIntersection, planes_intersection_temp);
		if (planes_intersection_temp.size() == 0){
			c.AddPaths(planes_intersection, ptSubject, true);
			c.AddPath(clipper_planes[w], ptClip, true);
			c.Execute(ctIntersection, planes_intersection_temp);
			if (planes_intersection_temp.size() == 0)
				continue;
			else
				planes_intersection = planes_intersection_temp;
		}
		else
			planes_intersection = planes_intersection_temp;

	}



	for (int x = 0; x < planes_intersection.size(); x++){
		C2DPolygon temp;
		clipper_2_geolib(planes_intersection[x], temp);
		cells.push_back(temp);
		//

	}


}

//Deletes the cell lines that pass through obstacles or venture outside the environment boundaries.
void SceneRobotEnvironment::Voronoi_delete_unwanted_lines(std::vector<C2DLine>& cell_lines){

	for (int x = 0; x < modified_polygon_obstacles.size(); x++){

		for (int y = 0; y < cell_lines.size(); y++){

			if ((modified_polygon_obstacles[x].Contains(cell_lines[y].GetPointFrom())) || (modified_polygon_obstacles[x].Contains(cell_lines[y].GetPointTo()))){

				cell_lines.erase(cell_lines.begin() + y);
				y--;
			}



		}
	}

	for (int w = 0; w < modified_polygon_obstacles.size(); w++){

		for (int y = 0; y < cell_lines.size(); y++){
			if (modified_polygon_obstacles[w].Crosses(cell_lines[y])){
				cell_lines.erase(cell_lines.begin() + y);
				y--;
			}
		}
	}

	for (int x = 0; x < cell_lines.size(); x++){

		if (!modified_boundary.Contains(cell_lines[x].GetPointFrom()) || !modified_boundary.Contains(cell_lines[x].GetPointTo())){

			cell_lines.erase(cell_lines.begin() + x);
			x--;
		}

	}

	for (int x = 0; x < line_obstacles.size(); x++){

		for (int y = 0; y < cell_lines.size(); y++){

			if (line_obstacles[x].Crosses(cell_lines[y])){

				cell_lines.erase(cell_lines.begin() + y);
				y--;
			}
		}
	}

	for (int x = 0; x < polygon_obstacles.size(); x++){
		for (int y = 0; y < cell_lines.size(); y++){
			if (polygon_obstacles[x].Crosses(cell_lines[y])){
				cell_lines.erase(cell_lines.begin() + y);
				y--;
			}
		}
	}




}

//Safest Path algorithm. A greedy best-first algorithm is implemented. The metric used is the euclidean distance.
void SceneRobotEnvironment::find_safest_path(C2DPoint point_1, C2DPoint point_2,bool& path_is_impossible){

	for (int x = 0; x < graph.size(); x++){

		graph[x].visited = false;
	}

	shortest_safest_path.clear();
	graph_node* first_node;
	graph_node* last_node;
	//assuming path is from point_1 to point_2
	//first find the nearest node to point1, that also generates a valid line(doesnt pass through obstacle or env.boundary)
	double MIN_distance = 9999999;
	int best_node_index_point1;
	for (int x = 0; x < graph.size(); x++){

		if (point_1.Distance(graph[x].vertex) < MIN_distance){
			C2DLine testline(point_1, graph[x].vertex);
			if (!line_is_valid(testline))
				continue;
			MIN_distance = point_1.Distance(graph[x].vertex);
			best_node_index_point1 = x;
		}
	}

	C2DLine first_line(point_1, graph[best_node_index_point1].vertex);

	first_node = &graph[best_node_index_point1];

	//next,find the nearest node to point2, that also generates a valid line
	MIN_distance = 9999999;
	int best_node_index_point2;
	for (int x = 0; x < graph.size(); x++){

		if (point_2.Distance(graph[x].vertex) < MIN_distance){
			C2DLine testline(point_2, graph[x].vertex);
			if (!line_is_valid(testline))
				continue;
			MIN_distance = point_2.Distance(graph[x].vertex);
			best_node_index_point2 = x;
		}
	}
	C2DLine last_line(point_2, graph[best_node_index_point2].vertex);

	last_node = &graph[best_node_index_point2];

	//special case:::
	if (first_node->vertex == last_node->vertex || point_1.Distance(point_2) <= 50){
		//points are so close together,there is no need to transverse voronoi graph (path will be a single straight line)
		//exception:that small line passes through an obstacle
		C2DLine path(point_1, point_2);
		if (line_is_valid(path)){
			m_canvas_pp.add(path, vvr::Colour::magenta);
			cout << endl << "++Path created++" << endl;
			cout << "Distance is very small,the safest and the best paths are the same" << endl;
			return;
		}
	}

	/////////////////////////////////////////////////////////////////////////

	//find the rest of the path via the graph (starting from first_node,trying to find path to last_node)
	path_is_impossible = true;
	std::vector<graph_node> node_path;
	graph[best_node_index_point1].visited = true;
	std::vector<double> order_of_preference; //based on their distance to the last node
	for (int x = 0; x < first_node->neighbors.size(); x++){

		double d = first_node->neighbors[x]->vertex.Distance(last_node->vertex);
		order_of_preference.push_back(d);
	}

	std::sort(order_of_preference.begin(), order_of_preference.end());


	for (int x = 0; x < order_of_preference.size(); x++){


		double distance = order_of_preference[x];
		int neighbor_index = index_of_corresponding_distance(first_node, distance, last_node);
		for (int x = 0; x < graph.size(); x++){
			if (x == best_node_index_point1)
				continue;
			graph[x].visited = false;

		}
		path_is_impossible = !neighbor_path_leads_to_target(first_node->neighbors[neighbor_index], node_path, last_node,1);
		if (!path_is_impossible)
			break;

	}

	if (path_is_impossible)
		cout <<endl<< "**This path cannot be created.**" << endl;
	else{

		node_path.push_back(*first_node);
		m_canvas_pp.add(first_line, vvr::Colour::magenta);
		m_canvas_pp.add(last_line, vvr::Colour::magenta);
		shortest_safest_path.push_back(first_line);
		shortest_safest_path.push_back(last_line);
		for (int x = 0; x < node_path.size()-1; x++){

			C2DLine line_to_add(node_path[x].vertex, node_path[x + 1].vertex);
			m_canvas_pp.add(line_to_add, vvr::Colour::magenta);
			shortest_safest_path.push_back(line_to_add);
		}


	}

}


//Optimal path algorithm.
void SceneRobotEnvironment::find_optimal_path(C2DPoint point_1, C2DPoint point_2){

	std::vector<std::vector<C2DLine>> possible_optimal_paths;
	optimal_path.clear();
	obs_encountered.clear();

	//first try the obvious solution:a straight line between the points
	C2DLine simple_solution(point_1, point_2);
	C2DPoint intersection_pt;
	int ob_line_index;
	int obstacle_index;
	bool no_obstacle_in_the_way = no_obstacles_hit(simple_solution,&intersection_pt,&obstacle_index,&ob_line_index);
	if (no_obstacle_in_the_way){
		m_canvas_pp.add(simple_solution, vvr::Colour::cyan);
		optimal_path.push_back(simple_solution);
		return;
	}


	else{

		//if the simple solution is not possible :
		C2DLine starting_line = simple_solution;
		starting_line.SetLength(1);
		std::vector<C2DLine> first_path;
		first_path.push_back(starting_line);
		possible_optimal_paths.push_back(first_path);
		int depth = 1;
		obs_enc = 0;
		find_optimal_recursive(depth,point_2,0,possible_optimal_paths,"grow");


			std::vector<double> total_distances;
			double current_tot_dis;

			//delete erroneous paths that may appear due to the approximate nature of intersection checks.
			for (int w = 0; w < possible_optimal_paths.size(); w++){


				for (int z = 0; z < possible_optimal_paths[w].size(); z++){

					if (no_of_obs_crossed(possible_optimal_paths[w][z]) >= 3 || line_crosses_real_ob(possible_optimal_paths[w][z]) || line_outside_bounds(possible_optimal_paths[w][z])){

						possible_optimal_paths.erase(possible_optimal_paths.begin() + w);
						w--;
						break;
						}

					}

				}



			//compare total distance for every path. The path with the least total distance is chosen.
			if (possible_optimal_paths.size() > 0){
				for (int w = 0; w < possible_optimal_paths.size(); w++){

					current_tot_dis = 0;
					for (int z = 0; z < possible_optimal_paths[w].size(); z++){

						current_tot_dis += possible_optimal_paths[w][z].GetLength();

					}
					total_distances.push_back(current_tot_dis);
				}


				int index_of_min;
				double MIN = 99999999;
				for (int w = 0; w < total_distances.size(); w++){

					if (total_distances[w] < simple_solution.GetLength())
						continue;
					if (total_distances[w] < MIN){
						MIN = total_distances[w];
						index_of_min = w;
					}
				}



				std::vector<C2DLine> chosen_path;
				if (index_of_min > 10000 || index_of_min < 0)
					index_of_min = 0;

				chosen_path = possible_optimal_paths[index_of_min];
				


				for (int w = 0; w < chosen_path.size(); w++) {

					m_canvas_pp.add(chosen_path[w], vvr::Colour::cyan);
					optimal_path.push_back(chosen_path[w]);
				}

			}





	}

}

//Recursive increase of path length.
//two modes : 'grow' -> increases in size in the direction of the destinations
//'obstacle' -> transverses through an obstacle boundary until it can 'escape'.
//dir id = 0 :clockwise,dir id = 1 :counterclockwise
void SceneRobotEnvironment::find_optimal_recursive(int depth, C2DPoint destination, int index, std::vector<std::vector<C2DLine>>& paths, string mode, int dir_id, int obs_index, int obs_line_index){


	if (depth >= 2000){
		emerg_termination = true;
		return;
	}
	if (mode == "grow"){

		int index_of_last_line = paths[index].size()-1;

		C2DLine last_line = paths[index][index_of_last_line];
		C2DLine new_line(last_line.GetPointTo(), destination);
		new_line.SetLength(2);
		if (points_are_identical(new_line.GetPointTo(), destination, 2)){
			new_line.SetPointTo(destination);
			paths[index].push_back(new_line);

			return;
		}
		C2DPoint intersection_pt;
		int ob_line_index;
		int obstacle_index;
		bool obstacle_found = !no_obstacles_hit(new_line, &intersection_pt, &obstacle_index, &ob_line_index);
		if (obstacle_found){
			if (obstacle_encountered(obstacle_index)){
				new_line = C2DLine(last_line.GetPointTo(), intersection_pt);
				paths[index].push_back(new_line);
				depth++;
				find_optimal_recursive(depth, destination, index, paths, "obstacle", dir_id, obstacle_index, ob_line_index);
				return;
			}
			else{
				std::vector<C2DLine> new_possible_path;
				new_line = C2DLine(last_line.GetPointTo(), intersection_pt);
				paths[index].push_back(new_line);
				for (int w = 0; w < paths[index].size(); w++)
					new_possible_path.push_back(paths[index][w]);
				paths.push_back(new_possible_path);
				int newpathind = paths.size() - 1;
				;
				depth++;
				obs_encountered.push_back(obstacle_index);

				obs_enc++;
				find_optimal_recursive(depth, destination, index, paths, "obstacle", 0, obstacle_index, ob_line_index);



				depth++;
				find_optimal_recursive(depth, destination, newpathind , paths, "obstacle", 1, obstacle_index, ob_line_index);
				return;
			}
		}
		else{

			paths[index].push_back(new_line);
			depth++;
			find_optimal_recursive(depth, destination, index, paths, "grow", dir_id);
			return;
		}
	}
	else if (mode == "obstacle"){

			//first check if a straight line can be resumed
			int index_of_last_line = paths[index].size()-1;
			C2DLine last_line = paths[index][index_of_last_line];
			C2DLine possible_new_line(last_line.GetPointTo(), destination);

			if (doesnt_hit_current_ob(possible_new_line, modified_polygon_obstacles_union[obs_index])){

				int index_of_last_line = paths[index].size()-1;
				C2DLine last_line = paths[index][index_of_last_line];
				C2DLine new_line(last_line.GetPointTo(), destination);
				new_line.SetLength(2);

				paths[index].push_back(new_line);
				depth++;
				find_optimal_recursive(depth,destination, index, paths, "grow",dir_id);
				return;
			}

			else{
				C2DPoint new_line_endpoint;
				if (dir_id == 0)
					new_line_endpoint = modified_polygon_obstacles_union[obs_index].GetLine(obs_line_index)->GetPointTo();
				else
					new_line_endpoint = modified_polygon_obstacles_union[obs_index].GetLine(obs_line_index)->GetPointFrom();

				C2DLine new_line(last_line.GetPointTo(), new_line_endpoint);

				paths[index].push_back(new_line);
				if (dir_id == 0){ //clockwise transversion
					int new_line_index = obs_line_index + 1;
					if (new_line_index >= modified_polygon_obstacles_union[obs_index].GetLineCount()){
						new_line_index = 0;

					}
					depth++;
					find_optimal_recursive(depth,destination, index, paths, "obstacle",dir_id, obs_index, new_line_index);
					return;
				}
				else{ //counterclockwise transversion
					int new_line_index = obs_line_index - 1;
					if (new_line_index < 0){
						new_line_index = modified_polygon_obstacles_union[obs_index].GetLineCount() - 1;

					}
					depth++;
					find_optimal_recursive(depth,destination, index, paths, "obstacle",dir_id, obs_index, new_line_index);
					return;
				}
			}
		}
}


bool SceneRobotEnvironment::obstacle_encountered(int ob_index){

	for (int w = 0; w < obs_encountered.size(); w++){

		if (ob_index == obs_encountered[w])
			return true;

	}
	return false;
}

//uses the union of the modified(minkowski) obstacles.
bool SceneRobotEnvironment::no_obstacles_hit(C2DLine test_line,C2DPoint* intersection_point,int* obs_index,int* obs_line_index){

	C2DPointSet temp;
	bool noobstacles_hit = true;
	int obstacle_index,obstacle_line_index;
	for (int w = 0; w < modified_polygon_obstacles_union.size(); w++){

		if (modified_polygon_obstacles_union[w].Crosses(test_line) || modified_polygon_obstacles_union[w].Contains(test_line)){
			*obs_index = w;

			obstacle_index = w;
			noobstacles_hit = false;

			break;
		}
	}
	if (!noobstacles_hit){
		for (int w = 0; w < modified_polygon_obstacles_union[obstacle_index].GetLineCount(); w++){

			if (modified_polygon_obstacles_union[obstacle_index].GetLine(w)->Crosses(test_line)){
				*obs_line_index = w;
				obstacle_line_index = w;


			}
		}
	}

	if (!noobstacles_hit){
		find_lines_intersection(test_line, *modified_polygon_obstacles_union[obstacle_index].GetLine(obstacle_line_index), *intersection_point);

	}


	return noobstacles_hit;

}

bool SceneRobotEnvironment::line_crosses_real_ob(C2DLine l){

	for (int w = 0; w < polygon_obstacles.size(); w++){
		if (m_robot.robot_type == line){
			if (polygon_obstacles[w].Contains(l)){
				return true;
			}
		}
		else{
			if (polygon_obstacles[w].Contains(l) || polygon_obstacles[w].Crosses(l)){
				return true;
			}

		}
	}

	for (int w = 0; w < line_obstacles.size(); w++){

		if (line_obstacles[w].Crosses(l)){
			return true;
		}
	}

	return false;


}

bool SceneRobotEnvironment::line_outside_bounds(C2DLine l){
	if (!rotational_points_placed){
		if (!modified_boundary.Contains(l))
			return true;
		else
			return false;
	}
	else
	{
		bool result = true;
		for (int x = 0; x < minkw_resulting_bounds.size(); x++){
			if (minkw_resulting_bounds[x].Contains(l))
				result = false;
		}
		return result;
	}

}

bool SceneRobotEnvironment::doesnt_hit_current_ob(C2DLine test_line, C2DPolygon obstacle){



	C2DLine test_line2;
	test_line2 = test_line;
	test_line2.SetLength(1);
	test_line2.SetPointFrom(test_line2.GetPointTo());
	C2DLine test_line3 = test_line2;
	test_line3.SetLength(1);
	test_line2.SetPointTo(test_line.GetPointTo());
	return (!line_intersects_with_polygon(test_line2, obstacle) && !obstacle.Contains(test_line3));







}

bool SceneRobotEnvironment::neighbor_path_leads_to_target(graph_node* neighbor,std::vector<graph_node>& path,graph_node* target,int depth){

	neighbor->visited = true;


	//first determine if target is reached
	if (points_are_identical(neighbor->vertex,target->vertex,2)){
		path.push_back(*neighbor);
		return true;
	}

	//next,check if this is a dead end or a loop (all its neighbors are in the current path)
	bool deadend = true;
	for (int x = 0; x < neighbor->neighbors.size(); x++) {

		if (neighbor->neighbors[x]->visited==false)
			deadend = false;
	}

	if (deadend){
		return false;
	}

	std::vector<double> order_of_preference;

	for (int x = 0; x < neighbor->neighbors.size(); x++){

		if (neighbor->neighbors[x]->visited)
			continue;
		double d = neighbor->neighbors[x]->vertex.Distance(target->vertex);
		order_of_preference.push_back(d);
	}

	std::sort(order_of_preference.begin(), order_of_preference.end());

	bool no_path_exists;

	for (int x = 0; x < order_of_preference.size(); x++){


		double distance = order_of_preference[x];
		int neighbor_index = index_of_corresponding_distance(neighbor, distance, target);
		no_path_exists = !neighbor_path_leads_to_target(neighbor->neighbors[neighbor_index], path, target,depth+1);
		if (!no_path_exists)
			break;

	}

	if (no_path_exists){
		return false;
	}


	else{
		path.push_back(*neighbor);
		return true;
	}


}

//For a given distance with a node,finds the index of that node.
int SceneRobotEnvironment::index_of_corresponding_distance(graph_node* starting_node, double distance, graph_node* destination_node){

	for (int x = 0; x < starting_node->neighbors.size(); x++){

		if (distance == starting_node->neighbors[x]->vertex.Distance(destination_node->vertex))
			return x;
	}


}

bool SceneRobotEnvironment::line_is_in_freespace(C2DLine line){

	bool result = false;

	for (int w = 0; w < free_space.size(); w++){

		if (free_space[w].Contains(line)){
			result = true;
			break;
		}
	}

	return result;


}

bool SceneRobotEnvironment::line_is_valid(C2DLine l){

	bool lineisvalid = true;

	for (int x = 0; x < modified_polygon_obstacles.size(); x++){

		if (modified_polygon_obstacles[x].Contains(l))
			lineisvalid = false;
	}

	return lineisvalid;
}

int SceneRobotEnvironment::no_of_obs_crossed(C2DLine l){

	int counter = 0;

	for (int x = 0; x < modified_polygon_obstacles.size(); x++){

		if (modified_polygon_obstacles[x].Contains(l)){
			counter++;
			continue;
		}

	}
	
	return counter;


}

bool SceneRobotEnvironment::line_is_valid(C2DPoint p1,C2DPoint p2){

	bool lineisvalid = false;

	C2DLine l(p1, p2);

	for (int x = 0; x < free_space.size(); x++){

		if (free_space[x].Contains(l))
			lineisvalid = true;
	}

	return lineisvalid;
}

bool SceneRobotEnvironment::valid_point_selected(C2DPoint p){

	bool validpointselected = false;

	bool inside_free_space = false;
	bool not_inside_modified_obs = true;

	for(int x=0;x<free_space.size();x++){

		if(free_space[x].Contains(p))
			inside_free_space = true;
	}

	for (int x = 0; x < modified_polygon_obstacles.size(); x++){

		if (modified_polygon_obstacles[x].Contains(p))
			not_inside_modified_obs = false;
	}

	validpointselected = inside_free_space && not_inside_modified_obs;
	return validpointselected;

}


//Path planning algorithm for combined motion.
void SceneRobotEnvironment::rotational_pp(C2DPoint starting_point,C2DPoint ending_point,double starting_angle, double ending_angle){

	minkw_union_freespace.clear();
	minkw_resulting_obs.clear();
	current_map_obs_union.clear();
	bool path_is_impossible;
	m_canvas_robot.clear();

	C2DLine templine;

	m_robot.orientation = C2DVector(1, 0);

	ChangeRobotOrientation('s',starting_angle);
	CalculateMinkowski();

	maps_obstacles[0] = modified_polygon_obstacles;
	maps_free_space[0] = free_space;
	map_directions[0] = m_robot.orientation;
	maps_boundaries[0] = modified_boundary;
	maps_obs_unions[0] = current_map_obs_union;

	if (!is_inside_space(starting_point,maps_obstacles[0],maps_free_space[0],templine) ){
		m_canvas_pp.clear();
		cout << endl << "*Path is impossible : Robot cannot have this starting angle with these dimensions,in the designated position*" << endl;
		freespacepoints.DeleteAll();
		cell_lines.clear();
		ShowRobot_in_env(starting_point,true);
		return;
	}

	ChangeRobotOrientation('s', ending_angle);
	CalculateMinkowski();

	maps_obstacles[no_of_maps_calculated -1] = modified_polygon_obstacles;
	maps_free_space[no_of_maps_calculated - 1] = free_space;
	map_directions[no_of_maps_calculated - 1] = m_robot.orientation;
	maps_boundaries[no_of_maps_calculated - 1] = modified_boundary;
	maps_obs_unions[no_of_maps_calculated - 1] = current_map_obs_union;

	if (!is_inside_space(ending_point,maps_obstacles[no_of_maps_calculated-1],maps_free_space[no_of_maps_calculated-1],templine)){
		m_canvas_pp.clear();
		cout << endl << "*Path is impossible : Robot cannot have this ending angle with these dimensions,in the designated position*" << endl;
		freespacepoints.DeleteAll();
		cell_lines.clear();
		ShowRobot_in_env(ending_point,true);
		return;
	}

	compute_discrete_minkw();
	minkowski_union();
	draw_total_minkw();

	modified_polygon_obstacles = minkw_resulting_obs;
	modified_polygon_obstacles_union = minkw_resulting_obs;
	free_space = minkw_union_freespace;

	C2DVector final_point_orientation = C2DVector(1, 0);
	final_point_orientation.TurnLeft(ending_angle);

	Generalized_Voronoi();
	find_safest_path(starting_point, ending_point, path_is_impossible);
	find_optimal_path(starting_point, ending_point);
	int errorInPath = 0;
	if (shortest_safest_path.size() != 0)
		errorInPath = find_path_orientations(shortest_safest_path);
	if (optimal_path.size() != 0){

		errorInPath = find_path_orientations(optimal_path);
	}

	cout << endl << "Note : robot dimensions in path have been scaled down for better visibility." << endl;
	if (errorInPath == 1)
	{
		cout << endl << "*** There is a point in the path where a suitable orientation";
		cout << "could not be found from the calculated maps. An approximation has been used instead.";
		cout << "The approximation is not 100% guaranteed to be correct, but in most cases it is.";
		cout << "This may occur because not enough maps are calculated .";
		cout << "In that case,increasing the number of maps calculated from the 2nd slider will resolve this. ";
		cout << "The point on which the approximation has been used is colored in dark green. ***" << endl << endl;
	}
	ChangeRobotOrientation('s', ending_angle);
	ShowRobot_in_env(ending_point, false, true,true,false);
}



//for a number of angles,calculates the free space.
void SceneRobotEnvironment::compute_discrete_minkw(){

	double angle_increment = 180.0 / (no_of_maps_calculated - 1.0);

	C2DVector sample_vector(1, 0);
	map_directions[1] = sample_vector;
	m_robot.orientation = sample_vector;
	CalculateMinkowski();
	maps_obstacles[1] = modified_polygon_obstacles;
	maps_free_space[1] = free_space;
	maps_boundaries[1] = modified_boundary;
	maps_obs_unions[1] = current_map_obs_union;


	for (int w = 2; w < no_of_maps_calculated - 1; w++){

		sample_vector.TurnLeft(angle_increment);
		map_directions[w] = sample_vector;
		m_robot.orientation = sample_vector;
		CalculateMinkowski();
		maps_obstacles[w] = modified_polygon_obstacles;
		maps_free_space[w] = free_space;
		maps_boundaries[w] = modified_boundary;
		maps_obs_unions[w] = current_map_obs_union;

	}

}


//Used in combined motion path planning;for every fundamental line in the path,
//checks the two extreme points and the midpoint,for necessary orientation changes.
//If 1 is returned,it means that there has been an internal error in the calculation of the path.
int SceneRobotEnvironment::find_path_orientations(std::vector<C2DLine> path){

	int returnvalue = 0;
	bool no_maps_suitable = false;

	ChangeRobotOrientation('s', starting_angle);
	C2DVector current_dir = m_robot.orientation;
	ShowRobot_in_env(path[0].GetPointFrom(), false,true);
	check_point_orientation(path[0].GetMidPoint(), current_dir, no_maps_suitable);
	if (no_maps_suitable)
		returnvalue = 1;
	check_point_orientation(path[0].GetPointTo(), current_dir, no_maps_suitable);
	if (no_maps_suitable)
		returnvalue = 1;


	for (int w = 1; w < path.size() - 1; w++){
		check_point_orientation(path[w].GetPointFrom(), current_dir, no_maps_suitable);
		if (no_maps_suitable)
			returnvalue = 1;
		check_point_orientation(path[w].GetMidPoint(), current_dir, no_maps_suitable);
		if (no_maps_suitable)
			returnvalue = 1;
		check_point_orientation(path[w].GetPointTo(), current_dir, no_maps_suitable);
		if (no_maps_suitable)
			returnvalue = 1;
	}

	check_point_orientation(path[path.size() - 1].GetPointFrom(), current_dir, no_maps_suitable);
	if (no_maps_suitable)
		returnvalue = 1;
	check_point_orientation(path[path.size() - 1].GetMidPoint(), current_dir, no_maps_suitable);
	if (no_maps_suitable)
		returnvalue = 1;

	return returnvalue;

	}


//finds the best (in terms of angle difference from the current) orientation for robot to have in a certain point,for combined motion. 
//however if the current orientation of the robot is viable, it is not changed.
void SceneRobotEnvironment::check_point_orientation(C2DPoint point,C2DVector& current_dir,bool& no_maps_suitable){


	std::vector<C2DVector> suitable_dirs;
	suitable_dirs.clear();
	C2DLine obstacle_line_intersected; 

	for (int w = 0; w < no_of_maps_calculated; w++){

		if (is_inside_space(point,maps_obstacles[w], maps_free_space[w],obstacle_line_intersected,map_directions[w],true))
			suitable_dirs.push_back(map_directions[w]);

	}

	for (int w = 0; w < suitable_dirs.size(); w++){

		if (suitable_dirs[w] == current_dir){
			m_robot.orientation = current_dir;
			return;
		}
	}

	//if there are no suitable directions,find the orientation of the obstacle line and use that.
	if (suitable_dirs.size() == 0){
		C2DVector new_approx_orientation = C2DVector(obstacle_line_intersected.GetPointFrom(), obstacle_line_intersected.GetPointTo());
		current_dir = new_approx_orientation;
		no_maps_suitable = true;
		ShowRobot_in_env(point, false, true, true, true);
	}

	//if there ARE suitable directions , find the one which requires the smallest change of angle
	//from the current one.
	else{
		no_maps_suitable = false;
		C2DVector new_dir;
		find_smallest_angle_change(current_dir,suitable_dirs,new_dir);

		current_dir = new_dir;
		m_robot.orientation = current_dir;
		ShowRobot_in_env(point, false,true,true);
	}

}

//from a set of suitable angles,chooses the one which requires the smallest change from the current angle.
void SceneRobotEnvironment::find_smallest_angle_change(C2DVector current,std::vector<C2DVector> suitable_vecs,C2DVector& new_dir){

	std::vector<double> angle_diffs;
	double dot_product,angle_difference;

	for(int w=0;w<suitable_vecs.size();w++){
			dot_product = dotproduct(current,suitable_vecs[w]);
			angle_difference = acos(dot_product);
			angle_diffs.push_back(angle_difference);
	}
	int index_of_min = 0;
	double MIN = 99999;
	for(int w=0;w<angle_diffs.size();w++){
		if(angle_diffs[w]<MIN){
			index_of_min = w;
			MIN = angle_diffs[w];
		}
	}
	new_dir = suitable_vecs[index_of_min];
}

//obstacles: finds the intersection of the union of each map.
//boundaries: finds the intersection of the boundaries of all the maps.
//resulting free space: the (boolean) difference between the intersection of the map unions and the intersection of the boundaries.
//this free space represents the best case scenario of all the discrete orientations of the robot that we have checked.
void SceneRobotEnvironment::minkowski_union(){

	Paths clipper_obstacle_intersections;
	Paths clipper_boundaries_intersections;
	Paths resulting_freespace_clipper;

	Paths* clipper_obstacle_unions = new Paths[no_of_maps_calculated];
	for (int w = 0; w < no_of_maps_calculated; w++){

		for (int z = 0; z < maps_obs_unions[w].size(); z++){

			Path temp;
			geolib_2_clipper(maps_obs_unions[w][z], temp);
			clipper_obstacle_unions[w].push_back(temp);

		}
	}

	Clipper c;
	Paths temp_result;
	c.AddPaths(clipper_obstacle_unions[0],ptSubject, true);
	c.AddPaths(clipper_obstacle_unions[1], ptClip, true);
	c.Execute(ctIntersection, temp_result);
	if (temp_result.size() == 0){
		c.AddPaths(clipper_obstacle_unions[0], ptClip, true);
		c.AddPaths(clipper_obstacle_unions[1], ptSubject, true);
		c.Execute(ctIntersection, temp_result);
		clipper_obstacle_intersections = temp_result;
	}
	else
		clipper_obstacle_intersections = temp_result;

	//intersection of obstacle unions.
	for (int w = 2; w < no_of_maps_calculated; w++){

		Clipper c;
		Paths temp_result;
		c.AddPaths(clipper_obstacle_unions[w], ptSubject, true);
		c.AddPaths(clipper_obstacle_intersections, ptClip, true);
		c.Execute(ctIntersection, temp_result);
		if (temp_result.size() == 0){
			c.AddPaths(clipper_obstacle_unions[w], ptClip, true);
			c.AddPaths(clipper_obstacle_intersections, ptSubject, true);
			c.Execute(ctIntersection, temp_result);
			if (temp_result.size() != 0)
				clipper_obstacle_intersections = temp_result;
			else
				continue;
		}
		else
			clipper_obstacle_intersections = temp_result;
	}

	///////////////////////////////////////////////
	Paths clipper_boundaries;
	for (int w = 0; w < no_of_maps_calculated; w++){

		Path temp;
		geolib_2_clipper(maps_boundaries[w], temp);
		clipper_boundaries.push_back(temp);
	}

	//intersection of boundaries.
	Clipper d;
	Paths temp_result2;
	d.AddPath(clipper_boundaries[0],ptSubject, true);
	d.AddPath(clipper_boundaries[1], ptClip, true);
	d.Execute(ctIntersection, temp_result2);
	if (temp_result2.size() == 0){
		d.AddPath(clipper_boundaries[0], ptClip, true);
		d.AddPath(clipper_boundaries[1], ptSubject, true);
		d.Execute(ctIntersection, temp_result2);
		clipper_boundaries_intersections = temp_result2;

	}
	else
		clipper_boundaries_intersections = temp_result2;

	for (int w = 2; w < no_of_maps_calculated; w++){

		Clipper c;
		Paths temp_result;
		c.AddPath(clipper_boundaries[w], ptSubject, true);
		c.AddPaths(clipper_boundaries_intersections, ptClip, true);
		c.Execute(ctIntersection, temp_result);
		if (temp_result.size() == 0){
			c.AddPath(clipper_boundaries[w], ptClip, true);
			c.AddPaths(clipper_obstacle_intersections, ptSubject, true);
			c.Execute(ctIntersection, temp_result);
			if (temp_result.size() != 0)
				clipper_boundaries_intersections = temp_result;
			else
				continue;
		}
		else
			clipper_boundaries_intersections = temp_result;
	}

	//find resulting free space
	Clipper e;
	e.AddPath(clipper_obstacle_intersections[0], ptClip, true);
	e.AddPaths(clipper_boundaries_intersections, ptSubject, true);
	e.Execute(ctDifference, resulting_freespace_clipper);
	for (int w = 1; w < clipper_obstacle_intersections.size(); w++){
		Clipper e;
		e.AddPath(clipper_obstacle_intersections[w], ptClip, true);
		e.AddPaths(resulting_freespace_clipper, ptSubject, true);
		e.Execute(ctDifference, resulting_freespace_clipper);
	}


	for (int w = 0; w < clipper_obstacle_intersections.size(); w++){
		C2DPolygon temp;
		clipper_2_geolib(clipper_obstacle_intersections[w], temp);
		minkw_resulting_obs.push_back(temp);
	}
	for (int w = 0; w < resulting_freespace_clipper.size(); w++){

		C2DPolygon temp;
		clipper_2_geolib(resulting_freespace_clipper[w], temp);
		minkw_union_freespace.push_back(temp);

	}
	///
	for (int w = 0; w < clipper_boundaries_intersections.size(); w++){

		C2DPolygon temp;
		clipper_2_geolib(clipper_boundaries_intersections[w], temp);
		minkw_resulting_bounds.push_back(temp);

	}

	delete[] clipper_obstacle_unions;


}

void SceneRobotEnvironment::approx_minkw_for_lineobs(C2DLine line_ob) {


	C2DPointSet resulting_minkw;

	C2DVector v(line_ob.GetPointFrom(), line_ob.GetPointTo());
	C2DVector r_v(line_ob.GetPointTo(), line_ob.GetPointFrom());

	C2DLine s_l(line_ob.GetPointTo(), v);
	s_l.SetLength(1);
	C2DLine r_s_l(line_ob.GetPointFrom(), r_v);
	r_s_l.SetLength(1);

	v.TurnLeft();
	C2DLine r_l_u(s_l.GetPointTo(), v);
	C2DLine l_l_u(r_s_l.GetPointTo(), v);
	r_l_u.SetLength(1);
	l_l_u.SetLength(1);
	resulting_minkw.AddCopy(r_l_u.GetPointTo());
	resulting_minkw.AddCopy(l_l_u.GetPointTo());

	v.TurnRight();
	v.TurnRight();

	C2DLine r_l_d(s_l.GetPointTo(), v);
	C2DLine l_l_d(r_s_l.GetPointTo(), v);
	r_l_d.SetLength(1);
	l_l_d.SetLength(1);
	resulting_minkw.AddCopy(r_l_d.GetPointTo());
	resulting_minkw.AddCopy(l_l_d.GetPointTo());

	C2DPolygon resulting_minkw_poly = C2DPolygon(resulting_minkw, true);
	modified_polygon_obstacles.push_back(resulting_minkw_poly);


}

void::SceneRobotEnvironment::draw_total_minkw(){


	for (int w = 0; w < minkw_union_freespace.size(); w++){

		vvr::draw(minkw_union_freespace[w], vvr::Colour::orange);
	}

}

//Draws the robot in a chosen position on the path. For combined motion mode only.
void SceneRobotEnvironment::ShowRobot_in_env(C2DPoint robot_center_pos,bool error_display,bool small_display,bool draw_center,bool colored){

	double robot_width, robot_len;

	if (small_display){
		robot_width = m_robot.width;
		robot_len = m_robot.len;
		if (m_robot.robot_type == line)
			m_robot.len = m_robot.len / 2.0;
		else if (m_robot.robot_type == rectangle){
			m_robot.len = m_robot.len / 2.0;
			m_robot.width = m_robot.width / 2.0;
		}
	}
	if (m_robot.robot_type == line){

		C2DLine line_to_top(robot_center_pos, m_robot.orientation);
		C2DVector opposite_dir;
		opposite_dir.i = -m_robot.orientation.i;
		opposite_dir.j = -m_robot.orientation.j;
		C2DLine line_to_bottom(robot_center_pos, opposite_dir);

		line_to_top.SetLength(m_robot.len / 2.0);
		line_to_bottom.SetLength(m_robot.len / 2.0);

		C2DLine line_robot(line_to_bottom.GetPointTo(), line_to_top.GetPointTo());

		if (error_display)
			m_canvas_robot.add(line_robot, vvr::Colour::red);
		else {
			if (colored)
				m_canvas_robot.add(line_robot, vvr::Colour::darkGreen);
			else
				m_canvas_robot.add(line_robot, vvr::Colour::blue);
			if (draw_center)
				m_canvas_robot.add(robot_center_pos, vvr::Colour::blue);
			
		}

	}

	if (m_robot.robot_type == rectangle){

		C2DLine rec_top, rec_bottom, rec_right, rec_left;
		C2DPoint toprightcorner, topleftcorner, bottomrightcorner, bottomleftcorner;

		C2DPoint rectangle_center = robot_center_pos;

		C2DLine line_to_top(rectangle_center, m_robot.orientation);
		line_to_top.SetLength(m_robot.len / 2.0);

		C2DVector temp_orientation = m_robot.orientation;
		temp_orientation.TurnRight();
		C2DLine line_to_topright_corner(line_to_top.GetPointTo(), temp_orientation);
		line_to_topright_corner.SetLength(m_robot.width / 2.0);
		toprightcorner = line_to_topright_corner.GetPointTo();

		temp_orientation = m_robot.orientation;
		temp_orientation.TurnLeft();
		C2DLine line_to_topleft_corner(line_to_top.GetPointTo(), temp_orientation);
		line_to_topleft_corner.SetLength(m_robot.width / 2.0);
		topleftcorner = line_to_topleft_corner.GetPointTo();

		C2DVector opposite_orientation(-m_robot.orientation.i, -m_robot.orientation.j);
		C2DLine line_to_bottom(rectangle_center, opposite_orientation);
		line_to_bottom.SetLength(m_robot.len / 2.0);

		temp_orientation = opposite_orientation;
		temp_orientation.TurnRight();
		C2DLine line_to_bottomleft_corner(line_to_bottom.GetPointTo(), temp_orientation);
		line_to_bottomleft_corner.SetLength(m_robot.width / 2.0);
		bottomleftcorner = line_to_bottomleft_corner.GetPointTo();

		temp_orientation = opposite_orientation;
		temp_orientation.TurnLeft();
		C2DLine line_to_bottomright_corner(line_to_bottom.GetPointTo(), temp_orientation);
		line_to_bottomright_corner.SetLength(m_robot.width / 2.0);
		bottomrightcorner = line_to_bottomright_corner.GetPointTo();

		rec_top = C2DLine(topleftcorner, toprightcorner);
		rec_right = C2DLine(toprightcorner, bottomrightcorner);
		rec_bottom = C2DLine(bottomrightcorner, bottomleftcorner);
		rec_left = C2DLine(bottomleftcorner, topleftcorner);

		if (error_display){
			m_canvas_robot.add(rec_top, vvr::Colour::red);
			m_canvas_robot.add(rec_bottom, vvr::Colour::red);
			m_canvas_robot.add(rec_right, vvr::Colour::red);
			m_canvas_robot.add(rec_left, vvr::Colour::red);
		}
		else{	
			if (colored){
				m_canvas_robot.add(rec_top, vvr::Colour::darkGreen);
				m_canvas_robot.add(rec_bottom, vvr::Colour::darkGreen);
				m_canvas_robot.add(rec_right, vvr::Colour::darkGreen);
				m_canvas_robot.add(rec_left, vvr::Colour::darkGreen);
			}
			else{
				m_canvas_robot.add(rec_top, vvr::Colour::blue);
				m_canvas_robot.add(rec_bottom, vvr::Colour::blue);
				m_canvas_robot.add(rec_right, vvr::Colour::blue);
				m_canvas_robot.add(rec_left, vvr::Colour::blue);
			}
				if (draw_center)
					m_canvas_robot.add(robot_center_pos, vvr::Colour::blue);
			}

		}
	if (small_display){
		m_robot.width = robot_width;
		m_robot.len = robot_len;
	}
	m_canvas_robot.draw();
	}


bool SceneRobotEnvironment::valid_point_selected_rotational(C2DPoint point){

	bool validpointselected = true;
	if (rectangular_boundary){

		C2DPolygon rec_boundary;
		lines_to_polygon(boundary_lines, rec_boundary);

		if (!rec_boundary.Contains(point))
			validpointselected = false;
	}

	else{
		if (!boundary.Contains(point))
			validpointselected = false;
	}
	for (int w = 0; w < polygon_obstacles.size(); w++){
		if (polygon_obstacles[w].Contains(point))
				validpointselected = false;
		}
	for (int w = 0; w < line_obstacles.size(); w++){
		if (point_is_on_line(point, line_obstacles[w]))
				validpointselected = false;
		}

	return validpointselected;

}

//check_dir : true -> check if robot is inside freespace.
//check_dir : false -> check if point is inside freespace.
bool SceneRobotEnvironment::is_inside_space(C2DPoint point, std::vector<C2DPolygon> obstacles, std::vector<C2DPolygon> freespace, C2DLine& intersectingObLine,C2DVector dir, bool check_dir){

	bool isinsidespace;

	//first check if current point is part of an obstacle boundary
	C2DLine selected_line;
	bool is_part_of_ob_boundary = false;
	for(int w=0;w<obstacles.size();w++){

		if(point_is_on_line(point,*obstacles[w].GetLine(w))){
			selected_line = *obstacles[w].GetLine(w);
			is_part_of_ob_boundary = true;
			break;
		}
	}

	//check_dir == true (check robot with dimensions) 
	if(check_dir && is_part_of_ob_boundary){

		if(m_robot.robot_type == line){
			C2DPoint center = point;
			C2DLine line_1(center,dir);
			line_1.SetLength(m_robot.len);
			C2DVector opp_dir;
			opp_dir.i = -dir.i;
			opp_dir.j = -dir.j;
			C2DLine line_2(center,opp_dir);
			line_2.SetLength(m_robot.len);
			for(int w=0;w<polygon_obstacles.size();w++){
				if (polygon_obstacles[w].Crosses(line_1) || polygon_obstacles[w].Crosses(line_2)){
					if (polygon_obstacles[w].Crosses(line_1))
						find_intersecting_line(polygon_obstacles[w], line_1, intersectingObLine);
					else
						find_intersecting_line(polygon_obstacles[w], line_2, intersectingObLine);
					return false;
				}


			}
			for(int w=0;w<line_obstacles.size();w++){
				if (line_obstacles[w].Crosses(line_1) || line_obstacles[w].Crosses(line_2)) {
					intersectingObLine = line_obstacles[w];
					return false;
				}
			}
		}

		else if(m_robot.robot_type == rectangle){
			C2DLine rec_top, rec_bottom, rec_right, rec_left;
			C2DPoint toprightcorner, topleftcorner, bottomrightcorner, bottomleftcorner;

			C2DPoint rectangle_center = point;

			C2DLine line_to_top(rectangle_center, m_robot.orientation);
			line_to_top.SetLength(m_robot.len / 2.0);

			C2DVector temp_orientation = m_robot.orientation;
			temp_orientation.TurnRight();
			C2DLine line_to_topright_corner(line_to_top.GetPointTo(), temp_orientation);
			line_to_topright_corner.SetLength(m_robot.width / 2.0);
			toprightcorner = line_to_topright_corner.GetPointTo();

			temp_orientation = m_robot.orientation;
			temp_orientation.TurnLeft();
			C2DLine line_to_topleft_corner(line_to_top.GetPointTo(), temp_orientation);
			line_to_topleft_corner.SetLength(m_robot.width / 2.0);
			topleftcorner = line_to_topleft_corner.GetPointTo();

			C2DVector opposite_orientation(-m_robot.orientation.i, -m_robot.orientation.j);
			C2DLine line_to_bottom(rectangle_center, opposite_orientation);
			line_to_bottom.SetLength(m_robot.len / 2.0);

			temp_orientation = opposite_orientation;
			temp_orientation.TurnRight();
			C2DLine line_to_bottomleft_corner(line_to_bottom.GetPointTo(), temp_orientation);
			line_to_bottomleft_corner.SetLength(m_robot.width / 2.0);
			bottomleftcorner = line_to_bottomleft_corner.GetPointTo();

			temp_orientation = opposite_orientation;
			temp_orientation.TurnLeft();
			C2DLine line_to_bottomright_corner(line_to_bottom.GetPointTo(), temp_orientation);
			line_to_bottomright_corner.SetLength(m_robot.width / 2.0);
			bottomrightcorner = line_to_bottomright_corner.GetPointTo();

			rec_top = C2DLine(topleftcorner, toprightcorner);
			rec_right = C2DLine(toprightcorner, bottomrightcorner);
			rec_bottom = C2DLine(bottomrightcorner, bottomleftcorner);
			rec_left = C2DLine(bottomleftcorner, topleftcorner);

			for(int w=0;w<polygon_obstacles.size();w++){
				if (polygon_obstacles[w].Crosses(rec_top) || polygon_obstacles[w].Crosses(rec_right) || polygon_obstacles[w].Crosses(rec_bottom) || polygon_obstacles[w].Crosses(rec_left)){
					if (polygon_obstacles[w].Crosses(rec_top))
						find_intersecting_line(polygon_obstacles[w], rec_top, intersectingObLine);
					else if (polygon_obstacles[w].Crosses(rec_right))
						find_intersecting_line(polygon_obstacles[w], rec_right, intersectingObLine);
					else if (polygon_obstacles[w].Crosses(rec_bottom))
						find_intersecting_line(polygon_obstacles[w], rec_bottom, intersectingObLine);
					else
						find_intersecting_line(polygon_obstacles[w], rec_left, intersectingObLine);
					return false;
				}
			}
			for(int w=0;w<line_obstacles.size();w++){
				if (line_obstacles[w].Crosses(rec_top) || line_obstacles[w].Crosses(rec_right) || line_obstacles[w].Crosses(rec_bottom) || line_obstacles[w].Crosses(rec_left)){
					intersectingObLine = line_obstacles[w];
					return false;
				}
			}

		}
		return true;
	}

	//check_dir == false (check point)
	else{
	bool is_inside_obs = false;
	bool is_inside_freespace = false;

	for (int w = 0; w < freespace.size(); w++){

		if (freespace[w].Contains(point))
			is_inside_freespace = true;
	}
	for (int w = 0; w < obstacles.size(); w++){

		if (obstacles[w].Contains(point))
			is_inside_obs = true;
	}

	isinsidespace = is_inside_freespace && !is_inside_obs;
	return isinsidespace;
}
}

//For a polygon that intersects with a given line,finds the edge of the polygon which causes the intersection (named polyline in function).
void SceneRobotEnvironment::find_intersecting_line(C2DPolygon poly, C2DLine line, C2DLine& polyline){

	for (int w = 0; w < poly.GetLineCount(); w++){

		if (poly.GetLine(w)->Crosses(line)){
			polyline = *poly.GetLine(w);
			return;
		}
	}


}


/* Application Entry Point */
int main(int argc, char* argv[])
{
	return vvr::mainLoop(argc, argv, new SceneRobotEnvironment);
}
