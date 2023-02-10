#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include "Image.h"

// This allows you to skip the `std::` in front of C++ standard library
// functions. You can also say `using std::cout` to be more selective.
// You should never do this in a header file.
using namespace std;

double RANDOM_COLORS[7][3] = {
	{0.0000,    0.4470,    0.7410},
	{0.8500,    0.3250,    0.0980},
	{0.9290,    0.6940,    0.1250},
	{0.4940,    0.1840,    0.5560},
	{0.4660,    0.6740,    0.1880},
	{0.3010,    0.7450,    0.9330},
	{0.6350,    0.0780,    0.1840},
};

struct color {
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct norm {
	double x;
	double y;
	double z;
};

struct vertex {
	double x;
	double y;
	double z;
	color c;
	norm norm = {0,0,0};
};

double** zbuff(int &rows, int &cols, double& input) {
	double **array = new double* [cols];
	for (int i = 0; i < cols; i++) {
		array[i] = new double[rows];
		for (int j = 0; j < rows; j++) {
			array[i][j] = input;
		}
	}
	return array;
}

color** fbuff(int &rows, int &cols, color& c) {
	color **array = new color* [cols];
	for (int i = 0; i < cols; i++) {
		array[i] = new color[rows];
		for (int j = 0; j < rows; j++) {
			array[i][j].r = c.r;
			array[i][j].g = c.g;
			array[i][j].b = c.b;
		}
	}
	return array;
}

double find_scale(double ymax, double ymin, double xmax, double xmin, double height, double width) {
	double delta_yw = ymax - ymin; // obj
	double sy = (height-1) / delta_yw;

	double delta_xw = xmax - xmin; // obj
	double sx = (width - 1) / delta_xw;

	// take smaller measurement for limiting factor
	double limiting_factor = sy;
	if (sx < sy) {
		limiting_factor = sx;
	}
	return limiting_factor;
}

vertex translate(double width, double height, double limiting_factor, double xmin, double xmax, double ymin, double ymax) {
	// middle of image
	vertex mid_i = { (double)width / 2, (double)height / 2 };
	// middle of scaled obj
	vertex mid_obj = { limiting_factor * .5 * (double)(xmin + xmax), limiting_factor * .5 * ((double)ymin + (double)ymax) };

	//translation = middle of image -middle of scaled obj
	double translation_y = (double)(mid_i.y - mid_obj.y);
	double translation_x = mid_i.x - mid_obj.x;
	vertex translation = { translation_x, translation_y };
	return translation;
}

std::vector<double> barycentric(vertex a, vertex b, vertex c, vertex p) {
	// citation https://www.geogebra.org/m/ZuvmPjmy 
	double alphanum = (double)((p.y - c.y) * (b.x - c.x) + (p.x - c.x) * (c.y - b.y));
	double alphadenom = (double)((a.y - c.y) * (b.x - c.x) + (a.x - c.x) * (c.y - b.y));

	// barycentric 
	std::vector<double> bary(3);

	bary[0] = (double)alphanum / (double)alphadenom; //alpha

	double betanum = (double)(p.y - a.y)* (c.x - a.x) + (p.x-a.x)* (a.y-c.y);
	double betadenom = (double)(b.y - a.y) * (c.x - a.x) + (b.x - a.x) * (a.y - c.y);
	bary[1] = (double)betanum / (double)betadenom; //beta
	bary[2] = 1.0 - bary[0] - bary[1]; //gamma

	return bary;
}

int main(int argc, char **argv)
{
	if(argc < 2) {
		cout << "Usage: A1 meshfile" << endl;
		return 0;
	}
	string meshName(argv[1]);
	string fileName(argv[2]);

	int width = atoi(argv[3]);
	// Height of image
	int height = atoi(argv[4]);

	// Create the image. We're using a `shared_ptr`, a C++11 feature.
	auto image = make_shared<Image>(width, height);

	// Load geometry
	vector<float> posBuf; // list of vertex positions
	vector<float> norBuf; // list of vertex normals
	vector<float> texBuf; // list of vertex texture coords
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;
	string errStr;
	bool rc = tinyobj::LoadObj(&attrib, &shapes, &materials, &errStr, meshName.c_str());
	if(!rc) {
		cerr << errStr << endl;
	} else {
		// Some OBJ files have different indices for vertex positions, normals,
		// and texture coordinates. For example, a cube corner vertex may have
		// three different normals. Here, we are going to duplicate all such
		// vertices.
		// Loop over shapes
		for(size_t s = 0; s < shapes.size(); s++) {
			// Loop over faces (polygons)
			size_t index_offset = 0;
			for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
				size_t fv = shapes[s].mesh.num_face_vertices[f];
				// Loop over vertices in the face.
				for(size_t v = 0; v < fv; v++) {
					// access to vertex
					tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+0]);
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+1]);
					posBuf.push_back(attrib.vertices[3*idx.vertex_index+2]);
					if(!attrib.normals.empty()) {
						norBuf.push_back(attrib.normals[3*idx.normal_index+0]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+1]);
						norBuf.push_back(attrib.normals[3*idx.normal_index+2]);
					}
					if(!attrib.texcoords.empty()) {
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+0]);
						texBuf.push_back(attrib.texcoords[2*idx.texcoord_index+1]);
					}
				}
				index_offset += fv;
				// per-face material (IGNORE)
				shapes[s].mesh.material_ids[f];
			}
		}
	}
	// to get rid of small round-off errors
	double epsilon = .001;

	int task = atoi(argv[5]);

	// min and max x for image
	float xmin = posBuf[0];
	float xmax = posBuf[0];
	for (int i = 0; i < posBuf.size(); i += 3) {
		xmin = min(xmin, posBuf[i]);
		xmax = max(xmax, posBuf[i]);
	}
	// min and max y
	float ymin = posBuf[1];
	float ymax = posBuf[1];
	for (int i = 1; i < posBuf.size(); i += 3) {
		ymin = min(ymin, posBuf[i]);
		ymax = max(ymax, posBuf[i]);
	}
	// min and max z
	float zmin = posBuf[2];
	float zmax = posBuf[2];
	for (int i = 2; i < posBuf.size(); i += 3) {
		zmin = min(zmin, posBuf[i]);
		zmax = max(zmax, posBuf[i]);
	}

	// compute the scale and translation factors

	double limiting_factor = find_scale(ymax, ymin, xmax, xmin, height, width);
	vertex translation = translate(width, height, limiting_factor, xmin, xmax, ymin, ymax);

	if (task == 1) {
		
		//for every 9 vals/triangle
		for (int i = 0; i < posBuf.size(); i += 9) {
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i+1];
			float indiv_ymax = posBuf[i+1];
			
			//min and max y for 3 vertexes
			indiv_xmin = min(indiv_xmin, posBuf[i+3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);
			
			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			// new bounding box
			for (int y = new_miny - epsilon; y < new_maxy+ epsilon; ++y) {
				for (int x = new_minx- epsilon; x < new_maxx+ epsilon; ++x) {
					image->setPixel(x, y, 255 * RANDOM_COLORS[i / 9 % 7][0], 255 * RANDOM_COLORS[i / 9 % 7][1], 255 * RANDOM_COLORS[i / 9 % 7][2]);
				}
			}
		}
		
	}
	else if (task == 2) {
		
		//for every 9 vals/triangle
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;
			
			//vertex for box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i+1] +translation.y };
			vertex v2 = { limiting_factor * posBuf[i+3] + translation.x, limiting_factor * posBuf[i+4] + translation.y };
			vertex v3 = { limiting_factor * posBuf[i+6] + translation.x, limiting_factor * posBuf[i+7] + translation.y };

			// for all pixels
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {

					// compute barycentric coords
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);

					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {
						image->setPixel(x, y, 255 * RANDOM_COLORS[i/9 % 7][0], 255 * RANDOM_COLORS[i/9 % 7][1], 255 * RANDOM_COLORS[i/9 % 7][2]);
					}

				}
			}

		}
	}
	else if (task == 3) {
		//for every 9 vals/ triangle
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			//vertex for box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i + 1] + translation.y };
			vertex v2 = { limiting_factor * posBuf[i + 3] + translation.x, limiting_factor * posBuf[i + 4] + translation.y };
			vertex v3 = { limiting_factor * posBuf[i + 6] + translation.x, limiting_factor * posBuf[i + 7] + translation.y };

			//add colors to vertex
			v1.c.r = (unsigned char)(RANDOM_COLORS[i / 3 % 7][0] * 255.0);
			v1.c.g = (unsigned char)(RANDOM_COLORS[i / 3 % 7][1] * 255.0);
			v1.c.b = (unsigned char)(RANDOM_COLORS[i / 3 % 7][2] * 255.0);
			v2.c.r = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][0] * 255.0);
			v2.c.g = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][1] * 255.0);
			v2.c.b = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][2] * 255.0);
			v3.c.r = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][0] * 255.0);
			v3.c.g = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][1] * 255.0);
			v3.c.b = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][2] * 255.0);

			// for all pixels in triangle
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);
					// if abc b/t 0 and 1, inside
					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {
						// color
						unsigned char r = (unsigned char)(((double)(v1.c.r * bary[0]) + ((double)(v2.c.r * bary[1])) + ((double)(v3.c.r * bary[2]))));
						unsigned char g = (unsigned char)(((double)(v1.c.g * bary[0])) + ((double)(v2.c.g * bary[1])) + ((double)(v3.c.g * bary[2])));
						unsigned char b = (unsigned char)(((double)(v1.c.b * bary[0])) + ((double)(v2.c.b * bary[1])) + ((double)(v3.c.b * bary[2])));

						image->setPixel(x, y, r, g, b);
						
					}

				}
			}
		}
	}
	else if (task == 4) {
		// y of scaled large bounding box
		double bb_miny = limiting_factor * ymin + translation.y;
		double bb_maxy = limiting_factor * ymax + translation.y;

		//for every 9 vals/ individual meshes
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale indiv bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			// vertexes for bounding box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i + 1] + translation.y };
			vertex v2 = { limiting_factor * posBuf[i + 3] + translation.x, limiting_factor * posBuf[i + 4] + translation.y };
			vertex v3 = { limiting_factor * posBuf[i + 6] + translation.x, limiting_factor * posBuf[i + 7] + translation.y };

			// small bounding boxes
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);
					// if abc b/t 0 and 1, inside
					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {
						// Interpolate red and blue values
						double fraction = (double)(bb_maxy - y) / (double)(bb_maxy - bb_miny);
						unsigned char blue = fraction * 255;
						unsigned char red = (1 - fraction) * 255;
						image->setPixel(x, y, red, 0, blue);
					}
				}
			}
		}
	}
	else if (task == 5) {
		
		//create and initialize bufs
		color black = { 0, 0, 0 };
		color** fbuf = fbuff( width, height, black);
		double neg_infinity = -std::numeric_limits<double>::infinity();
		double** zbuf = zbuff(width, height, neg_infinity);

		// z of scaled large bounding box
		double bb_minz = limiting_factor * zmin;
		double bb_maxz = limiting_factor * zmax;

		//for every 9 vals/ triangle , individual meshes
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			// vertex for bounding box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i + 1] + translation.y, limiting_factor * posBuf[i+2] };
			vertex v2 = { limiting_factor * posBuf[i + 3] + translation.x, limiting_factor * posBuf[i + 4] + translation.y, limiting_factor * posBuf[i+5] };
			vertex v3 = { limiting_factor * posBuf[i + 6] + translation.x, limiting_factor * posBuf[i + 7] + translation.y, limiting_factor * posBuf[i+8] };

			//colors to vertex
			v1.c.r = (unsigned char)(RANDOM_COLORS[i / 3 % 7][0] * 255.0);
			v1.c.g = (unsigned char)(RANDOM_COLORS[i / 3 % 7][1] * 255.0);
			v1.c.b = (unsigned char)(RANDOM_COLORS[i / 3 % 7][2] * 255.0);
			v2.c.r = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][0] * 255.0);
			v2.c.g = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][1] * 255.0);
			v2.c.b = (unsigned char)(RANDOM_COLORS[(i + 3) / 3 % 7][2] * 255.0);
			v3.c.r = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][0] * 255.0);
			v3.c.g = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][1] * 255.0);
			v3.c.b = (unsigned char)(RANDOM_COLORS[(i + 6) / 3 % 7][2] * 255.0);

			//for each pixel in triangle bounding box
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);
					// if abc b/t 0 and 1, inside
					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {

						// compute zcoodinate
						double z = bary[0] * v1.z + bary[1] * v2.z + bary[2] * v3.z;

						//compute rgb color
						unsigned char red = (unsigned char)(((double)(v1.c.r * bary[0]) + ((double)(v2.c.r * bary[1])) + ((double)(v3.c.r * bary[2]))));
						unsigned char green = (unsigned char)(((double)(v1.c.g * bary[0])) + ((double)(v2.c.g * bary[1])) + ((double)(v3.c.g * bary[2])));
						unsigned char blue = (unsigned char)(((double)(v1.c.b * bary[0])) + ((double)(v2.c.b * bary[1])) + ((double)(v3.c.b * bary[2])));
						color c = { red, green, blue };
						if (z > zbuf[y][x]) {
							// set 
							fbuf[y][x] = c;
							zbuf[y][x] = z;
							image->setPixel(x, y, 255 * ((z - bb_minz) / (bb_maxz - bb_minz)), 0, 0);
						}
					}
				}
			}
		}
	}
	else if (task == 6) {
		//create and initialize bufs
		color black = { 0, 0, 0 };
		color** fbuf = fbuff(width, height, black);
		double neg_infinity = -std::numeric_limits<double>::infinity();
		double** zbuf = zbuff(width, height, neg_infinity);

		// y of scaled large bounding box
		double bb_minz = limiting_factor * zmin;
		double bb_maxz = limiting_factor * zmax;

		//for every 9 vals/ triangle , individual meshes
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			// vertex for bounding box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i + 1] + translation.y, limiting_factor * posBuf[i + 2] };
			vertex v2 = { limiting_factor * posBuf[i + 3] + translation.x, limiting_factor * posBuf[i + 4] + translation.y, limiting_factor * posBuf[i + 5] };
			vertex v3 = { limiting_factor * posBuf[i + 6] + translation.x, limiting_factor * posBuf[i + 7] + translation.y, limiting_factor * posBuf[i + 8] };

			//store norbufs
			v1.norm.x = norBuf[i];
			v1.norm.y = norBuf[i+1];
			v1.norm.z = norBuf[i+2];
			v2.norm.x = norBuf[i+3];
			v2.norm.y = norBuf[i + 4];
			v2.norm.z = norBuf[i + 5];
			v3.norm.x = norBuf[i + 6];
			v3.norm.y = norBuf[i + 7];
			v3.norm.z = norBuf[i + 8];

			//for each pixel in triangle bounding box
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);
					// if abc b/t 0 and 1, inside
					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {
						// compute zcoodinate
						double z = bary[0] * v1.z + bary[1] * v2.z + bary[2] * v3.z;
						
						//compute the normal of pixel
						pixel.norm.x = bary[0] * v1.norm.x + bary[1] * v2.norm.x + bary[2] * v3.norm.x;
						pixel.norm.y = bary[0] * v1.norm.y + bary[1] * v2.norm.y + bary[2] * v3.norm.y;
						pixel.norm.z = bary[0] * v1.norm.z + bary[1] * v2.norm.z + bary[2] * v3.norm.z;

						//compute rgb color
						unsigned char r = (unsigned char)255 * (0.5 * pixel.norm.x + 0.5);
						unsigned char g = (unsigned char)255 * (0.5 * pixel.norm.y + 0.5);
						unsigned char b = (unsigned char)255 * (0.5 * pixel.norm.z + 0.5);
						color c = { r, g, b };
						if (z > zbuf[y][x]) {
							// set 
							fbuf[y][x] = c;
							zbuf[y][x] = z;
							image->setPixel(x, y, r, g, b);
						}
					}
				}
			}
		}
	}
	else if (task == 7) {
		//create and initialize bufs
		color black = { 0, 0, 0 };
		color** fbuf = fbuff(width, height, black);
		double neg_infinity = -std::numeric_limits<double>::infinity();
		double** zbuf = zbuff(width, height, neg_infinity);

		// y of scaled large bounding box
		double bb_minz = limiting_factor * zmin;
		double bb_maxz = limiting_factor * zmax;

		//for every 9 vals/ triangle, individual meshes
		for (int i = 0; i < posBuf.size(); i += 9) {
			//min and max y for 3 vertexes
			float indiv_xmin = posBuf[i];
			float indiv_xmax = posBuf[i];
			float indiv_ymin = posBuf[i + 1];
			float indiv_ymax = posBuf[i + 1];

			indiv_xmin = min(indiv_xmin, posBuf[i + 3]);
			indiv_xmin = min(indiv_xmin, posBuf[i + 6]);

			indiv_xmax = max(indiv_xmax, posBuf[i + 3]);
			indiv_xmax = max(indiv_xmax, posBuf[i + 6]);


			indiv_ymin = min(indiv_ymin, posBuf[i + 4]);
			indiv_ymin = min(indiv_ymin, posBuf[i + 7]);

			indiv_ymax = max(indiv_ymax, posBuf[i + 4]);
			indiv_ymax = max(indiv_ymax, posBuf[i + 7]);

			//scale bounding box
			double new_minx = limiting_factor * indiv_xmin + translation.x;
			double new_miny = limiting_factor * indiv_ymin + translation.y;
			double new_maxx = limiting_factor * indiv_xmax + translation.x;
			double new_maxy = limiting_factor * indiv_ymax + translation.y;

			// vertexes for box
			vertex v1 = { limiting_factor * posBuf[i] + translation.x, limiting_factor * posBuf[i + 1] + translation.y, limiting_factor * posBuf[i + 2] };
			vertex v2 = { limiting_factor * posBuf[i + 3] + translation.x, limiting_factor * posBuf[i + 4] + translation.y, limiting_factor * posBuf[i + 5] };
			vertex v3 = { limiting_factor * posBuf[i + 6] + translation.x, limiting_factor * posBuf[i + 7] + translation.y, limiting_factor * posBuf[i + 8] };

			//store norbufs
			v1.norm.x = norBuf[i];
			v1.norm.y = norBuf[i + 1];
			v1.norm.z = norBuf[i + 2];
			v2.norm.x = norBuf[i + 3];
			v2.norm.y = norBuf[i + 4];
			v2.norm.z = norBuf[i + 5];
			v3.norm.x = norBuf[i + 6];
			v3.norm.y = norBuf[i + 7];
			v3.norm.z = norBuf[i + 8];

			//for each pixel in triangle bounding box
			for (int y = new_miny - epsilon; y < new_maxy + epsilon; ++y) {
				for (int x = new_minx - epsilon; x < new_maxx + epsilon; ++x) {
					vertex pixel = { x,y };
					vector<double> bary = barycentric(v1, v2, v3, pixel);
					// if abc b/t 0 and 1, inside
					if (0 <= bary[0] && bary[0] <= 1 && 0 <= bary[1] && bary[1] <= 1 && 0 <= bary[2] && bary[2] <= 1) {
						// compute zcoodinate
						double z = bary[0] * v1.z + bary[1] * v2.z + bary[2] * v3.z;

						//compute normal of pixel
						pixel.norm.x = bary[0] * v1.norm.x + bary[1] * v2.norm.x + bary[2] * v3.norm.x;
						pixel.norm.y = bary[0] * v1.norm.y + bary[1] * v2.norm.y + bary[2] * v3.norm.y;
						pixel.norm.z = bary[0] * v1.norm.z + bary[1] * v2.norm.z + bary[2] * v3.norm.z;

						//compute rgb color + lighting
						vector<double> l = { sqrt(3) / 3 , sqrt(3) / 3 , sqrt(3) / 3 };
						double dot = pixel.norm.x * l[0] + pixel.norm.y * l[1] + pixel.norm.z * l[2];
						unsigned char r = 255.0 * max(dot, 0.0);
						unsigned char g = 255.0 * max(dot, 0.0);
						unsigned char b = 255.0 * max(dot, 0.0);

						color c = { r, g, b };
						if (z > zbuf[y][x]) {
							// set 
							fbuf[y][x] = c;
							zbuf[y][x] = z;
							image->setPixel(x, y, r, g, b);
						}
					}
				}
			}
		}
	}


	image->writeToFile(fileName);
	
	cout << "Number of vertices: " << posBuf.size()/3 << endl;
	
	return 0;
}
