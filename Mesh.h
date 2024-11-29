#pragma once

#include <stdio.h>
#include <math.h>
#include <vector>

using namespace std;

struct Vertex
{
	int idx; //who am i; verts[idx]

	float curGeoDist; //current geo dist to the vertex during each geo dist v
	bool curProcessed; //if was processed in current geo dist calc
	Vertex* curGeoParent = NULL;
	vector<float>geoDistList; //this v's calculateDijkstra() result. Geo dist of this v to all others


	float curMinDistFPS; //to decide which FPS sample this v belongs to, ayni zamanda MGD.

	float avgGeoDist; // avg geo dist

	float* coords; //3d coordinates etc
	vector< int > vertList; //adj vvertices;
	vector< int > triList;
	vector< int > edgeList;

	Vertex(const Vertex&) = default;

	Vertex(int i, float* c) : idx(i), coords(c) { curGeoDist = 99999.0f; };
};

struct Edge
{
	int idx; //edges[idx]
	int v1i, v2i; //endpnts
	float length;
	Edge(int id, int v1, int v2) : idx(id), v1i(v1), v2i(v2) { computeLength(); };

	void computeLength()
	{
		length = 7;
	}
};

struct Triangle
{
	int idx; //tris[idx]
	int v1i, v2i, v3i;
	Triangle(int id, int v1, int v2, int v3) : idx(id), v1i(v1), v2i(v2), v3i(v3) {};
};

class Mesh
{
private:
	void addTriangle(int v1, int v2, int v3);
	void addEdge(int v1, int v2);
	void addVertex(float x, float y, float z);
	bool makeVertsNeighbor(int v1i, int v2i);
public:
	vector< Vertex* > verts;
	vector< Triangle* > tris;
	vector< Edge* > edges;
	vector< int > samples; //FPS samples

	float maxAvgGeoDist;
	float minAvgGeoDist = 99999.0f;

	Mesh() {};
	Mesh(const char* name);
	void calculateEdgeLengths();
	void loadOff(const char* name);
	void loadObj(const char* name);
};